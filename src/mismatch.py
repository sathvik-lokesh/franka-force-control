"""Model mismatch for M4: the world stops matching what the controller believes.

Mismatch() with all defaults changes nothing, so M1-M3 reproduce exactly.
"""
import copy
from collections import deque
from dataclasses import asdict, dataclass

import mujoco
import numpy as np

from force_sensing import FlangeFT
from franka_model import PEG_TIP, PEG_TOP

LP_HZ = 100.0     # F/T low-pass, on whenever there is noise


@dataclass
class Mismatch:
    payload_kg: float = 0.0        # true tool mass minus believed
    joint_friction_Nm: float = 0.0 # Coulomb, all 7 arm joints
    ft_noise_N: float = 0.0        # sigma per axis
    ft_bias_N: float = 0.0         # z, appears after the tare
    delay_ms: int = 0              # torque command delay, whole physics steps
    hole_yaw_deg: float = 0.0      # M3 only
    seed: int = 0                  # noise stream

    def as_dict(self) -> dict:
        return asdict(self)

    def apply(self, model: mujoco.MjModel) -> mujoco.MjModel | None:
        """Change the true model in place. Returns the believed model, or None if they still match."""
        nominal = None
        if self.payload_kg:
            nominal = copy.copy(model)
            nominal.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
            _change_tool_mass(model, self.payload_kg)
        if self.joint_friction_Nm:
            model.dof_frictionloss[:7] = self.joint_friction_Nm
        return nominal

    def make_ft(self, model: mujoco.MjModel) -> FlangeFT:
        return FlangeFT(model, noise_std=self.ft_noise_N, bias_z=self.ft_bias_N,
                        lp_hz=LP_HZ if self.ft_noise_N else None, seed=self.seed)

    def make_delay(self) -> "CommandDelay":
        return CommandDelay(self.delay_ms)


def _change_tool_mass(model: mujoco.MjModel, dm: float) -> None:
    """Add dm to the hand body at the middle of the tool. Geom mass is ignored on this body because the
    hand has an explicit <inertial>, so body_mass is edited directly. Rotational inertia scales with
    mass (a coarse approximation; the gravity moment comes from the COM shift, which is exact)."""
    b = model.body("hand").id
    m0 = float(model.body_mass[b])
    assert m0 + dm > 0.05, "tool mass would go non-positive"
    tool = np.array([0.0, 0.0, (PEG_TOP + PEG_TIP) / 2])
    model.body_ipos[b] = (m0 * model.body_ipos[b] + dm * tool) / (m0 + dm)
    model.body_mass[b] = m0 + dm
    model.body_inertia[b] *= (m0 + dm) / m0
    mujoco.mj_setConst(model, mujoco.MjData(model))


class CommandDelay:
    """Holds torque commands back by n physics steps. Starts filled with the first command, so a delay
    doesn't look like the arm being dropped at t = 0."""

    def __init__(self, n: int):
        self.n = n
        self.q: deque | None = None

    def __call__(self, tau: np.ndarray) -> np.ndarray:
        if self.n == 0:
            return tau
        if self.q is None:
            self.q = deque([tau.copy() for _ in range(self.n)], maxlen=self.n)
        out = self.q[0].copy()
        self.q.append(tau.copy())
        return out
