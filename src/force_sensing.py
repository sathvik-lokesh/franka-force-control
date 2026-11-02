"""Flange F/T sensor processing and ground-truth contact force.

The MuJoCo force sensor on the flange site measures the interaction between the
hand (tool) and link7, in the site frame. It contains the tool's weight and
inertia as well as the contact, exactly like a real F/T sensor, so it has to be
tared in free space before it says anything about contact.
"""
import mujoco
import numpy as np


class FlangeFT:
    """Reads the flange sensor, rotates it to world frame and removes the tare."""

    def __init__(self, model: mujoco.MjModel, site: str = "ft", noise_std: float = 0.0,
                 bias_z: float = 0.0, lp_hz: float | None = None, seed: int = 0):
        """noise_std, bias_z and lp_hz stay off for M1-M3. Call contact_force once per control tick."""
        self.model = model
        self.noise_std = noise_std
        self.bias_z = bias_z
        self.rng = np.random.default_rng(seed)
        self.alpha = None
        if lp_hz is not None:
            rc = 1.0 / (2 * np.pi * lp_hz)
            self.alpha = model.opt.timestep / (rc + model.opt.timestep)
        self.filtered = None
        self.site_id = model.site(site).id
        s = model.sensor("ft_force")
        self.adr = s.adr[0]
        self.tare_world = np.zeros(3)

    def raw_world(self, data: mujoco.MjData) -> np.ndarray:
        """Sensor force in world frame (still includes tool weight)."""
        f_site = data.sensordata[self.adr:self.adr + 3]
        return data.site_xmat[self.site_id].reshape(3, 3) @ f_site

    def tare(self, data: mujoco.MjData) -> None:
        """Zero the sensor with the tool hanging free and still."""
        self.tare_world = self.raw_world(data).copy()

    def contact_force(self, data: mujoco.MjData, sign: float = -1.0) -> np.ndarray:
        """Estimated force the environment pushes on the tool, world frame.

        sign = -1 was checked against true_contact_force (2026-11-01): MuJoCo reports the
        force the child exerts on the parent, not the other way round.
        """
        f = sign * (self.raw_world(data) - self.tare_world)
        if self.noise_std:
            f = f + self.rng.normal(0.0, self.noise_std, 3)
        if self.bias_z:
            f = f + np.array([0.0, 0.0, self.bias_z])
        if self.alpha is not None:
            self.filtered = f if self.filtered is None else self.filtered + self.alpha * (f - self.filtered)
            return self.filtered.copy()
        return f


class FreeSpaceZero:
    """M4 v2 contact detector. Tracks the free-space F/T reading and only calls contact when the
    reading rises above that baseline for several ticks in a row, so a bias that appeared after the
    tare, or noise, doesn't look like a touch. The baseline freezes at contact and stays subtracted.
    A bias that appears after contact is not handled."""

    def __init__(self, dt: float, tau: float = 0.2, warmup: float = 0.05, thresh: float = 1.0, ticks: int = 5):
        self.a = dt / tau
        self.warmup = int(warmup / dt)
        self.thresh = thresh
        self.ticks = ticks
        self.n = 0
        self.count = 0
        self.baseline = 0.0
        self.in_contact = False

    def update(self, f: float) -> tuple[float, bool]:
        """Reading along the contact axis -> (reading minus baseline, contact yet?)."""
        if self.in_contact:
            return f - self.baseline, True
        self.n += 1
        if self.n <= self.warmup:
            self.baseline += (f - self.baseline) / self.n
            return 0.0, False
        rel = f - self.baseline
        if rel > self.thresh:
            self.count += 1
            self.in_contact = self.count >= self.ticks
        else:
            self.count = 0
            self.baseline += self.a * rel
        return rel, self.in_contact


def true_contact_force(model: mujoco.MjModel, data: mujoco.MjData, geom: str) -> np.ndarray:
    """Sum of all contact forces acting ON the given geom, world frame (ground truth)."""
    gid = model.geom(geom).id
    total = np.zeros(3)
    f6 = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        if gid not in (c.geom1, c.geom2):
            continue
        mujoco.mj_contactForce(model, data, i, f6)
        # f6[:3] is in the contact frame; rows of c.frame are the frame axes in world.
        f_world = c.frame.reshape(3, 3).T @ f6[:3]
        # MuJoCo: the contact normal points geom1 -> geom2 and the force acts on geom2 along it.
        total += f_world if c.geom2 == gid else -f_world
    return total
