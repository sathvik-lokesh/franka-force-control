"""Cartesian impedance controller for the Panda, torque level.

    tau = J^T (K e - D xdot) + c(q, qdot) + N^T tau_null

e is the 6D pose error at the TCP (position, then orientation as a rotation
vector), c is MuJoCo's qfrc_bias (gravity + Coriolis), and tau_null is a joint
PD toward a rest posture, projected into the nullspace so it doesn't disturb
the TCP. This is the same structure as franka_ros' cartesian_impedance_example.
"""
from dataclasses import dataclass, field

import mujoco
import numpy as np


def quat_error(q_des: np.ndarray, q_cur: np.ndarray) -> np.ndarray:
    """Rotation vector (world frame) taking q_cur to q_des."""
    q_cur_conj = np.zeros(4)
    mujoco.mju_negQuat(q_cur_conj, q_cur)
    q_err = np.zeros(4)
    mujoco.mju_mulQuat(q_err, q_des, q_cur_conj)
    rotvec = np.zeros(3)
    mujoco.mju_quat2Vel(rotvec, q_err, 1.0)
    return rotvec


@dataclass
class ImpedanceGains:
    k_pos: float = 500.0          # N/m
    k_rot: float = 50.0           # Nm/rad
    k_null: float = 10.0          # Nm/rad
    # Damping ratio 1 for a unit-mass approximation: d = 2 sqrt(k)
    d_pos: float | None = None
    d_rot: float | None = None
    d_null: float | None = None
    q_rest: np.ndarray = field(default_factory=lambda: np.zeros(7))

    def __post_init__(self):
        self.d_pos = 2 * np.sqrt(self.k_pos) if self.d_pos is None else self.d_pos
        self.d_rot = 2 * np.sqrt(self.k_rot) if self.d_rot is None else self.d_rot
        self.d_null = 2 * np.sqrt(self.k_null) if self.d_null is None else self.d_null


class CartesianImpedance:
    """Computes arm torques for a desired TCP pose. Call step() once per physics step."""

    def __init__(self, model: mujoco.MjModel, gains: ImpedanceGains, site: str = "tcp",
                 believed: mujoco.MjModel | None = None):
        """believed: the model the controller trusts for bias, Jacobian and inertia, when it differs
        from the true one (M4). Measured joint state still comes from the true data."""
        self.model = believed if believed is not None else model
        self.believed_data = mujoco.MjData(believed) if believed is not None else None
        self.g = gains
        self.site_id = model.site(site).id
        self.dof = np.arange(7)
        self.jacp = np.zeros((3, model.nv))
        self.jacr = np.zeros((3, model.nv))
        self.K = np.diag([gains.k_pos] * 3 + [gains.k_rot] * 3)
        self.D = np.diag([gains.d_pos] * 3 + [gains.d_rot] * 3)

    def tcp_pose(self, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
        pos = data.site_xpos[self.site_id].copy()
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, data.site_xmat[self.site_id])
        return pos, quat

    def jacobian(self, data: mujoco.MjData) -> np.ndarray:
        mujoco.mj_jacSite(self.model, data, self.jacp, self.jacr, self.site_id)
        return np.vstack([self.jacp[:, self.dof], self.jacr[:, self.dof]])

    def step(self, data: mujoco.MjData, pos_des: np.ndarray, quat_des: np.ndarray,
             wrench_override: dict[int, float] | None = None,
             vel_des: np.ndarray | None = None,
             wrench_ff: np.ndarray | None = None) -> np.ndarray:
        """Arm torques for the desired pose.

        wrench_override maps a Cartesian axis index (0..5, world frame) to a commanded
        wrench component that replaces the spring term on that axis. Damping stays on
        every axis. This is how the force controller takes over z while x/y/rot stay
        impedance.

        vel_des (6,) is the desired TCP twist; damping acts on the velocity error, so a
        moving reference isn't dragged by D*v. None = 0 (pure regulation, as in M0/M1).

        wrench_ff (6,) is added to the commanded wrench as is (e.g. cancelling the measured
        contact friction so it doesn't leak into other axes through the arm's inertia).
        """
        if self.believed_data is not None:
            self.believed_data.qpos[:] = data.qpos
            self.believed_data.qvel[:] = data.qvel
            mujoco.mj_forward(self.model, self.believed_data)
            data = self.believed_data
        pos, quat = self.tcp_pose(data)
        J = self.jacobian(data)
        qd = data.qvel[self.dof]

        e = np.concatenate([pos_des - pos, quat_error(quat_des, quat)])
        spring = self.K @ e
        for axis, value in (wrench_override or {}).items():
            spring[axis] = value
        v_err = J @ qd if vel_des is None else J @ qd - vel_des
        wrench = spring - self.D @ v_err
        if wrench_ff is not None:
            wrench = wrench + wrench_ff
        tau_task = J.T @ wrench

        # Dynamically consistent nullspace projector N = I - J^T Jbar^T
        M = np.zeros((self.model.nv, self.model.nv))
        mujoco.mj_fullM(self.model, data, M)  # MuJoCo >= 3.17 signature
        Minv = np.linalg.inv(M[:7, :7])
        Lambda = np.linalg.pinv(J @ Minv @ J.T)
        Jbar = Minv @ J.T @ Lambda
        N = np.eye(7) - J.T @ Jbar.T
        q = data.qpos[self.dof]
        tau_null = N @ (self.g.k_null * (self.g.q_rest - q) - self.g.d_null * qd)

        return tau_task + tau_null + data.qfrc_bias[self.dof]
