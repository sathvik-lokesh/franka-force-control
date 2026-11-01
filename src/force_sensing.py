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

    def __init__(self, model: mujoco.MjModel, site: str = "ft"):
        self.model = model
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
        return sign * (self.raw_world(data) - self.tare_world)


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
