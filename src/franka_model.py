"""Load the Menagerie Panda and turn its 7 arm actuators into torque motors.

Menagerie ships the arm with position servos (gainprm/biasprm = a built-in PD).
For impedance control we want ctrl = joint torque, so each arm actuator is
rewritten as a unit-gain motor clipped to the real Panda torque limits.
The gripper (actuator8) is left as Menagerie defines it.
"""
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PANDA_DIR = ROOT / "third_party" / "mujoco_menagerie" / "franka_emika_panda"

# Franka datasheet joint torque limits [Nm]
TAU_MAX = np.array([87, 87, 87, 87, 12, 12, 12], dtype=float)
# Flange (Menagerie "hand" body) to TCP between the fingertips [m]
TCP_OFFSET = 0.1034
HOME_Q = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])


def build_model(scene: str = "scene.xml", timestep: float = 0.001, task: str | None = None,
                table_top: float = 0.45, table_tilt_deg: float = 0.0,
                friction: float | None = None, contact_dmin: float | None = None,
                hole_xy: tuple[float, float] = (0.55, 0.0), plate_top: float = 0.43,
                hole_yaw_deg: float = 0.0) -> mujoco.MjModel:
    """Compile the Panda scene with torque-controlled arm joints and a TCP site.

    task="press" adds a ball-tipped probe tool, a flange F/T sensor and a table.
    table_tilt_deg rotates the table about the world x axis through (0.55, 0, table_top), so
    the surface height at y is table_top + y*tan(tilt) and the start point is unchanged.
    friction sets the sliding friction of ball and table (None = MuJoCo default 1.0).
    contact_dmin softens the ball-table contact (solimp dmin, MuJoCo default 0.9). 0.2 models a
    compliant pad: ~5e4 N/m measured at 10 N, friction still Coulomb within 3% (see PLAN M2).

    task="peg" adds a square peg, the same flange F/T sensor and a plate with a blind square hole
    centred at hole_xy, top face at plate_top. friction and contact_dmin apply to peg and plate.
    """
    spec = mujoco.MjSpec.from_file(str(PANDA_DIR / scene))
    spec.option.timestep = timestep

    for i in range(7):
        act = spec.actuator(f"actuator{i + 1}")
        act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        act.gainprm[:] = 0
        act.gainprm[0] = 1.0
        act.biastype = mujoco.mjtBias.mjBIAS_NONE
        act.biasprm[:] = 0
        act.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
        act.ctrlrange = [-TAU_MAX[i], TAU_MAX[i]]
        act.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
        act.forcerange = [-TAU_MAX[i], TAU_MAX[i]]

    spec.body("hand").add_site(name="tcp", pos=[0, 0, TCP_OFFSET], size=[0.005] * 3)
    if task == "press":
        _add_press_task(spec, table_top, table_tilt_deg, friction, contact_dmin)
    elif task == "peg":
        _add_peg_task(spec, hole_xy, plate_top, friction, contact_dmin, hole_yaw_deg)
    return spec.compile()


# Probe tool: rod + ball tip on the flange, longer than the fingers so only the ball touches.
PROBE_RADIUS = 0.01
PROBE_CENTER = 0.12           # flange -> ball centre [m]; ball bottom at 0.13, fingertips end ~0.11
PROBE_MASS = 0.1
TABLE_TOP = 0.45              # world z of the table surface [m]; probe bottom is at 0.4945 at home


def _add_press_task(spec: mujoco.MjSpec, table_top: float, tilt_deg: float,
                    friction: float | None, contact_dmin: float | None) -> None:
    hand = spec.body("hand")
    # Geoms live on the hand body itself, so the flange sensor sees them as "tool below the sensor".
    hand.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[0, 0, 0.06, 0, 0, PROBE_CENTER],
                  size=[0.006, 0, 0], mass=PROBE_MASS * 0.6, contype=0, conaffinity=0,
                  rgba=[0.2, 0.2, 0.2, 1])
    hand.add_geom(name="probe_ball", type=mujoco.mjtGeom.mjGEOM_SPHERE, pos=[0, 0, PROBE_CENTER],
                  size=[PROBE_RADIUS, 0, 0], mass=PROBE_MASS * 0.4, rgba=[0.9, 0.3, 0.1, 1])
    hand.add_site(name="probe", pos=[0, 0, PROBE_CENTER], size=[0.004] * 3)
    hand.add_site(name="ft", pos=[0, 0, 0], size=[0.004] * 3)
    spec.add_sensor(name="ft_force", type=mujoco.mjtSensor.mjSENS_FORCE,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="ft")
    spec.add_sensor(name="ft_torque", type=mujoco.mjtSensor.mjSENS_TORQUE,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="ft")

    half_h = table_top / 2
    a = np.deg2rad(tilt_deg) / 2
    table = spec.worldbody.add_body(name="table", pos=[0.55, 0, table_top],
                                    quat=[np.cos(a), np.sin(a), 0, 0])
    table.add_geom(name="table_top", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.3, 0.4, half_h],
                   pos=[0, 0, -half_h], rgba=[0.6, 0.5, 0.4, 1])
    if friction is not None:
        for g in ("table_top", "probe_ball"):
            spec.geom(g).friction = [friction, 0.005, 0.0001]
    if contact_dmin is not None:
        # MuJoCo mixes the two geoms' solimp, so set both to get exactly this value.
        for g in ("table_top", "probe_ball"):
            spec.geom(g).solimp = [contact_dmin, 0.95, 0.001, 0.5, 2.0]


# Peg task geometry [m]. Peg tip is 50 mm past the fingertips, so the fingers clear the 20 mm hole.
PEG_SIDE = 0.020
PEG_TOP = 0.06                # flange -> peg upper end
PEG_TIP = 0.16                # flange -> peg tip
PEG_MASS = 0.1                # has no effect: the hand's explicit <inertial> overrides geom mass (see PLAN M4)
HOLE_SIDE = 0.021             # 1 mm total clearance, no chamfer
HOLE_DEPTH = 0.020
PLATE_HALF = 0.06             # outer half-width; peg stays on the plate for any r + spiral used here


def _add_peg_task(spec: mujoco.MjSpec, hole_xy: tuple[float, float], plate_top: float,
                  friction: float | None, contact_dmin: float | None, yaw_deg: float = 0.0) -> None:
    hand = spec.body("hand")
    half_len = (PEG_TIP - PEG_TOP) / 2
    hand.add_geom(name="peg", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[0, 0, PEG_TOP + half_len],
                  size=[PEG_SIDE / 2, PEG_SIDE / 2, half_len], mass=PEG_MASS, rgba=[0.9, 0.3, 0.1, 1])
    hand.add_site(name="peg_tip", pos=[0, 0, PEG_TIP], size=[0.004] * 3)
    hand.add_site(name="ft", pos=[0, 0, 0], size=[0.004] * 3)
    spec.add_sensor(name="ft_force", type=mujoco.mjtSensor.mjSENS_FORCE,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="ft")
    spec.add_sensor(name="ft_torque", type=mujoco.mjtSensor.mjSENS_TORQUE,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="ft")

    a_yaw = np.deg2rad(yaw_deg) / 2
    plate = spec.worldbody.add_body(name="plate", pos=[hole_xy[0], hole_xy[1], plate_top],
                                    quat=[np.cos(a_yaw), 0, 0, np.sin(a_yaw)])
    h, a, d = HOLE_SIDE / 2, PLATE_HALF, HOLE_DEPTH
    wall = (a - h) / 2                # half thickness of each wall around the hole
    mid = h + wall
    boxes = {
        "wall_xp": ([mid, 0, -d / 2], [wall, a, d / 2]),
        "wall_xn": ([-mid, 0, -d / 2], [wall, a, d / 2]),
        "wall_yp": ([0, mid, -d / 2], [h, wall, d / 2]),
        "wall_yn": ([0, -mid, -d / 2], [h, wall, d / 2]),
        "hole_floor": ([0, 0, -d - (plate_top - d) / 2], [a, a, (plate_top - d) / 2]),
    }
    for name, (pos, size) in boxes.items():
        plate.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, pos=pos, size=size,
                       rgba=[0.6, 0.5, 0.4, 1])
    for g in ("peg", *boxes):
        if friction is not None:
            spec.geom(g).friction = [friction, 0.005, 0.0001]
        if contact_dmin is not None:
            spec.geom(g).solimp = [contact_dmin, 0.95, 0.001, 0.5, 2.0]


def reset_home(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Put the arm at Menagerie's home keyframe with the gripper open."""
    key = model.key("home").id
    mujoco.mj_resetDataKeyframe(model, data, key)
    data.ctrl[:7] = 0.0
    mujoco.mj_forward(model, data)
