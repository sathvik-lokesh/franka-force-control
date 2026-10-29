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


def build_model(scene: str = "scene.xml", timestep: float = 0.001) -> mujoco.MjModel:
    """Compile the Panda scene with torque-controlled arm joints and a TCP site."""
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
    return spec.compile()


def reset_home(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Put the arm at Menagerie's home keyframe with the gripper open."""
    key = model.key("home").id
    mujoco.mj_resetDataKeyframe(model, data, key)
    data.ctrl[:7] = 0.0
    mujoco.mj_forward(model, data)
