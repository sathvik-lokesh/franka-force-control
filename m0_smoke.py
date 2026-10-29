"""M0 smoke test: does the torque-controlled Panda behave like physics says it should?

Test A: gravity compensation only (tau = qfrc_bias). The arm should hold the
        home pose; any drift means the model or torque path is wrong.
Test B: Cartesian impedance hold + a known external force F at the TCP.
        Steady-state deflection along that axis should be F / k_pos.
"""
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).parent / "src"))
from franka_model import HOME_Q, build_model, reset_home  # noqa: E402
from impedance import CartesianImpedance, ImpedanceGains  # noqa: E402

RESULTS = Path(__file__).parent / "results"


def test_gravity_hold(model, seconds=3.0):
    data = mujoco.MjData(model)
    reset_home(model, data)
    q0 = data.qpos[:7].copy()
    for _ in range(int(seconds / model.opt.timestep)):
        data.ctrl[:7] = data.qfrc_bias[:7]
        mujoco.mj_step(model, data)
    drift = np.abs(data.qpos[:7] - q0)
    return {"seconds": seconds, "max_joint_drift_rad": float(drift.max())}


def test_stiffness(model, force=10.0, k_pos=500.0, settle=2.0, hold=1.0):
    out = {}
    for axis, name in enumerate("xyz"):
        data = mujoco.MjData(model)
        reset_home(model, data)
        ctl = CartesianImpedance(model, ImpedanceGains(k_pos=k_pos, q_rest=HOME_Q.copy()))
        pos_des, quat_des = ctl.tcp_pose(data)
        hand = model.body("hand").id
        f = np.zeros(3)
        f[axis] = force

        steps_settle = int(settle / model.opt.timestep)
        steps_hold = int(hold / model.opt.timestep)
        samples = []
        for i in range(steps_settle + steps_hold):
            data.qfrc_applied[:] = 0
            # Apply F at the TCP point (not the body COM) so it adds no moment about the TCP
            mujoco.mj_applyFT(model, data, f, np.zeros(3), data.site_xpos[ctl.site_id],
                              hand, data.qfrc_applied)
            data.ctrl[:7] = ctl.step(data, pos_des, quat_des)
            mujoco.mj_step(model, data)
            if i >= steps_settle:
                samples.append(data.site_xpos[ctl.site_id][axis] - pos_des[axis])

        measured = float(np.mean(samples))
        expected = force / k_pos
        out[name] = {
            "expected_m": expected,
            "measured_m": measured,
            "rel_error": abs(measured - expected) / expected,
        }
    return out


def main():
    model = build_model()
    assert np.all(model.actuator_biastype[:7] == mujoco.mjtBias.mjBIAS_NONE), "arm still has servo bias"
    assert np.allclose(model.actuator_gainprm[:7, 0], 1.0), "arm actuator gain is not 1"

    res = {
        "timestep_s": model.opt.timestep,
        "gravity_hold": test_gravity_hold(model),
        "stiffness_10N_k500": test_stiffness(model),
    }
    res["pass"] = {
        "gravity_hold": res["gravity_hold"]["max_joint_drift_rad"] < 1e-3,
        "stiffness": all(v["rel_error"] < 0.05 for v in res["stiffness_10N_k500"].values()),
    }
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "m0_smoke.json").write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
