"""M5: record the Python M1 force run, build the C++ port, compare torques and measure timing.

Bars are fixed in PLAN.md (M5) before this was run. Writes results/m5_cpp.json.

Trace record, 135 doubles per tick: pos 3, quat 4, J 42 (row-major), M 49 (row-major), q 7, qd 7,
bias 7, pos_des 3, quat_des 4, has_override 1, override 1, tau 7.
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).parent
sys.path.insert(0, str(ROOT / "src"))
import m1_press as m1  # noqa: E402
from force_sensing import FlangeFT, true_contact_force  # noqa: E402
from franka_model import HOME_Q, build_model, reset_home  # noqa: E402
from impedance import CartesianImpedance, ImpedanceGains  # noqa: E402

BUILD = ROOT / "cpp" / "build"
RESULTS = ROOT / "results"
RT_SECONDS = 60


def record() -> dict:
    """The M1 force loop with the controller's inputs recorded. Checked equal to m1_press.run below."""
    model = build_model(task="press")
    mujoco.mj_saveModel(model, str(BUILD / "press.mjb"), None)
    data = mujoco.MjData(model)
    reset_home(model, data)
    data.ctrl[7] = 255
    ctl = CartesianImpedance(model, ImpedanceGains(k_pos=m1.K_POS, q_rest=HOME_Q.copy()), site="probe")
    ft = FlangeFT(model)
    pi = m1.ForcePI()
    dt = model.opt.timestep
    p0, quat0 = ctl.tcp_pose(data)
    tare_steps = int(m1.TARE_TIME / dt)
    t_contact, switched = None, False
    rows, zs, times = [], [], []
    M = np.zeros((model.nv, model.nv))
    for step in range(int(15.0 / dt)):
        t = step * dt
        if step == tare_steps:
            ft.tare(data)
        z_des = p0[2] - m1.V_APPROACH * max(0.0, t - m1.TARE_TIME)
        pos_des = np.array([p0[0], p0[1], z_des])
        f_est = ft.contact_force(data)[2] if step >= tare_steps else 0.0
        override = None
        if not switched and step > tare_steps and f_est > m1.CONTACT_THRESH:
            switched = True
            pi.reset(max(0.0, m1.K_POS * (data.site_xpos[ctl.site_id][2] - z_des)), t)
        if switched:
            override = {2: pi.wrench_z(f_est, t, dt)}

        pos, quat = ctl.tcp_pose(data)
        J = ctl.jacobian(data)
        mujoco.mj_fullM(model, data, M)
        inputs = [pos, quat, J.ravel(), M[:7, :7].ravel(), data.qpos[:7], data.qvel[:7], data.qfrc_bias[:7],
                  pos_des, quat0, [float(switched)], [override[2] if switched else 0.0]]
        inputs = [np.array(x, dtype=float).copy() for x in inputs]
        a = time.perf_counter_ns()
        tau = ctl.step(data, pos_des, quat0, wrench_override=override)
        times.append((time.perf_counter_ns() - a) * 1e-3)
        rows.append(np.concatenate(inputs + [tau]))
        data.ctrl[:7] = tau
        mujoco.mj_step(model, data)
        zs.append(data.site_xpos[ctl.site_id][2])
        if t_contact is None and true_contact_force(model, data, "probe_ball")[2] > 0.0:
            t_contact = t
        if t_contact is not None and t >= t_contact + m1.HOLD:
            break

    ref = m1.run("force")["log"]["z"]
    assert np.array_equal(ref, np.array(zs)), "recorded loop differs from m1_press.run"
    trace = np.stack(rows)
    assert trace.shape[1] == 135
    with open(BUILD / "trace.bin", "wb") as f:
        np.array([len(trace)], dtype=float).tofile(f)
        trace.tofile(f)
    v = np.sort(times)
    return {"ticks": len(trace), "p50": float(v[len(v) // 2]), "p99": float(v[int(0.99 * len(v))]),
            "p999": float(v[int(0.999 * len(v))]), "max": float(v[-1]), "mean": float(v.mean())}


def cpp(*args: str) -> dict:
    out = subprocess.run([str(BUILD / "m5"), *args], check=True, capture_output=True, text=True).stdout
    return json.loads(out)


def main(rt_seconds: int) -> None:
    BUILD.mkdir(parents=True, exist_ok=True)
    subprocess.run(["cmake", "-S", str(ROOT / "cpp"), "-B", str(BUILD), "-DCMAKE_BUILD_TYPE=Release"], check=True,
                   capture_output=True)
    build = subprocess.run(["cmake", "--build", str(BUILD), "-j8"], capture_output=True, text=True)
    if build.returncode:
        print(build.stdout[-4000:], build.stderr[-4000:])
        sys.exit(1)
    warnings = [ln for ln in build.stdout.splitlines() + build.stderr.splitlines() if "warning" in ln and "m5.cpp" in ln]
    print("compiler warnings in m5.cpp:", warnings or "none")

    py = record()
    print("python compute us", py)
    mjb, trace = str(BUILD / "press.mjb"), str(BUILD / "trace.bin")
    law = cpp("law", trace)
    closed = cpp("closed", mjb, trace)
    print("law", law)
    print("closed", closed)

    rt = {"sleep_idle": cpp("rt", mjb, str(rt_seconds), "sleep")}
    print("sleep idle", rt["sleep_idle"], flush=True)
    rt["spin_idle"] = cpp("rt", mjb, str(rt_seconds), "spin")
    print("spin idle", rt["spin_idle"], flush=True)
    burners = [subprocess.Popen(["sh", "-c", "while :; do :; done"]) for _ in range(16)]
    try:
        rt["sleep_loaded_16_procs"] = cpp("rt", mjb, str(rt_seconds), "sleep")
    finally:
        for b in burners:
            b.kill()
            b.wait()
    print("sleep loaded", rt["sleep_loaded_16_procs"], flush=True)

    res = {
        "python_compute_us": py, "law": law, "closed": closed, "rt": rt,
        "pass": {
            "a_law_lt_1e-9_Nm": law["max_abs_diff_Nm"] < 1e-9,
            "b_closed_lt_1e-6_Nm": closed["max_abs_diff_Nm"] < 1e-6,
            "c_cpp_p999_lt_100us": law["compute_us"]["p999"] < 100 and closed["control_us"]["p999"] < 100,
        },
    }
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "m5_cpp.json").write_text(json.dumps(res, indent=2))
    print(json.dumps(res["pass"]))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--rt-seconds", type=int, default=RT_SECONDS)
    main(ap.parse_args().rt_seconds)
