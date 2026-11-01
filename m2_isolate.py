"""M2 isolation test: does the sliding chatter need the arm?

A free-floating ball probe (3 slide joints, no rotation, no arm) presses and slides
on the same table with the same contact, friction, path and control laws as
m2_wipe.py (Cartesian spring-damper with velocity feedforward, z force PI with
bumpless switch). If it chatters too, the cause is the contact + friction model;
if it doesn't, the cause is the arm's dynamics under the controller.
"""
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).parent / "src"))
from m1_press import CONTACT_THRESH, F_DES, K_POS, V_APPROACH, ForcePI  # noqa: E402
from m2_wipe import SETTLE, TAIL, make_path  # noqa: E402

RESULTS = Path(__file__).parent / "results"
TABLE_TOP = 0.45
R = 0.01
START = np.array([0.5545, 0.0, 0.5045])     # same probe centre as the arm at home


def build(mass: float, mu: float, dmin: float | None) -> mujoco.MjModel:
    spec = mujoco.MjSpec()
    spec.option.timestep = 0.001
    spec.worldbody.add_geom(name="table_top", type=mujoco.mjtGeom.mjGEOM_BOX,
                            size=[0.3, 0.4, TABLE_TOP / 2], pos=[0.55, 0, TABLE_TOP / 2])
    b = spec.worldbody.add_body(name="probe", pos=START.tolist())
    for i, ax in enumerate(("x", "y", "z")):
        axis = [0, 0, 0]
        axis[i] = 1
        b.add_joint(name=f"slide_{ax}", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=axis)
    b.add_geom(name="probe_ball", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[R, 0, 0], mass=mass)
    for g in ("table_top", "probe_ball"):
        spec.geom(g).friction = [mu, 0.005, 0.0001]
        if dmin is not None:
            spec.geom(g).solimp = [dmin, 0.95, 0.001, 0.5, 2.0]
    return spec.compile()


def contact_fz(model, data) -> float:
    """World-z contact force on the ball (ground truth; also used as the ideal F/T)."""
    f6 = np.zeros(6)
    total = 0.0
    gid = model.geom("probe_ball").id
    for i in range(data.ncon):
        c = data.contact[i]
        mujoco.mj_contactForce(model, data, i, f6)
        f = c.frame.reshape(3, 3).T @ f6[:3]
        total += (f if c.geom2 == gid else -f)[2]
    return total


def run(mode: str, kind: str, mass: float, mu: float = 0.5, dmin: float | None = None,
        v: float = 0.05) -> dict:
    model = build(mass, mu, dmin)
    data = mujoco.MjData(model)
    dt = model.opt.timestep
    d = 2 * np.sqrt(K_POS)                     # same Cartesian damping as the arm controller
    g = mass * 9.81
    path = make_path(kind, START[:2].copy(), v, dt)
    z_stop = TABLE_TOP + R - F_DES / K_POS if mode == "impedance" else -np.inf
    pi = ForcePI()

    t_contact = t_move = None
    switched = False
    f_log, moving = [], []
    for step in range(int(30 / dt)):
        t = step * dt
        p = START + data.qpos[:3]
        vel = data.qvel[:3]
        z_des = max(START[2] - V_APPROACH * t, z_stop)
        k = -1
        xy_des, v_des = START[:2], np.zeros(2)
        if t_contact is not None and t >= t_contact + SETTLE:
            t_move = t if t_move is None else t_move
            k = int(round((t - t_move) / dt))
            if k < len(path.pos):
                xy_des, v_des = path.pos[k], path.vel[k]
            else:
                xy_des = path.pos[-1]
        f_meas = contact_fz(model, data)
        F = np.zeros(3)
        F[:2] = K_POS * (xy_des - p[:2]) - d * (vel[:2] - v_des)
        F[2] = K_POS * (z_des - p[2]) - d * vel[2]
        if mode == "force":
            if not switched and f_meas > CONTACT_THRESH:
                switched = True
                pi.reset(max(0.0, K_POS * (p[2] - z_des)), t)
            if switched:
                F[2] = pi.wrench_z(f_meas, t, dt) - d * vel[2]
        F[2] += g                              # gravity compensation (exact, like qfrc_bias)
        data.qfrc_applied[:3] = F
        mujoco.mj_step(model, data)

        fz = contact_fz(model, data)
        if t_contact is None and fz > 0:
            t_contact = t
        f_log.append(fz)
        moving.append(0 <= k < len(path.pos))
        if t_move is not None and k >= len(path.pos) + int(TAIL / dt):
            break
    f = np.array(f_log)[np.array(moving)]
    return {"mode": mode, "path": kind, "mass": mass, "mu": mu, "dmin": dmin,
            "steps_without_contact": int(np.sum(f <= 0)),
            "force_rmse_N": float(np.sqrt(np.mean((f - F_DES) ** 2))),
            "force_max_N": float(f.max()), "force_min_N": float(f.min())}


def main():
    rows = [run(mode, kind, mass, mu)
            for mass in (0.86, 4.6) for mu in (0.0, 0.1, 0.5)
            for kind in ("line", "circle") for mode in ("impedance", "force")]
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "m2_isolate.json").write_text(json.dumps(rows, indent=2))
    for r in rows:
        print(f"mass {r['mass']:<4} mu {r['mu']:<3} {r['path']:6s} {r['mode']:9s} | noContact "
              f"{r['steps_without_contact']:5d} rmse {r['force_rmse_N']:.3f} "
              f"min {r['force_min_N']:.2f} max {r['force_max_N']:.2f}")


if __name__ == "__main__":
    main()
