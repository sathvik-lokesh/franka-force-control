"""M3 isolation test: does the no-contact rate during the spiral need the arm?

On the arm, no-contact during search FALLS as friction rises (M3 test: mu 0.2 / 0.5 / 1.0 ->
6 % / 1 % / 0.25 % at 1 cm/s), the opposite of the M2 ball on a straight or circular path.
Here a body on 3 slide joints (no arm, no rotation) runs the same spiral with the same contact,
force PI and xy gains on a plain plate, once with the square peg and once with the M2 ball.
If the trend survives, it comes from contact + friction, not the arm.
"""
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).parent / "src"))
from franka_model import PEG_SIDE  # noqa: E402
from m1_press import CONTACT_THRESH, V_APPROACH, ForcePI  # noqa: E402
from m3_peg import DMIN, F_SEARCH, K_SEARCH_XY, K_SOFT, R_MAX, SETTLE, spiral  # noqa: E402

RESULTS = Path(__file__).parent / "results"
TOP = 0.43
GAP = 0.005                # start height above the plate [m]
BALL_R = 0.01


def build(shape: str, mass: float, mu: float, dmin: float | None) -> mujoco.MjModel:
    spec = mujoco.MjSpec()
    spec.option.timestep = 0.001
    spec.worldbody.add_geom(name="plate", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.1, 0.1, TOP / 2],
                            pos=[0, 0, TOP / 2])
    half = PEG_SIDE / 2 if shape == "peg" else BALL_R
    b = spec.worldbody.add_body(name="tool", pos=[0, 0, TOP + half + GAP])
    for i, ax in enumerate("xyz"):
        axis = [0, 0, 0]
        axis[i] = 1
        b.add_joint(name=f"slide_{ax}", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=axis)
    if shape == "peg":
        b.add_geom(name="tool", type=mujoco.mjtGeom.mjGEOM_BOX, size=[half] * 3, mass=mass)
    else:
        b.add_geom(name="tool", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[half, 0, 0], mass=mass)
    for g in ("plate", "tool"):
        spec.geom(g).friction = [mu, 0.005, 0.0001]
        if dmin is not None:
            spec.geom(g).solimp = [dmin, 0.95, 0.001, 0.5, 2.0]
    return spec.compile()


def contact_fz(model, data) -> float:
    """Net world-z contact force on the tool (ground truth, also used as the ideal F/T)."""
    f6 = np.zeros(6)
    gid = model.geom("tool").id
    total = 0.0
    for i in range(data.ncon):
        c = data.contact[i]
        mujoco.mj_contactForce(model, data, i, f6)
        f = c.frame.reshape(3, 3).T @ f6[:3]
        total += (f if c.geom2 == gid else -f)[2]
    return total


def run(shape: str, mu: float, v: float, pitch: float, mass: float = 4.6, dmin: float | None = DMIN) -> dict:
    model = build(shape, mass, mu, dmin)
    data = mujoco.MjData(model)
    dt = model.opt.timestep
    path_pos, path_vel = spiral(pitch, v, R_MAX, dt)
    k_xy, d_xy, d_z = K_SEARCH_XY, 2 * np.sqrt(K_SEARCH_XY), 2 * np.sqrt(K_SOFT)
    pi = ForcePI(f_des=F_SEARCH)
    switched, t_contact, k_path = False, None, 0
    fz_log = []
    for step in range(int((GAP / V_APPROACH + SETTLE + len(path_pos) * dt + 2.0) / dt)):
        t = step * dt
        z = data.qpos[2]
        f_meas = contact_fz(model, data)
        xy_des, v_xy = np.zeros(2), np.zeros(2)
        searching = t_contact is not None and t >= t_contact + SETTLE and k_path < len(path_pos)
        if searching:
            xy_des, v_xy = path_pos[k_path], path_vel[k_path]
            k_path += 1
        F = np.zeros(3)
        F[:2] = k_xy * (xy_des - data.qpos[:2]) - d_xy * (data.qvel[:2] - v_xy)
        if not switched and f_meas > CONTACT_THRESH:
            switched = True
            t_contact = t
            pi.reset(max(0.0, K_SOFT * (z + V_APPROACH * t)), t)
        F[2] = (pi.wrench_z(f_meas, t, dt) if switched else K_SOFT * (-V_APPROACH * t - z)) - d_z * data.qvel[2]
        data.qfrc_applied[:3] = F + np.array([0, 0, mass * 9.81])
        mujoco.mj_step(model, data)
        if searching:
            fz_log.append(contact_fz(model, data))
        if k_path >= len(path_pos):
            break
    f = np.array(fz_log)
    return {"shape": shape, "mu": mu, "v": v, "pitch": pitch, "mass": mass, "dmin": dmin,
            "search_rmse_N": float(np.sqrt(np.mean((f - F_SEARCH) ** 2))),
            "nocontact_pct": float(100 * np.mean(f <= 0)), "steps": len(f)}


def main():
    rows = [run(shape, mu, v, p) for shape in ("peg", "ball") for v, p in ((0.01, 0.00075), (0.02, 0.001))
            for mu in (0.2, 0.5, 1.0)]
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "m3_isolate.json").write_text(json.dumps(rows, indent=2))
    for r in rows:
        print(f"{r['shape']:4s} v={r['v']:.2f} p={r['pitch'] * 1e3:.2f} mm mu={r['mu']:<3} | "
              f"Fz rmse {r['search_rmse_N']:.3f} no-contact {r['nocontact_pct']:.2f} % ({r['steps']} steps)")


if __name__ == "__main__":
    main()
