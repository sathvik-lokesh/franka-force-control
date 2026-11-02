"""M3: square peg into a blind square hole whose position the controller doesn't know.

Four controllers, defined with the pass bars in EXPERIMENTS.md (M3) before any peg run:
  force_spiral    : force-controlled spiral search, compliant insert
  stiff_blind     : stiff position control straight down at the believed hole
  compliant_blind : soft impedance straight down at the believed hole, no search
  stiff_spiral    : same spiral with stiff position control, no force feedback

Usage:
  m3_peg.py --check      geometry + contact sanity, no bars
  m3_peg.py --dev        spiral speed/pitch grid on dev seeds (>= 1000)
  m3_peg.py --test       frozen configs A and B on test seeds 0-49, writes results/m3_peg.json
"""
import argparse
import json
import os
import sys
from multiprocessing import Pool
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent / "src"))
from force_sensing import FreeSpaceZero  # noqa: E402
from franka_model import HOLE_DEPTH, HOME_Q, build_model, reset_home  # noqa: E402
from impedance import CartesianImpedance, ImpedanceGains  # noqa: E402
from mismatch import Mismatch  # noqa: E402
from m1_press import CONTACT_THRESH, TARE_TIME, V_APPROACH, ForcePI  # noqa: E402

RESULTS = Path(__file__).parent / "results"
PLATE_TOP = 0.43       # believed plate top [m]
F_SEARCH = 5.0         # N
K_SOFT = 500.0         # N/m
K_STIFF = 5000.0       # N/m
K_SEARCH_XY = 2000.0   # N/m
K_INSERT_XY = 200.0    # N/m
SETTLE = 0.5           # s between first contact and the spiral
DROP = 0.0005          # m below the measured contact height = peg is falling into the hole
IN_HOLE = 0.005        # m past the believed surface with no contact = found the hole on the way down
RIM_FALL = 0.0015      # m below first contact during settle = clipped the rim and is falling in
V_INSERT = 0.01        # m/s
F_CAP = 20.0           # N, insert push limit
R_MAX = 0.007          # m, spiral outer radius
SUCCESS_DEPTH = 0.018  # m
T_INSERT_MAX = 5.0     # s allowed from the drop to full depth
T_LIMIT = 90.0         # s
MU = 0.5
DMIN = 0.2
LEVELS = (0.002, 0.0035, 0.005)
CONTROLLERS = ("force_spiral", "stiff_blind", "compliant_blind", "stiff_spiral")


def hole_offset(r: float, seed: int) -> np.ndarray:
    """True hole minus believed hole: magnitude r, direction from the seed."""
    a = np.random.default_rng(seed).uniform(0, 2 * np.pi)
    return r * np.array([np.cos(a), np.sin(a)])


def spiral(pitch: float, v: float, r_max: float, dt: float, a_max: float = 0.5):
    """Archimedean spiral r = pitch*theta/(2 pi) at constant path speed. Returns (N,2) offset and velocity."""
    b = pitch / (2 * np.pi)
    theta, pts, vels, t = 0.0, [], [], 0.0
    while b * theta < r_max:
        speed = min(v, a_max * t)
        thetad = speed / np.hypot(b * theta, b)
        r = b * theta
        c, s = np.cos(theta), np.sin(theta)
        pts.append((r * c, r * s))
        vels.append((b * thetad * c - r * thetad * s, b * thetad * s + r * thetad * c))
        theta += thetad * dt
        t += dt
    return np.array(pts), np.array(vels)


def peg_contact_force(model, data, peg_id: int) -> tuple[float, float]:
    """(sum of per-contact force magnitudes on the peg, net normal-direction Fz pushing the peg up)."""
    total, fz = 0.0, 0.0
    f6 = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        if peg_id not in (c.geom1, c.geom2):
            continue
        mujoco.mj_contactForce(model, data, i, f6)
        total += float(np.linalg.norm(f6[:3]))
        f_world = c.frame.reshape(3, 3).T @ f6[:3]
        fz += f_world[2] if c.geom2 == peg_id else -f_world[2]
    return total, fz


def set_xy_gain(ctl: CartesianImpedance, k: float) -> None:
    for i in (0, 1):
        ctl.K[i, i] = k
        ctl.D[i, i] = 2 * np.sqrt(k)


def set_z_gain(ctl: CartesianImpedance, k: float) -> None:
    ctl.K[2, 2] = k
    ctl.D[2, 2] = 2 * np.sqrt(k)


def run(controller: str, r: float, seed: int, v: float = 0.01, pitch: float = 0.00075,
        height_err: float = 0.0, mu: float = MU, dmin: float | None = DMIN,
        keep_log: bool = False, mm: Mismatch | None = None, stiff_uses_ft: bool = False,
        robust: bool = False, bumpless_insert: bool = False) -> dict:
    """One trial. height_err = true plate top - believed plate top.

    stiff_uses_ft=True is the M3 test's stiff_spiral, which timed its search start off the F/T. False is
    the really force-blind version: search starts on a timer, nothing reads the sensor.
    robust = M4 v2 for force_spiral: FreeSpaceZero contact detection and no clamp on the insert ramp.
    bumpless_insert = M4 v3: the insert spring starts from the force the PI was commanding.
    """
    mm = mm or Mismatch()
    probe = build_model(task="peg", friction=mu, contact_dmin=dmin)
    data = mujoco.MjData(probe)
    reset_home(probe, data)
    believed_xy = data.site_xpos[probe.site("peg_tip").id][:2].copy()
    offset = hole_offset(r, seed)
    true_xy = believed_xy + offset
    true_top = PLATE_TOP + height_err

    model = build_model(task="peg", friction=mu, contact_dmin=dmin,
                        hole_xy=tuple(true_xy), plate_top=true_top, hole_yaw_deg=mm.hole_yaw_deg)
    believed = mm.apply(model)
    delay = mm.make_delay()
    data = mujoco.MjData(model)
    reset_home(model, data)
    data.ctrl[7] = 255
    stiff = controller.startswith("stiff")
    k0 = K_STIFF if stiff else K_SOFT
    ctl = CartesianImpedance(model, ImpedanceGains(k_pos=k0, q_rest=HOME_Q.copy()), site="peg_tip",
                             believed=believed)
    ft = mm.make_ft(model)
    robust_ctl = robust and controller == "force_spiral"
    zero = FreeSpaceZero(model.opt.timestep) if robust_ctl else None
    pi = ForcePI(f_des=F_SEARCH)
    dt = model.opt.timestep
    peg_id = model.geom("peg").id
    p0, quat0 = ctl.tcp_pose(data)
    blind = controller.endswith("blind")
    spiral_ctl = controller.endswith("spiral")
    z_floor_believed = PLATE_TOP - HOLE_DEPTH
    z_stop = (z_floor_believed if blind
              else PLATE_TOP - F_SEARCH / K_STIFF if controller == "stiff_spiral" else -np.inf)
    path_pos, path_vel = spiral(pitch, v, R_MAX, dt) if spiral_ctl else (None, None)

    phase = "approach"
    switched = False
    t_first_contact = t_settled = t_search = t_drop = t_success = None
    z_contact = None
    z_window = []
    k_path = 0
    xy_hold = believed_xy.copy()
    z_ins0 = 0.0
    last_cmd = None
    peak_total, peak_time = 0.0, 0.0
    search_f, search_nocontact = [], 0
    tare_steps = int(TARE_TIME / dt)
    log = {k: [] for k in ("t", "x", "y", "z", "f_total", "fz", "phase", "qpos")} if keep_log else None

    for step in range(int(T_LIMIT / dt)):
        t = step * dt
        if step == tare_steps:
            ft.tare(data)
        f_est = ft.contact_force(data)[2] if step >= tare_steps else 0.0
        tip = data.site_xpos[ctl.site_id]
        z = tip[2]
        xy_des, v_xy = believed_xy, np.zeros(2)
        z_des = max(p0[2] - V_APPROACH * max(0.0, t - TARE_TIME), z_stop)
        v_z = 0.0
        override = None

        if spiral_ctl:
            reads_ft = controller == "force_spiral" or stiff_uses_ft
            touched = reads_ft and f_est > CONTACT_THRESH and step > tare_steps
            if zero is not None and step >= tare_steps:
                f_est, is_touch = zero.update(f_est)
                touched = is_touch and step > tare_steps
            if phase == "approach":
                if touched:
                    t_first_contact = t
                    z_first_contact = z
                    phase = "settle"
                    if controller == "force_spiral":
                        pi.reset(max(0.0, ctl.K[2, 2] * (z - z_des)), t)
                        switched = True
                elif z < PLATE_TOP - IN_HOLE:
                    phase = "insert"             # hole was under the peg: no contact, already past the surface
                    t_drop = t
                elif controller == "stiff_spiral" and z_des <= z_stop and t > TARE_TIME + (p0[2] - z_stop) / V_APPROACH + SETTLE:
                    phase = "search"             # reached the height it believes in, no contact felt: search anyway
                    t_search = t
                    z_contact = z
                    set_xy_gain(ctl, K_STIFF)
            elif phase == "settle":
                if z < z_first_contact - RIM_FALL:
                    phase = "insert"             # clipped the rim on the way in, then kept falling
                    t_drop = t
                elif t >= t_first_contact + SETTLE - 0.1:
                    z_window.append(z)
                if phase == "settle" and t >= t_first_contact + SETTLE:
                    z_contact = float(np.mean(z_window))
                    phase = "search"
                    t_search = t
                    set_xy_gain(ctl, K_SEARCH_XY if controller == "force_spiral" else K_STIFF)
            if phase == "search":
                if k_path < len(path_pos):
                    xy_des = believed_xy + path_pos[k_path]
                    v_xy = path_vel[k_path]
                    k_path += 1
                else:
                    xy_des = believed_xy + path_pos[-1]
                if z < z_contact - DROP:
                    phase = "insert"
                    t_drop = t
            if phase == "insert":
                if z_ins0 == 0.0:
                    xy_hold = tip[:2].copy()
                    z_ins0 = z
                    if bumpless_insert and controller == "force_spiral" and last_cmd is not None:
                        z_ins0 = z + last_cmd / K_SOFT     # keeps pushing as hard as PI was (last_cmd < 0)
                    switched = False
                    set_xy_gain(ctl, K_INSERT_XY if controller == "force_spiral" else K_STIFF)
                    set_z_gain(ctl, K_SOFT if controller == "force_spiral" else K_STIFF)
                xy_des = xy_hold
                z_des = z_ins0 - V_INSERT * (t - t_drop)
                if not robust_ctl:
                    z_des = max(z_des, PLATE_TOP - HOLE_DEPTH - 0.002)   # v1 clamp: leaves 2 N at success depth
                v_z = -V_INSERT
                if controller == "force_spiral":
                    override = {2: max(ctl.K[2, 2] * (z_des - z), -F_CAP)}
            elif switched:
                override = {2: pi.wrench_z(f_est, t, dt)}
                last_cmd = override[2]

        pos_des = np.array([xy_des[0], xy_des[1], z_des])
        twist = np.array([v_xy[0], v_xy[1], v_z, 0, 0, 0])
        data.ctrl[:7] = delay(ctl.step(data, pos_des, quat0, wrench_override=override, vel_des=twist))
        mujoco.mj_step(model, data)

        total, fz = peg_contact_force(model, data, peg_id)
        if total > peak_total:
            peak_total, peak_time = total, t
        if phase == "search":
            search_f.append(fz)
            search_nocontact += total <= 0.0
        depth = true_top - data.site_xpos[ctl.site_id][2]
        if keep_log:
            for k_, val in zip(log, (t, *data.site_xpos[ctl.site_id], total, fz, phase)):
                log[k_].append(val)
            log["qpos"].append(data.qpos.copy())
        if t_success is None and depth >= SUCCESS_DEPTH:
            t_success = t
            break
        if t_drop is not None and t > t_drop + T_INSERT_MAX:
            break
        if blind and z_des <= z_stop and t > TARE_TIME + (p0[2] - z_stop) / V_APPROACH + 2.0:
            break
        if spiral_ctl and t_search is not None and t_drop is None and k_path >= len(path_pos) and t > t_search + len(path_pos) * dt + 1.0:
            break

    sf = np.array(search_f) if search_f else None
    res = {
        "controller": controller, "r": r, "seed": seed, "height_err": height_err, "mu": mu, "dmin": dmin,
        "v": v, "pitch": pitch,
        "success": t_success is not None,
        "t_total": t_success,
        "t_search": (t_drop - t_search) if t_drop is not None and t_search is not None else None,
        "t_drop": t_drop,
        "peak_force_N": peak_total,
        "peak_time_s": peak_time,
        "final_depth_mm": float((true_top - data.site_xpos[ctl.site_id][2]) * 1e3),
        "final_phase": phase,
        "search_rmse_N": float(np.sqrt(np.mean((sf - F_SEARCH) ** 2))) if sf is not None else None,
        "search_steps": 0 if sf is None else len(sf),
        "search_nocontact_steps": int(search_nocontact),
        "true_offset_mm": (offset * 1e3).tolist(),
        "xy_error_at_end_mm": ((data.site_xpos[ctl.site_id][:2] - true_xy) * 1e3).tolist(),
    }
    if keep_log:
        res["log"] = log
    return res


def _run_kw(kw: dict) -> dict:
    return run(**kw)


def check() -> None:
    """Geometry sanity before anything else: peg aligned with the hole, no contact at rest, FK as assumed."""
    m = build_model(task="peg", friction=MU, contact_dmin=DMIN)
    d = mujoco.MjData(m)
    reset_home(m, d)
    tip = d.site_xpos[m.site("peg_tip").id]
    R = d.geom_xmat[m.geom("peg").id].reshape(3, 3)
    print("tip", tip, "gap to plate top [mm]", (tip[2] - PLATE_TOP) * 1e3)
    print("peg axes in world:\n", np.round(R, 4))
    print("contacts at home:", d.ncon)
    for kw in ({"controller": "force_spiral", "r": 0.0, "seed": 0},
               {"controller": "force_spiral", "r": 0.002, "seed": 1},
               {"controller": "compliant_blind", "r": 0.0, "seed": 0},
               {"controller": "stiff_blind", "r": 0.002, "seed": 1},
               {"controller": "stiff_spiral", "r": 0.002, "seed": 1}):
        res = run(**kw, keep_log=False)
        print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in res.items()
               if k in ("controller", "r", "success", "t_total", "t_search", "peak_force_N",
                        "final_depth_mm", "final_phase", "search_rmse_N", "search_nocontact_steps")})


def dev() -> None:
    seeds = range(1000, 1010)
    jobs = [dict(controller="force_spiral", r=0.005, seed=s, v=v, pitch=p)
            for v in (0.01, 0.02) for p in (0.0005, 0.00075, 0.001) for s in seeds]
    with Pool(12) as pool:
        out = pool.map(_run_kw, jobs)
    rows = []
    for v in (0.01, 0.02):
        for p in (0.0005, 0.00075, 0.001):
            rs = [x for x in out if x["v"] == v and x["pitch"] == p]
            ok = [x for x in rs if x["success"]]
            row = dict(v=v, pitch=p, success=len(ok), n=len(rs),
                       median_total=float(np.median([x["t_total"] for x in ok])) if ok else None,
                       max_total=max((x["t_total"] for x in ok), default=None),
                       peak=max(x["peak_force_N"] for x in rs),
                       rmse=float(np.mean([x["search_rmse_N"] for x in rs if x["search_rmse_N"] is not None])))
            rows.append(row)
            print(row)
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "m3_dev.json").write_text(json.dumps({"rows": rows, "trials": out}, indent=2))


CONFIGS = {"A": dict(v=0.02, pitch=0.001),      # what the registered selection rule picks
           "B": dict(v=0.01, pitch=0.00075)}    # chosen after seeing dev, see EXPERIMENTS.md
TEST_SEEDS = range(50)
SANITY_SEEDS = range(5)                         # r = 0: direction is meaningless
HEIGHTS = (0.0, 0.003, -0.003)


def test_jobs() -> list[dict]:
    jobs = []
    for name, cfg in CONFIGS.items():
        for ctrl in ("force_spiral", "stiff_spiral"):
            for h in HEIGHTS:
                jobs += [dict(controller=ctrl, r=r, seed=s, height_err=h, stiff_uses_ft=True, **cfg)
                         for r in LEVELS for s in TEST_SEEDS]
            jobs += [dict(controller=ctrl, r=0.0, seed=s, stiff_uses_ft=True, **cfg) for s in SANITY_SEEDS]
        for sens in (dict(dmin=None), dict(mu=0.2), dict(mu=1.0)):
            jobs += [dict(controller="force_spiral", r=r, seed=s, **cfg, **sens)
                     for r in LEVELS for s in TEST_SEEDS]
    for ctrl in ("stiff_blind", "compliant_blind"):
        jobs += [dict(controller=ctrl, r=r, seed=s) for r in LEVELS for s in TEST_SEEDS]
        jobs += [dict(controller=ctrl, r=0.0, seed=s) for s in SANITY_SEEDS]
    return jobs


def summarize(rs: list[dict]) -> dict:
    """Aggregate one group of trials. Search force stats are pooled over steps."""
    ok = [x for x in rs if x["success"]]
    steps = sum(x["search_steps"] for x in rs)
    t_search = [x["t_search"] for x in ok if x["t_search"] is not None]
    return {
        "success": len(ok), "n": len(rs),
        "t_total_median_s": float(np.median([x["t_total"] for x in ok])) if ok else None,
        "t_total_max_s": max((x["t_total"] for x in ok), default=None),
        "t_search_median_s": float(np.median(t_search)) if t_search else None,
        "peak_median_N": float(np.median([x["peak_force_N"] for x in rs])),
        "peak_max_N": max(x["peak_force_N"] for x in rs),
        "peak_max_success_N": max((x["peak_force_N"] for x in ok), default=None),
        "search_rmse_N": (float(np.sqrt(sum(x["search_rmse_N"] ** 2 * x["search_steps"] for x in rs
                                            if x["search_steps"]) / steps)) if steps else None),
        "search_nocontact_pct": 100 * sum(x["search_nocontact_steps"] for x in rs) / steps if steps else None,
    }


def bars(trials: list[dict], cfg: dict) -> dict:
    """The four registered bars for one spiral config."""
    def select(h):
        return [x for x in trials if x["controller"] == "force_spiral" and x["height_err"] == h
                and x["v"] == cfg["v"] and x["pitch"] == cfg["pitch"] and x["mu"] == MU
                and x["dmin"] == DMIN and x["r"] in LEVELS]

    nominal = select(0.0)
    s = summarize(nominal)
    per_r = lambda rs: {str(r): sum(x["success"] for x in rs if x["r"] == r) for r in LEVELS}
    a = per_r(nominal)
    c = {str(h): per_r(select(h)) for h in HEIGHTS[1:]}
    return {
        "a_success_per_r": a,
        "a_pass": all(v >= 48 for v in a.values()),
        "b_peak_max_N": s["peak_max_success_N"],
        "b_pass": s["peak_max_success_N"] is not None and s["peak_max_success_N"] < 30.0,
        "c_success_per_height_and_r": c,
        "c_pass": all(v >= 48 for d in c.values() for v in d.values()),
        "d_search_rmse_N": s["search_rmse_N"],
        "d_search_nocontact_pct": s["search_nocontact_pct"],
        "d_pass": s["search_rmse_N"] < 1.0 and s["search_nocontact_pct"] < 1.0,
    }


def test() -> None:
    jobs = test_jobs()
    print(len(jobs), "trials", flush=True)
    with Pool(12) as pool:
        out = pool.map(_run_kw, jobs, chunksize=4)
    keys = ("controller", "v", "pitch", "height_err", "mu", "dmin", "r")
    groups: dict[tuple, list[dict]] = {}
    for x in out:
        k = tuple(x[k_] for k_ in keys)
        groups.setdefault(k, []).append(x)
    table = [{**dict(zip(keys, k)), **summarize(rs)} for k, rs in groups.items()]
    res = {"configs": CONFIGS, "bars": {n: bars(out, c) for n, c in CONFIGS.items()},
           "table": table, "trials": out}
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "m3_peg.json").write_text(json.dumps(res, indent=1))
    for row in table:
        print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in row.items()})
    print(json.dumps(res["bars"], indent=1))


def plot() -> None:
    """Force feedback vs none on the same spiral: success and peak force by unknown fixture height."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    trials = json.loads((RESULTS / "m3_peg.json").read_text())["trials"]
    ink, muted, grid = "#0b0b0b", "#52514e", "#e4e3df"
    series = (("force_spiral", "Force control + spiral", "#2a78d6"),
              ("stiff_spiral", "Stiff position + spiral", "#eb6834"))
    titles = {"A": "Config A: 2 cm/s, 1 mm pitch (selected by the rule)",
              "B": "Config B: 1 cm/s, 0.75 mm pitch (chosen after dev)"}
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True, facecolor="#fcfcfb")
    for row, (name, cfg) in enumerate(CONFIGS.items()):
        for col, (metric, ylabel) in enumerate((("success", "Inserted, out of 150 trials"),
                                                ("peak", "Max peak contact force [N]"))):
            ax = axes[row, col]
            ax.set_facecolor("#fcfcfb")
            for i, (ctrl, label, colour) in enumerate(series):
                vals = []
                for h in (-0.003, 0.0, 0.003):
                    rs = [x for x in trials if x["controller"] == ctrl and x["height_err"] == h
                          and x["v"] == cfg["v"] and x["pitch"] == cfg["pitch"] and x["mu"] == MU
                          and x["dmin"] == DMIN and x["r"] in LEVELS]
                    vals.append(sum(x["success"] for x in rs) if metric == "success"
                                else max(x["peak_force_N"] for x in rs))
                xs = np.arange(3) + (i - 0.5) * 0.36
                bars_ = ax.bar(xs, vals, width=0.34, color=colour, label=label)
                ax.bar_label(bars_, labels=[f"{v:.0f}" for v in vals], padding=3, fontsize=9, color=ink)
            ax.set_xticks(range(3), ["-3 mm", "as believed", "+3 mm"])
            ax.set_ylabel(ylabel, color=muted, fontsize=9)
            ax.set_ylim(0, 175 if metric == "success" else 65)
            ax.grid(axis="y", color=grid, lw=0.8)
            ax.set_axisbelow(True)
            ax.tick_params(colors=muted, length=0, labelsize=9)
            for side in ("top", "right", "left"):
                ax.spines[side].set_visible(False)
            ax.spines["bottom"].set_color(muted)
            if col == 0:
                ax.set_title(titles[name], loc="left", fontsize=11, color=ink)
            if row == 1:
                ax.set_xlabel("True fixture height vs believed", color=muted, fontsize=9)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(RESULTS / "m3_peg.png", dpi=130)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--dev", action="store_true")
    ap.add_argument("--test", action="store_true")
    args = ap.parse_args()
    if args.check:
        check()
    elif args.dev:
        dev()
    elif args.test:
        test()
    elif args.plot:
        plot()
