"""M2: wiping. Hold 10 N on the table while tracing a line or a circle.

Approach + contact exactly as in M1 (force control switches on contact with
bumpless transfer). After a settle, pos_des moves along the path in x/y with a
trapezoidal speed profile and the matching vel_des (velocity feedforward).

Conditions: flat table and a table tilted 3 deg about x that the controller
doesn't know about. Pass bars are fixed in PLAN.md (M2) before this was run.
"""
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
import mujoco
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent / "src"))
from force_sensing import FlangeFT, true_contact_force  # noqa: E402
from franka_model import HOME_Q, PROBE_RADIUS, TABLE_TOP, build_model, reset_home  # noqa: E402
from impedance import CartesianImpedance, ImpedanceGains  # noqa: E402
from m1_press import CONTACT_THRESH, F_DES, K_POS, TARE_TIME, V_APPROACH, ForcePI  # noqa: E402

RESULTS = Path(__file__).parent / "results"
A_MAX = 0.5           # m/s^2, path acceleration limit
SETTLE = 1.5          # s from first contact to motion start (impedance spring ramp-in takes ~0.9 s)
TAIL = 0.3            # s held still after the path ends
LINE_LEN = 0.10       # m
CIRCLE_R = 0.05       # m
MU = 0.5
PAD_DMIN = 0.2        # compliant pad; None = MuJoCo default rigid contact (chatters, see PLAN M2)


def trapezoid(length: float, v: float, a: float, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Arc length s(t) and speed sdot(t) for a rest-to-rest trapezoidal profile."""
    t_acc = v / a
    if a * t_acc ** 2 > length:           # never reaches v: triangle profile
        t_acc = np.sqrt(length / a)
        v = a * t_acc
    t_cruise = (length - a * t_acc ** 2) / v
    T = 2 * t_acc + t_cruise
    t = np.arange(0.0, T + dt, dt)
    s = np.where(t < t_acc, 0.5 * a * t ** 2,
                 np.where(t < t_acc + t_cruise, 0.5 * a * t_acc ** 2 + v * (t - t_acc),
                          length - 0.5 * a * np.clip(T - t, 0, None) ** 2))
    sd = np.where(t < t_acc, a * t, np.where(t < t_acc + t_cruise, v, a * np.clip(T - t, 0, None)))
    return np.clip(s, 0, length), sd


@dataclass
class Path2D:
    """Desired x/y over time, tangent, and the geometry for cross-track error."""
    kind: str
    pos: np.ndarray            # (N, 2)
    vel: np.ndarray            # (N, 2)
    cruise: np.ndarray         # (N,) bool, constant-speed part
    center: np.ndarray = field(default_factory=lambda: np.zeros(2))

    def cross_track(self, xy: np.ndarray) -> np.ndarray:
        """Signed geometric distance to the path (circle: + = outside)."""
        if self.kind == "line":
            return xy[:, 0] - self.center[0]
        return np.linalg.norm(xy - self.center, axis=1) - CIRCLE_R


def make_path(kind: str, start: np.ndarray, v: float, dt: float) -> Path2D:
    if kind == "line":
        d = np.array([0.0, 1.0])
        parts = []
        for a, b in ((start, start + LINE_LEN * d), (start + LINE_LEN * d, start)):
            s, sd = trapezoid(LINE_LEN, v, A_MAX, dt)
            u = (b - a) / LINE_LEN
            parts.append((a + s[:, None] * u, sd[:, None] * u, np.isclose(sd, sd.max())))
        pos, vel, cruise = (np.concatenate(x) for x in zip(*parts))
        return Path2D("line", pos, vel, cruise, center=start.copy())
    center = start - np.array([CIRCLE_R, 0.0])
    s, sd = trapezoid(2 * np.pi * CIRCLE_R, v, A_MAX, dt)
    phi = s / CIRCLE_R
    pos = center + CIRCLE_R * np.stack([np.cos(phi), np.sin(phi)], 1)
    vel = sd[:, None] * np.stack([-np.sin(phi), np.cos(phi)], 1)
    return Path2D("circle", pos, vel, np.isclose(sd, sd.max()), center=center)


def run(mode: str, kind: str, v: float = 0.05, tilt: float = 0.0, mu: float = MU,
        k_xy: float = K_POS, dmin: float | None = PAD_DMIN, fric_ff: bool = False) -> dict:
    model = build_model(task="press", table_tilt_deg=tilt, friction=mu, contact_dmin=dmin)
    data = mujoco.MjData(model)
    reset_home(model, data)
    data.ctrl[7] = 255
    ctl = CartesianImpedance(model, ImpedanceGains(k_pos=K_POS, q_rest=HOME_Q.copy()), site="probe")
    for i in (0, 1):
        ctl.K[i, i] = k_xy
        ctl.D[i, i] = 2 * np.sqrt(k_xy)
    ft = FlangeFT(model)
    pi = ForcePI()
    dt = model.opt.timestep

    p0, quat0 = ctl.tcp_pose(data)
    path = make_path(kind, p0[:2].copy(), v, dt)
    z_stop = TABLE_TOP + PROBE_RADIUS - F_DES / K_POS if mode == "impedance" else -np.inf

    t_contact, t_move, switched = None, None, False
    tare_steps = int(TARE_TIME / dt)
    log = {k: [] for k in ("t", "f_true", "ft_tan", "xy", "xy_des", "moving", "k", "pen")}
    for step in range(int(30.0 / dt)):
        t = step * dt
        if step == tare_steps:
            ft.tare(data)
        z_des = max(p0[2] - V_APPROACH * max(0.0, t - TARE_TIME), z_stop)

        k = -1
        xy_des, v_des = p0[:2], np.zeros(2)
        if t_contact is not None and t >= t_contact + SETTLE:
            if t_move is None:
                t_move = t
            k = int(round((t - t_move) / dt))
            if k < len(path.pos):
                xy_des, v_des = path.pos[k], path.vel[k]
            else:
                xy_des = path.pos[-1]
        pos_des = np.array([xy_des[0], xy_des[1], z_des])
        twist = np.concatenate([v_des, np.zeros(4)])

        f_vec = ft.contact_force(data) if step >= tare_steps else np.zeros(3)
        f_est = f_vec[2]
        override = None
        if mode == "force":
            if not switched and step > tare_steps and f_est > CONTACT_THRESH:
                switched = True
                z_now = data.site_xpos[ctl.site_id][2]
                pi.reset(max(0.0, K_POS * (z_now - z_des)), t)
            if switched:
                override = {2: pi.wrench_z(f_est, t, dt)}

        ff = None
        if fric_ff and switched:
            # Cancel the measured tangential contact force (friction) at the TCP.
            ff = np.array([-f_vec[0], -f_vec[1], 0, 0, 0, 0])
        data.ctrl[:7] = ctl.step(data, pos_des, quat0, wrench_override=override, vel_des=twist,
                                 wrench_ff=ff)
        mujoco.mj_step(model, data)

        f = true_contact_force(model, data, "probe_ball")
        if t_contact is None and f[2] > 0.0:
            t_contact = t
        log["t"].append(t)
        log["f_true"].append(f[2])
        log["ft_tan"].append(np.linalg.norm(f[:2]))
        log["xy"].append(data.site_xpos[ctl.site_id][:2].copy())
        log["xy_des"].append(xy_des.copy())
        log["moving"].append(0 <= k < len(path.pos))
        log["k"].append(k)
        log["pen"].append(-min((data.contact[i].dist for i in range(data.ncon)), default=0.0))
        if t_move is not None and k >= len(path.pos) + int(TAIL / dt):
            break

    log = {k_: np.array(v_) for k_, v_ in log.items()}
    return {"mode": mode, "path": kind, "v": v, "tilt_deg": tilt, "mu": mu, "k_xy": k_xy, "dmin": dmin, "fric_ff": fric_ff,
            "metrics": metrics(log, path, mu, k_xy), "log": log, "path_obj": path}


def metrics(log: dict, path: Path2D, mu: float, k_xy: float) -> dict:
    mv = log["moving"]
    f = log["f_true"][mv]
    xy = log["xy"][mv]
    ks = log["k"][mv]
    ct = path.cross_track(xy)
    cruise = path.cruise[ks]
    tangent = path.vel[ks] / np.maximum(np.linalg.norm(path.vel[ks], axis=1, keepdims=True), 1e-12)
    lag = np.sum((path.pos[ks] - xy) * tangent, axis=1)[cruise]
    pred_lag = mu * F_DES / k_xy
    return {
        "force_rmse_N": float(np.sqrt(np.mean((f - F_DES) ** 2))),
        "force_min_N": float(f.min()),
        "force_max_N": float(f.max()),
        "contact_losses": int(f[0] <= 0) + int(np.sum((f[1:] <= 0) & (f[:-1] > 0))),
        "steps_without_contact": int(np.sum(f <= 0.0)),
        "cross_track_rms_mm": float(np.sqrt(np.mean(ct ** 2)) * 1e3),
        "cross_track_max_mm": float(np.abs(ct).max() * 1e3),
        "cross_track_mean_mm": float(ct.mean() * 1e3),
        "lag_cruise_mm": float(lag.mean() * 1e3) if len(lag) else None,
        "lag_pred_mm": pred_lag * 1e3,
        "lag_rel_err": float(abs(lag.mean() - pred_lag) / pred_lag) if len(lag) else None,
        "friction_ratio_cruise": float(np.median((log["ft_tan"][mv] / np.maximum(f, 1e-9))[cruise & (f > 1)])),
        "contact_k_eff_N_per_m": contact_k(log),
        "duration_s": float(mv.sum() * 1e-3),
    }


def contact_k(log: dict) -> float:
    """Effective contact stiffness from the static hold: mean force / mean penetration over the
    last 0.5 s before motion starts (chattering runs bias any estimate taken while moving)."""
    i0 = int(np.argmax(log["moving"]))
    w = slice(i0 - 500, i0)
    return float(log["f_true"][w].mean() / log["pen"][w].mean())


def plot(trials: list[dict], path: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for col, kind in enumerate(("line", "circle")):
        ax_f, ax_xy = axes[0, col], axes[1, col]
        for tr in trials:
            if tr["path"] != kind:
                continue
            lg = tr["log"]
            mv = lg["moving"]
            t = lg["t"][mv] - lg["t"][mv][0]
            label = f"{tr['mode']}, {'flat' if tr['tilt_deg'] == 0 else 'tilt 3°'}"
            ax_f.plot(t, lg["f_true"][mv], label=label)
            if tr["mode"] == "force":
                ax_xy.plot(lg["xy"][mv][:, 0] * 1e3, lg["xy"][mv][:, 1] * 1e3, label=label)
        p = next(tr["path_obj"] for tr in trials if tr["path"] == kind)
        ax_xy.plot(p.pos[:, 0] * 1e3, p.pos[:, 1] * 1e3, "k--", lw=0.8, label="desired")
        ax_f.axhline(F_DES, color="k", ls="--", lw=0.8)
        ax_f.set_title(f"{kind}, 5 cm/s, μ={MU}, compliant pad: true Fz while moving")
        ax_f.set_xlabel("time since motion start [s]")
        ax_f.set_ylabel("Fz [N]")
        ax_f.legend(fontsize=8)
        ax_f.grid(alpha=0.3)
        ax_xy.set_aspect("equal")
        ax_xy.set_title(f"{kind}: probe path (force control)")
        ax_xy.set_xlabel("x [mm]")
        ax_xy.set_ylabel("y [mm]")
        ax_xy.legend(fontsize=8)
        ax_xy.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)


def strip(tr: dict) -> dict:
    return {k: v for k, v in tr.items() if k not in ("log", "path_obj")}


def main():
    main_trials = [run(mode, kind, tilt=tilt) for kind in ("line", "circle")
                   for tilt in (0.0, 3.0) for mode in ("impedance", "force")]
    extra = [run("force", "circle", v=v) for v in (0.02, 0.10, 0.20)]
    extra += [run("force", "circle", mu=1.0), run("force", "line", mu=1.0),
              run("force", "circle", k_xy=2000.0), run("force", "line", k_xy=2000.0)]
    # The pre-registered setup: rigid probe (MuJoCo default contact). It chatters; kept as a result.
    rigid = [run(mode, kind, dmin=None) for kind in ("line", "circle") for mode in ("impedance", "force")]
    # Where does chatter start? Contact softness sweep, force control, line, 5 cm/s.
    stiff_sweep = [run(mode, "line", dmin=d) for d in (0.0, 0.2, 0.3, 0.5, 0.7, None)
                   for mode in ("impedance", "force")]

    bar_a = [tr for tr in main_trials if tr["mode"] == "force"]
    bar_b = [tr for tr in main_trials if tr["mode"] == "force" and tr["tilt_deg"] == 0.0]
    res = {
        "mu": MU, "a_max": A_MAX, "line_len": LINE_LEN, "circle_r": CIRCLE_R,
        "main": [strip(t) for t in main_trials],
        "extra": [strip(t) for t in extra],
        "rigid_probe": [strip(t) for t in rigid],
        "contact_stiffness_sweep": [strip(t) for t in stiff_sweep],
        "pass": {
            "a_no_contact_loss_and_rmse_lt_1N": all(
                t["metrics"]["steps_without_contact"] == 0 and t["metrics"]["force_rmse_N"] < 1.0
                for t in bar_a),
            "b_lag_within_15pct": all(t["metrics"]["lag_rel_err"] < 0.15 for t in bar_b),
            "a_on_rigid_probe_as_preregistered": all(
                t["metrics"]["steps_without_contact"] == 0 and t["metrics"]["force_rmse_N"] < 1.0
                for t in rigid if t["mode"] == "force"),
        },
    }
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "m2_wipe.json").write_text(json.dumps(res, indent=2))
    plot(main_trials, RESULTS / "m2_wipe.png")
    for tr in main_trials + extra + rigid + stiff_sweep:
        m = tr["metrics"]
        print(f"{tr['mode']:9s} {tr['path']:6s} v={tr['v']:.2f} tilt={tr['tilt_deg']:.0f} mu={tr['mu']} "
              f"dmin={tr['dmin']} k_c={m['contact_k_eff_N_per_m']:.1e} "
              f"kxy={tr['k_xy']:.0f} | F rmse {m['force_rmse_N']:.3f} min {m['force_min_N']:.2f} "
              f"max {m['force_max_N']:.2f} noContact {m['steps_without_contact']} | ct rms "
              f"{m['cross_track_rms_mm']:.2f} mean {m['cross_track_mean_mm']:+.2f} max "
              f"{m['cross_track_max_mm']:.2f} | lag {m['lag_cruise_mm']:.2f} (pred {m['lag_pred_mm']:.1f}) "
              f"fric {m['friction_ratio_cruise']:.3f}")
    print(json.dumps(res["pass"]))


if __name__ == "__main__":
    main()
