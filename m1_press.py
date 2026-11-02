"""M1: press a ball-tipped probe on a table and hold Fz = 10 N for 5 s.

Two controllers, same approach (pos_des ramps down at 2 cm/s, k_pos = 500 N/m):
  impedance : ramp stops at believed surface - F/k, force comes from the spring offset
  force     : on contact (tared F/T > 1 N) z switches to PI on measured Fz with
              feedforward; x/y/rot stay impedance

Each runs on the nominal table and on a table that is secretly 5 mm higher or
lower than the controller believes. Metrics use the ground-truth contact force.
Pass bars are fixed in PLAN.md (M1) before this was run.
"""
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib
import mujoco
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent / "src"))
from force_sensing import FlangeFT, FreeSpaceZero, true_contact_force  # noqa: E402
from franka_model import HOME_Q, PROBE_RADIUS, TABLE_TOP, build_model, reset_home  # noqa: E402
from impedance import CartesianImpedance, ImpedanceGains  # noqa: E402
from mismatch import Mismatch  # noqa: E402

RESULTS = Path(__file__).parent / "results"
F_DES = 10.0          # N
K_POS = 500.0         # N/m
V_APPROACH = 0.02     # m/s
HOLD = 5.0            # s after first contact
TARE_TIME = 0.5       # s hanging still before the approach starts
CONTACT_THRESH = 1.0  # N, tared F/T reading that switches to force control


@dataclass
class ForcePI:
    """PI on contact force along world z with feedforward and anti-windup."""
    kp: float = 0.5
    ki: float = 20.0
    ramp: float = 0.02         # s, chosen by the ramp sweep in main() on the nominal table; reference ramp from the switch command up to F_DES
    i_max: float = 10.0        # N, clamp on the integral contribution
    f_des: float = F_DES       # N, setpoint the reference ramps up to

    def reset(self, f_switch: float, t_switch: float):
        """f_switch = downward force the impedance law was commanding at the switch.

        Starting the ramp there (bumpless transfer) matters: contact is detected during
        the impact spike, and v1 started the ramp from that spiked measurement, which
        doubled the push in one tick (peak 17.5 N vs the 10.7 N impact itself).
        """
        self.integral = 0.0
        self.f0 = f_switch
        self.t0 = t_switch

    def reference(self, t: float) -> float:
        a = min(1.0, (t - self.t0) / self.ramp)
        return self.f0 + a * (self.f_des - self.f0)

    def wrench_z(self, f_meas: float, t: float, dt: float) -> float:
        f_ref = self.reference(t)
        e = f_ref - f_meas
        self.integral = np.clip(self.integral + self.ki * e * dt, -self.i_max, self.i_max)
        # Push down (negative z) with the reference force plus correction.
        return -(f_ref + self.kp * e + self.integral)


def run(mode: str, table_offset: float = 0.0, pi: ForcePI | None = None,
        mm: Mismatch | None = None, robust: bool = False) -> dict:
    """One press trial. table_offset = true surface height - believed surface height.
    robust = M4 v2 contact detection (FreeSpaceZero) instead of the fixed threshold."""
    mm = mm or Mismatch()
    model = build_model(task="press", table_top=TABLE_TOP + table_offset)
    believed = mm.apply(model)
    delay = mm.make_delay()
    data = mujoco.MjData(model)
    reset_home(model, data)
    data.ctrl[7] = 255  # gripper open, out of the way of the probe
    ctl = CartesianImpedance(model, ImpedanceGains(k_pos=K_POS, q_rest=HOME_Q.copy()), site="probe",
                             believed=believed)
    ft = mm.make_ft(model)
    zero = FreeSpaceZero(model.opt.timestep) if robust else None
    pi = pi or ForcePI()
    dt = model.opt.timestep

    p0, quat0 = ctl.tcp_pose(data)
    z_contact_believed = TABLE_TOP + PROBE_RADIUS
    z_stop = z_contact_believed - F_DES / K_POS if mode == "impedance" else -np.inf

    log = {k: [] for k in ("t", "f_true", "f_est", "z", "z_des")}
    t_contact = None      # first ground-truth contact
    switched = False
    t = 0.0
    tare_steps = int(TARE_TIME / dt)
    max_steps = int(15.0 / dt)

    for step in range(max_steps):
        t = step * dt
        if step == tare_steps:
            ft.tare(data)

        z_des = p0[2] - V_APPROACH * max(0.0, t - TARE_TIME)
        z_des = max(z_des, z_stop)
        pos_des = np.array([p0[0], p0[1], z_des])

        f_est = ft.contact_force(data)[2] if step >= tare_steps else 0.0
        override = None
        if mode == "force":
            touched = f_est > CONTACT_THRESH
            if zero is not None and step >= tare_steps:
                f_est, touched = zero.update(f_est)
            if not switched and step > tare_steps and touched:
                switched = True
                z_now = data.site_xpos[ctl.site_id][2]
                pi.reset(max(0.0, K_POS * (z_now - z_des)), t)
            if switched:
                override = {2: pi.wrench_z(f_est, t, dt)}

        data.ctrl[:7] = delay(ctl.step(data, pos_des, quat0, wrench_override=override))
        mujoco.mj_step(model, data)

        f_true = true_contact_force(model, data, "probe_ball")[2]
        if t_contact is None and f_true > 0.0:
            t_contact = t
        log["t"].append(t)
        log["f_true"].append(f_true)
        log["f_est"].append(f_est)
        log["z"].append(data.site_xpos[ctl.site_id][2])
        log["z_des"].append(z_des)
        if t_contact is not None and t >= t_contact + HOLD:
            break

    log = {k: np.array(v) for k, v in log.items()}
    return {"mode": mode, "table_offset_m": table_offset, "t_contact": t_contact,
            "metrics": metrics(log, t_contact), "log": log}


def metrics(log: dict, t_contact: float | None) -> dict:
    if t_contact is None:
        return {"contact": False}
    w = log["t"] >= t_contact
    f = log["f_true"][w]
    t = log["t"][w] - t_contact
    err = f - F_DES
    outside = np.where(np.abs(err) >= 0.5)[0]
    settle = float(t[outside[-1] + 1]) if len(outside) and outside[-1] + 1 < len(t) else (
        0.0 if not len(outside) else None)
    steady = t >= HOLD - 2.0          # last 2 s
    lost = int(np.sum((f[1:] <= 0) & (f[:-1] > 0)))
    return {
        "contact": True,
        "rmse_N": float(np.sqrt(np.mean(err ** 2))),
        "peak_N": float(f.max()),
        "overshoot_N": float(max(0.0, f.max() - F_DES)),
        "settle_s": settle,
        "steady_mean_N": float(f[steady].mean()),
        "steady_err_N": float(f[steady].mean() - F_DES),
        "steady_rmse_N": float(np.sqrt(np.mean(err[steady] ** 2))),
        "contact_losses": lost,
        "ft_vs_true_steady_mean_abs_N": float(np.mean(np.abs(log["f_est"][w][steady] - f[steady]))),
    }


def plot(trials: list[dict], path: Path) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(15, 4), sharey=True)
    for ax, off in zip(axes, (-0.005, 0.0, 0.005)):
        for tr in trials:
            if tr["table_offset_m"] != off:
                continue
            lg = tr["log"]
            t = lg["t"] - tr["t_contact"]
            ax.plot(t, lg["f_true"], label=tr["mode"])
        ax.axhline(F_DES, color="k", ls="--", lw=0.8)
        ax.set_xlim(-0.5, HOLD)
        ax.set_title(f"table {off * 1000:+.0f} mm vs believed")
        ax.set_xlabel("time since first contact [s]")
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("true contact force Fz [N]")
    axes[0].legend()
    fig.tight_layout()
    fig.savefig(path, dpi=120)


def main():
    trials = [run(mode, off) for off in (-0.005, 0.0, 0.005) for mode in ("impedance", "force")]
    summary = [{k: v for k, v in tr.items() if k != "log"} for tr in trials]

    nominal = {tr["mode"]: tr["metrics"] for tr in trials if tr["table_offset_m"] == 0.0}
    res = {
        "f_des_N": F_DES, "k_pos": K_POS, "v_approach": V_APPROACH,
        "force_pi": vars(ForcePI()),
        "trials": summary,
        "pass": {
            "a_force_rmse_lt_1N": nominal["force"]["rmse_N"] < 1.0,
            "b_ft_vs_true_lt_0.1N": all(tr["metrics"]["ft_vs_true_steady_mean_abs_N"] < 0.1
                                        for tr in trials),
        },
    }
    # Ramp sweep: peak force vs RMSE trade-off (a step reference spikes, a slow ramp sags).
    res["ramp_sweep"] = []
    for ramp in (0.001, 0.02, 0.05, 0.1, 0.2):
        for off in (-0.005, 0.0, 0.005):
            m = run("force", off, ForcePI(ramp=ramp))["metrics"]
            res["ramp_sweep"].append({"ramp_s": ramp, "table_offset_m": off,
                                      **{k: m[k] for k in ("rmse_N", "peak_N", "settle_s")}})
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "m1_press.json").write_text(json.dumps(res, indent=2))
    plot(trials, RESULTS / "m1_press.png")
    for tr in summary:
        m = tr["metrics"]
        print(f"{tr['mode']:9s} table {tr['table_offset_m'] * 1000:+.0f}mm | rmse {m['rmse_N']:.3f} "
              f"peak {m['peak_N']:.2f} settle {m['settle_s']} steady_err {m['steady_err_N']:+.3f} "
              f"losses {m['contact_losses']} ft_err {m['ft_vs_true_steady_mean_abs_N']:.4f}")
    print(json.dumps(res["pass"]))


if __name__ == "__main__":
    main()
