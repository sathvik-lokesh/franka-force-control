"""M4: re-run M1, M2 and M3 with the world no longer matching the controller's model.

Factors, levels, seeds and pass bars are fixed in PLAN.md (M4) before this was run. Nothing is
tuned here. Writes results/m4_robustness.json and .png.

Usage: m4_robustness.py               v1, full run
       m4_robustness.py --v2          v2 fixes on fresh seeds, writes results/m4_v2*.
       m4_robustness.py --plot [--v2] redraw the figure from the saved JSON
"""
import argparse
import json
import os
import sys
from multiprocessing import Pool
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent / "src"))
import m1_press as m1  # noqa: E402
import m2_wipe as m2  # noqa: E402
import m3_peg as m3  # noqa: E402
from mismatch import Mismatch  # noqa: E402

RESULTS = Path(__file__).parent / "results"
CONFIG_B = dict(v=0.01, pitch=0.00075)
BAR_SEEDS = {False: range(100, 150), True: range(200, 250)}       # key = v2
SWEEP_SEEDS = {False: range(100, 130), True: range(200, 230)}
COMBINED = dict(joint_friction_Nm=0.3, ft_noise_N=0.2, delay_ms=2, hole_yaw_deg=0.5)

CONDITIONS: dict[str, dict] = {
    "none": {},
    "C+": dict(payload_kg=0.2, ft_bias_N=0.3, **COMBINED),
    "C-": dict(payload_kg=-0.2, ft_bias_N=-0.3, **COMBINED),
    **{f"payload {p:+.1f} kg": dict(payload_kg=p) for p in (-0.5, -0.2, 0.2, 0.5)},
    **{f"friction {f} Nm": dict(joint_friction_Nm=f) for f in (0.1, 0.3, 0.5, 1.0)},
    **{f"noise {n} N": dict(ft_noise_N=n) for n in (0.1, 0.2, 0.5, 1.0)},
    **{f"bias {b:+.1f} N": dict(ft_bias_N=b) for b in (-2.0, -0.5, 0.5, 2.0)},
    **{f"delay {d} ms": dict(delay_ms=d) for d in (1, 2, 5)},
    **{f"yaw {y} deg": dict(hole_yaw_deg=y) for y in (0.5, 1.0, 2.0)},
}
BAR_CONDITIONS = ("none", "C+", "C-")


def m3_job(job: dict) -> dict:
    kw = dict(job)
    cond = kw.pop("cond")
    params = CONDITIONS.get(cond, {})
    res = m3.run(mm=Mismatch(**params, seed=kw["seed"]), **kw)
    res["cond"] = cond
    return res


def press_job(job: dict) -> dict:
    """M1 (nominal table) and M2 (pad, flat, line, 5 cm/s) for one condition and controller."""
    cond, mode = job["cond"], job["mode"]
    params = {k: v for k, v in CONDITIONS[cond].items() if k != "hole_yaw_deg"}
    out = {"cond": cond, "mode": mode}
    a = m1.run(mode, mm=Mismatch(**params), robust=job["robust"])
    metrics = a["metrics"]
    out["m1"] = {k: metrics.get(k) for k in ("contact", "rmse_N", "peak_N", "steady_err_N", "settle_s")}
    try:
        b = m2.run(mode, "line", mm=Mismatch(**params), robust=job["robust"])["metrics"]
        out["m2"] = {k: b[k] for k in ("force_rmse_N", "force_min_N", "force_max_N", "steps_without_contact",
                                       "lag_cruise_mm", "cross_track_rms_mm")}
    except (ValueError, IndexError) as e:     # e.g. a run that never reaches the moving phase
        out["m2"] = {"error": repr(e)}
    return out


def m3_jobs(v2: bool = False) -> list[dict]:
    jobs = []
    for cond in CONDITIONS:
        seeds = BAR_SEEDS[v2] if cond in BAR_CONDITIONS else SWEEP_SEEDS[v2]
        jobs += [dict(cond=cond, controller=c, r=r, seed=s, robust=v2, **CONFIG_B)
                 for c in ("force_spiral", "stiff_spiral") for r in m3.LEVELS for s in seeds]
    return jobs


def addendum_jobs() -> list[dict]:
    """M3's stiff conditions again, on M3's own test seeds, with the really force-blind baseline."""
    return [dict(cond="m3_blind_baseline", controller="stiff_spiral", r=r, seed=s, height_err=h, **cfg)
            for cfg in m3.CONFIGS.values() for h in m3.HEIGHTS for r in m3.LEVELS for s in m3.TEST_SEEDS]


def bars(table: list[dict], press: list[dict]) -> dict:
    out = {}
    for cond in ("C+", "C-"):
        rows = [t for t in table if t["cond"] == cond and t["controller"] == "force_spiral"]
        p = next(x for x in press if x["cond"] == cond and x["mode"] == "force")
        peaks = [t["peak_max_success_N"] for t in rows if t["peak_max_success_N"] is not None]
        m2_ok = "error" not in p["m2"]
        out[cond] = {
            "a_success_per_r": {str(t["r"]): t["success"] for t in rows},
            "a_pass": all(t["success"] >= 45 for t in rows),
            "b_peak_max_N": max(peaks) if peaks else None,
            "b_pass": bool(peaks) and max(peaks) < 30.0,
            "c_m1_rmse_N": p["m1"]["rmse_N"],
            "c_pass": p["m1"]["rmse_N"] is not None and p["m1"]["rmse_N"] < 1.0,
            "d_m2_rmse_N": p["m2"].get("force_rmse_N"),
            "d_m2_steps_without_contact": p["m2"].get("steps_without_contact"),
            "d_pass": m2_ok and p["m2"]["force_rmse_N"] < 1.0 and p["m2"]["steps_without_contact"] == 0,
        }
    return out


def main(v2: bool = False) -> None:
    jobs = m3_jobs(v2) + ([] if v2 else addendum_jobs())
    stem = "m4_v2" if v2 else "m4_robustness"
    press_jobs = [dict(cond=c, mode=mode, robust=v2) for c in CONDITIONS for mode in ("force", "impedance")]
    print(len(jobs), "peg trials,", len(press_jobs), "press/wipe jobs", flush=True)
    with Pool(12) as pool:
        press = pool.map(press_job, press_jobs, chunksize=1)
        print("press/wipe done", flush=True)
        trials = pool.map(m3_job, jobs, chunksize=4)

    keys = ("cond", "controller", "r", "height_err", "v", "pitch")
    groups: dict[tuple, list[dict]] = {}
    for x in trials:
        groups.setdefault(tuple(x[k] for k in keys), []).append(x)
    table = []
    for k, rs in groups.items():
        row = {**dict(zip(keys, k)), **m3.summarize(rs)}
        row["end_phase"] = {p: sum(x["final_phase"] == p for x in rs if not x["success"])
                            for p in {x["final_phase"] for x in rs if not x["success"]}}
        table.append(row)
    res = {"conditions": CONDITIONS, "bars": bars(table, press), "press": press, "table": table}
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / f"{stem}.json").write_text(json.dumps(res, indent=1))
    (RESULTS / ("m4_v2_trials.json" if v2 else "m4_trials.json")).write_text(json.dumps(trials))
    plot(res, stem)
    report(res)


def report(res: dict) -> None:
    print(json.dumps(res["bars"], indent=1))
    print("\nM3 pooled over r: cond | force ok/n peak rmse nocontact% | stiff ok/n peak | force end phases")
    for cond in CONDITIONS:
        line = [f"{cond:18s}"]
        for ctrl in ("force_spiral", "stiff_spiral"):
            rows = [t for t in res["table"] if t["cond"] == cond and t["controller"] == ctrl]
            ok, n = sum(t["success"] for t in rows), sum(t["n"] for t in rows)
            rm = [t["search_rmse_N"] for t in rows if t["search_rmse_N"] is not None]
            nc = [t["search_nocontact_pct"] for t in rows if t["search_nocontact_pct"] is not None]
            line.append(f"{ok:3d}/{n} per r {[t['success'] for t in rows]} peak {max(t['peak_max_N'] for t in rows):6.1f} "
                        f"rmse {np.mean(rm) if rm else float('nan'):5.2f} nc {np.mean(nc) if nc else float('nan'):5.2f}")
        ph: dict[str, int] = {}
        for t in res["table"]:
            if t["cond"] == cond and t["controller"] == "force_spiral":
                for p, c in t["end_phase"].items():
                    ph[p] = ph.get(p, 0) + c
        print(" | ".join(line), "|", ph)
    print("\nM1 / M2: cond | mode | M1 rmse peak steady | M2 rmse max noContact lag")
    for x in res["press"]:
        a, b = x["m1"], x["m2"]
        fmt = lambda v: "-" if v is None else f"{v:.2f}"
        print(f"{x['cond']:18s} {x['mode']:9s} | {fmt(a['rmse_N'])} {fmt(a['peak_N'])} {fmt(a['steady_err_N'])} | "
              + (f"{b['force_rmse_N']:.2f} {b['force_max_N']:.1f} {b['steps_without_contact']} {b['lag_cruise_mm']:.2f}"
                 if "error" not in b else b["error"]))
    print("\nAddendum, M3 stiff_spiral really force-blind, seeds 0-49: v pitch height r ok peak")
    for t in res["table"]:
        if t["cond"] == "m3_blind_baseline":
            print(t["v"], t["pitch"], t["height_err"], t["r"], f"{t['success']}/{t['n']}", round(t["peak_max_N"], 1))


def plot(res: dict, stem: str = "m4_robustness") -> None:
    """Four panels sharing one condition axis: M3 success, M3 peak force, M1 RMSE, M2 RMSE."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ink, muted, grid, surface = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
    ours, base = "#2a78d6", "#eb6834"
    conds = list(res["conditions"])
    y = np.arange(len(conds))[::-1]

    def m3_vals(ctrl, field):
        out = []
        for c in conds:
            rows = [t for t in res["table"] if t["cond"] == c and t["controller"] == ctrl]
            if field == "pct":
                out.append(100 * sum(t["success"] for t in rows) / sum(t["n"] for t in rows))
            else:
                out.append(max(t["peak_max_N"] for t in rows))
        return out

    def press_vals(mode, key, field):
        return [next(x for x in res["press"] if x["cond"] == c and x["mode"] == mode)[key].get(field) or 0.0
                for c in conds]

    panels = (
        ("M3: inserted [%]", m3_vals("force_spiral", "pct"), m3_vals("stiff_spiral", "pct")),
        ("M3: max peak contact force [N]", m3_vals("force_spiral", "peak"), m3_vals("stiff_spiral", "peak")),
        ("M1 press: true force RMSE [N]", press_vals("force", "m1", "rmse_N"), press_vals("impedance", "m1", "rmse_N")),
        ("M2 line: true force RMSE [N]", press_vals("force", "m2", "force_rmse_N"),
         press_vals("impedance", "m2", "force_rmse_N")),
    )
    fig, axes = plt.subplots(1, 4, figsize=(15, 8.5), sharey=True, facecolor=surface)
    h = 0.38
    for ax, (title, a, b) in zip(axes, panels):
        ax.set_facecolor(surface)
        ax.barh(y + h / 2 + 0.01, a, height=h, color=ours, label="Force control")
        ax.barh(y - h / 2 - 0.01, b, height=h, color=base,
                label="No force feedback (M3 stiff position, M1 and M2 impedance)")
        ax.set_title(title, loc="left", fontsize=10.5, color=ink)
        ax.grid(axis="x", color=grid, lw=0.8)
        ax.set_axisbelow(True)
        ax.tick_params(colors=muted, length=0, labelsize=9)
        for s in ("top", "right", "bottom"):
            ax.spines[s].set_visible(False)
        ax.spines["left"].set_color(muted)
    axes[0].set_yticks(y, conds)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(RESULTS / f"{stem}.png", dpi=130)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--v2", action="store_true")
    args = ap.parse_args()
    if args.plot:
        name = "m4_v2" if args.v2 else "m4_robustness"
        saved = json.loads((RESULTS / f"{name}.json").read_text())
        plot(saved, name)
        report(saved)
    else:
        main(args.v2)
