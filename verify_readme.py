"""Check the numbers quoted in README.md against results/. Exits non-zero on any mismatch."""
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).parent
RES = ROOT / "results"
README = (ROOT / "README.md").read_text()
fails = 0


def load(name: str) -> dict:
    return json.loads((RES / name).read_text())


def check(label: str, computed: float, quoted: float, tol: float) -> None:
    global fails
    ok = abs(computed - quoted) <= tol
    fails += not ok
    print(f"{'ok  ' if ok else 'FAIL'} {label}: results {computed:.6g}, README {quoted:g}")


def text(label: str, needle: str) -> None:
    global fails
    ok = needle in README
    fails += not ok
    print(f"{'ok  ' if ok else 'FAIL'} README contains {label}: {needle!r}")


def rows(table, **want):
    return [t for t in table if all(t.get(k) == v for k, v in want.items())]


def pooled(table, field="peak_max_N", **want):
    rs = rows(table, **want)
    return sum(t["success"] for t in rs), max(t[field] for t in rs)


m3 = load("m3_peg.json")
B = dict(v=0.01, pitch=0.00075, mu=0.5, dmin=0.2)
A = dict(v=0.02, pitch=0.001, mu=0.5, dmin=0.2)


def m3_row(ctrl, h, cfg):
    return [t for t in rows(m3["table"], controller=ctrl, height_err=h, **cfg) if t["r"] > 0]


for h, ok_f, pk_f, ok_s, pk_s in ((-0.003, 150, 14.5, 0, 0.0), (0.0, 150, 14.5, 150, 17.5), (0.003, 150, 14.6, 150, 52.4)):
    f, s = m3_row("force_spiral", h, B), m3_row("stiff_spiral", h, B)
    check(f"M3 B force inserted, h={h}", sum(t["success"] for t in f), ok_f, 0)
    check(f"M3 B force peak, h={h}", max(t["peak_max_N"] for t in f), pk_f, 0.05)
    check(f"M3 B stiff inserted, h={h}", sum(t["success"] for t in s), ok_s, 0)
    check(f"M3 B stiff peak, h={h}", max(t["peak_max_N"] for t in s), pk_s, 0.05)
text("M3 config A row", "148 / 149 / 150")
for h, want in ((-0.003, 148), (0.0, 149), (0.003, 150)):
    check(f"M3 A force inserted, h={h}", sum(t["success"] for t in m3_row("force_spiral", h, A)), want, 0)
check("M3 A bar d no-contact %", m3["bars"]["A"]["d_search_nocontact_pct"], 5.29, 0.005)
check("M3 B bar d no-contact %", m3["bars"]["B"]["d_search_nocontact_pct"], 1.09, 0.005)
check("M3 A peak", m3["bars"]["A"]["b_peak_max_N"], 16.5, 0.05)
check("M3 rigid B inserted", sum(t["success"] for t in rows(m3["table"], controller="force_spiral", v=0.01, pitch=0.00075, dmin=None)), 150, 0)
check("M3 rigid B peak", max(t["peak_max_N"] for t in rows(m3["table"], controller="force_spiral", v=0.01, pitch=0.00075, dmin=None)), 27.5, 0.05)

v3 = load("m4_v3.json")
for cond, ok_f, pk_f, ok_s, pk_s in (("payload -0.5 kg", 90, 19.4, 0, None), ("friction 1.0 Nm", 84, 34.4, 77, 110.2),
                                     ("C-", 150, 18.8, 144, 52.6), ("noise 1.0 N", 80, 61.0, 90, 17.6), ("yaw 2.0 deg", 66, None, 57, None)):
    fo, fp = pooled(v3["table"], cond=cond, controller="force_spiral")
    so, sp = pooled(v3["table"], cond=cond, controller="stiff_spiral")
    check(f"M4 v3 force inserted, {cond}", fo, ok_f, 0)
    check(f"M4 v3 stiff inserted, {cond}", so, ok_s, 0)
    if pk_f is not None:
        check(f"M4 v3 force peak, {cond}", fp, pk_f, 0.6)
    if pk_s is not None:
        check(f"M4 v3 stiff peak, {cond}", sp, pk_s, 0.6)
for cond in ("C+", "C-"):
    b = v3["bars"][cond]
    print(f"     v3 bars {cond}: a {b['a_pass']} b {b['b_pass']} c {b['c_pass']} d {b['d_pass']}")
    fails += not (b["a_pass"] and b["b_pass"] and b["c_pass"] and not b["d_pass"])
check("M4 v3 bar d steps C+", v3["bars"]["C+"]["d_m2_steps_without_contact"], 4, 0)
check("M4 v3 bar d steps C-", v3["bars"]["C-"]["d_m2_steps_without_contact"], 39, 0)
check("nominal peak v3", pooled(v3["table"], cond="none", controller="force_spiral")[1], 21, 0.6)

v2 = load("m4_v2.json")
check("nominal peak v2", pooled(v2["table"], cond="none", controller="force_spiral")[1], 14, 0.2)

v1 = load("m4_robustness.json")
p = lambda data, cond, mode: next(x for x in data["press"] if x["cond"] == cond and x["mode"] == mode)
check("M4 v1 false-contact spike, noise 1 N", p(v1, "noise 1.0 N", "force")["m1"]["peak_N"], 199, 0.5)
check("M4 v1 false-contact spike, bias +2 N", p(v1, "bias +2.0 N", "force")["m1"]["peak_N"], 193, 0.5)
check("M4 v1 payload impedance error", abs(p(v1, "payload -0.5 kg", "impedance")["m1"]["steady_err_N"]), 4.9, 0.05)
check("M4 v1 payload force error", abs(p(v1, "payload -0.5 kg", "force")["m1"]["steady_err_N"]), 0.0, 0.005)
check("M4 v1 bias passes through force control", abs(p(v1, "bias -2.0 N", "force")["m1"]["steady_err_N"]), 2.0, 0.01)

m1 = load("m1_press.json")
nominal_force = next(t for t in m1["trials"] if t["mode"] == "force" and t["table_offset_m"] == 0.0)
check("M1 force RMSE", nominal_force["metrics"]["rmse_N"], 0.14, 0.005)
check("M1 impedance steady error, 5 mm", abs(next(t for t in m1["trials"] if t["mode"] == "impedance" and t["table_offset_m"] == 0.005)["metrics"]["steady_err_N"]), 2.5, 0.05)

m5 = load("m5_cpp.json")
check("M5 law diff", m5["law"]["max_abs_diff_Nm"], 8.5e-14, 5e-16 * 100)
check("M5 closed diff", m5["closed"]["max_abs_diff_Nm"], 3.5e-11, 5e-13)
check("M5 C++ p50", m5["law"]["compute_us"]["p50"], 3.2, 0.05)
check("M5 C++ p99.9", m5["law"]["compute_us"]["p999"], 24.5, 0.05)
check("M5 Python p50", m5["python_compute_us"]["p50"], 65, 0.5)
check("M5 Python p99.9", m5["python_compute_us"]["p999"], 304, 0.5)
for key, misses, wake, worst in (("sleep_idle", 12, 198, 1.9), ("spin_idle", 16, 6.6, 4.4), ("sleep_loaded_16_procs", 2404, 3300, 32.9)):
    r = m5["rt"][key]
    check(f"M5 {key} misses", r["deadline_misses"], misses, 0)
    check(f"M5 {key} wake p99 us", r["wake_latency_us"]["p99"], wake, max(0.05, wake * 0.005))
    check(f"M5 {key} worst tick ms", r["tick_end_us"]["max"] / 1000, worst, 0.05)

for f in re.findall(r"!\[[^\]]*\]\(([^)]+)\)", README):
    exists = (ROOT / f).exists()
    fails += not exists
    print(f"{'ok  ' if exists else 'FAIL'} image exists: {f}")

print(f"\n{fails} failure(s)")
sys.exit(1 if fails else 0)
