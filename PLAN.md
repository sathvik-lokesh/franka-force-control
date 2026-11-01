# franka-force-control: PLAN

## Scope
Torque-level Cartesian impedance and force control on a Franka Panda in MuJoCo,
on contact tasks, plus a C++ 1 kHz control loop with measured timing.

## Setup
- MuJoCo 3.17 (pip), Python 3.12 venv in `.venv` (uv). ~230 MB.
- Panda model: `third_party/mujoco_menagerie/franka_emika_panda` (Apache-2.0, sparse clone, 44 MB).
  Arm actuators rewritten to torque motors in `src/franka_model.py` (limits 87/12 Nm, 1 kHz).
- C++ (M5): link against the wheel's `libmujoco.so.3.17.0` + `include/`; Eigen via CMake FetchContent (no sudo).
- No GPU, no dataset. Total disk < 1 GB.

Run: `.venv/bin/python m0_smoke.py`, `.venv/bin/python m1_press.py`, `.venv/bin/python m2_wipe.py`

## API notes
- MuJoCo 3.17: `mj_fullM(model, data, dst)`, old `mj_fullM(model, dst, data.qM)` is gone (`data.qM` doesn't exist).
- Flange `force` sensor reports the force the child (hand+tool) exerts on the parent, so contact on the tool
  = -(R·f_sensor - tare). Checked against summed `mj_contactForce` (src/force_sensing.py). Tare = tool
  weight 7.4556 N (0.76 kg).
- `mj_contactForce` gives the force on geom2 along the normal (geom1->geom2); flip if your geom is geom1.

## Milestones (pass bars fixed BEFORE running)

### M0: torque model + impedance sanity (done 2026-10-29)
- A: gravity comp only, 3 s at home -> max joint drift < 1e-3 rad. **Got 2e-14** (trivially exact in
  sim because qfrc_bias IS the model; proves wiring only, not realism, M4 tests this).
- B: k=500 N/m, 10 N push at TCP on x/y/z -> deflection = F/k = 20 mm within 5%. **Got <0.03% on all axes.**

### M1: force sensing + constant-force pressing (done 2026-11-01)
- Add a table + F/T sensor at the flange (MuJoCo `force`/`torque` sensors on a site).
- Task: come down, hold Fz = 10 N on the table for 5 s.
- Compare: pure impedance (force = stiffness × penetration offset) vs explicit force control
  (PI on Fz in z, impedance in x/y/rot).
- Metrics: force RMSE, overshoot at impact, settle time. Bar: explicit force RMSE < 1 N.
- Fixed 2026-11-01 before running:
  - Tool = rigid probe with a 1 cm ball tip on the flange (one contact point, longer than the fingers).
    F/T sensor = MuJoCo force/torque sensors on a site at the flange, so it reads tool weight + inertia
    + contact, like a real one. Tare in free space before contact (what real Franka setups do).
  - Both controllers approach the same way: pos_des ramps down at 2 cm/s, k_pos = 500 N/m.
    Impedance: ramp stops at believed surface - F/k (20 mm), force = stiffness x offset.
    Force control: on contact (|F| > 1 N), z switches to PI on measured Fz with feedforward + damping;
    x/y/rot stay impedance.
  - RMSE window = 5 s starting at first contact (impact transient included, stricter).
    Settle time = from first contact until |F - 10| < 0.5 N and stays there.
  - Bars: (a) force-control RMSE < 1 N on the nominal table. (b) tared F/T estimate vs ground-truth
    contact force (summed from MuJoCo contacts), steady-state mean abs diff < 0.1 N.
  - Prediction (to check, not a bar): table secretly ±5 mm off -> impedance steady error ≈ k·δ = 2.5 N,
    force control steady error < 0.2 N. This is the reason explicit force control exists.

- **Results 2026-11-01** (ground-truth contact force, 5 s window from first contact):

  | controller | table vs believed | RMSE | peak | settle | steady error |
  |---|---|---|---|---|---|
  | impedance | -5 mm | 3.13 N | 10.97 N | never | -2.52 N |
  | impedance | 0 | 2.08 N | 10.74 N | 0.83 s | -0.03 N |
  | impedance | +5 mm | 3.02 N | 12.47 N | never | +2.47 N |
  | force (PI) | -5 mm | 0.14 N | 10.97 N | 0.05 s | 0.000 N |
  | force (PI) | 0 | 0.14 N | 10.74 N | 0.02 s | 0.000 N |
  | force (PI) | +5 mm | 0.14 N | 10.75 N | 0.02 s | 0.000 N |

  Bar (a) PASS 0.14 N < 1 N. Bar (b) PASS: F/T vs truth 0.0000 N, trivially exact in sim (ideal sensor,
  static tool), M4 adds noise/bias. Prediction confirmed: impedance off by k·δ = ±2.5 N, force control 0.
  Impedance RMSE is partly by design (spring offset ramps in over 1 s at 2 cm/s).
- **Bug found and fixed:** v1 started the PI reference ramp from the *measured* force at
  the switch. Contact is detected during the 10.7 N impact spike, so the push doubled in one tick ->
  17.5 N peak (RMSE 0.21). Fix = bumpless transfer (start the ramp from the force the impedance law was
  already commanding). With the old 0.2 s ramp that sagged -> RMSE 0.988 (barely passing). Ramp sweep
  (in m1_press.json): step 20.0 N peak / 0.23 RMSE; 20 ms 10.7 N / 0.14; 50 ms 0.38; 100 ms 0.65;
  200 ms 0.99. Picked 20 ms on the nominal table; ±5 mm tables (unused in picking) give the same numbers.
- The 10.7 N impact spike itself is the same for both controllers: it's 2 cm/s into a stiff contact
  before any controller can react. Lower approach speed or a compliant tool would cut it (M4 / M3 topic).
- PI gains kp=0.5, ki=20 were the first guess and never tuned.

### M2: wiping (force + motion together) (closed 2026-11-01: bar (b) PASS, bar (a) FAIL (sim limitation))
- Hold 10 N while tracing a line/circle on the table.
- Metrics: force RMSE while moving, path error, contact-loss count. Bar: zero contact loss at 5 cm/s.
- Fixed 2026-11-01 before running:
  - Same probe/sensor/approach as M1; force control switches on contact (bumpless, 20 ms ramp), then
    1 s settle, then the path. Paths: line 10 cm out along +y and back (two trapezoidal segments,
    a_max 0.5 m/s²), circle r = 5 cm one lap. Friction μ = 0.5 on ball + table (steel on laminate-ish);
    MuJoCo default μ = 1 run as a stress case.
  - Impedance change: damping acts on velocity error (J·qd − v_des), i.e. velocity feedforward. Default
    v_des = 0 keeps M0/M1 identical (re-run both to confirm).
  - Conditions: flat table, and table tilted 3° about x (height changes along y, controller not told).
  - Metrics over the moving phase only: true Fz RMSE vs 10 N, min Fz, contact losses (true Fz hits 0),
    cross-track error (geometric distance to the path), along-track lag at constant speed.
  - Bars: (a) force control, μ=0.5, 5 cm/s, line + circle, flat AND tilted: zero contact losses and
    moving-force RMSE < 1 N. (b) physics check: steady along-track lag = μ·N/k = 10 mm within 15%.
  - Predictions (not bars): circle shrinks by r − sqrt(r² − lag²) = 1.0 mm (μ=0.5), 4.2 mm (μ=1, lag 20 mm).
    Impedance on the 3° tilt: surface moves 5.2 mm over 10 cm -> force drifts by ~2.6 N, force control holds.
    k_xy = 2000 -> lag 2.5 mm. Speed sweep 2/5/10/20 cm/s on the circle: report where it degrades.

- **M2 investigation 2026-11-01:**
  - Bar (b) PASS: steady lag 9.4–9.7 mm vs μN/k = 10 mm (line); k_xy = 2000 -> 2.5 mm vs 2.5. Friction
    ratio measured 0.49–0.50. The physics of sliding friction against the spring is right.
  - Bar (a) FAIL on the pre-registered rigid probe: BOTH controllers chatter while sliding, hundreds of
    micro-hops ≤ 20 µm high, 3–7 ms each, ~1.4 s (line) / ~2.3 s (circle) of no contact out of ~4 s;
    force spikes to 20–30 N. Starts exactly at stick->slip breakaway.
  - Ruled out (chatter unchanged): pyramidal vs elliptic cone, impratio 10, integrators euler/implicitfast/
    implicit/RK4, noslip 10, timestep 1 / 0.5 / 0.25 ms (so NOT a dt artifact, I first claimed it was,
    wrong), z damping ×3, k_rot 300/1000 (tool tilt only 0.15°), friction feedforward from the F/T
    (fixes lag 9.7 -> 0.7 mm, chatter stays).
  - Inertia coupling is real (J M⁻¹ Jᵀ: x-force -> z-accel is 0.53 of a z-force; y only −0.05) and explains
    why circles are worse than the y-line, but cancelling friction didn't remove chatter, so not the root.
  - Friction sweep, rigid line: μ=0 -> 12 ms no contact (485 Hz), μ=0.01 -> 124 ms, μ=0.03 -> 295 ms (144 Hz),
    μ=0.1 -> 708 ms (86 Hz ≈ vertical contact resonance √(1.3e6/4.6 kg) ≈ 84 Hz). 0.1 N of friction
    should not physically be able to lift a 10 N preload, so something amplifies it, mechanism unknown.
  - Contact stiffness (measured in the static hold before motion) vs chatter, line, 5 cm/s, both controllers:
    2.4e4 N/m clean · 3.1e4 clean · 3.8e4 ~0.5 s lost · 8.5e4 ~1.0 s · 2.6e5 ~1.3 s · default 1.3e6 ~1.4 s.
    Sharp threshold 3.1–3.8e4 N/m. Pad (solimp dmin 0.2 = 3.1e4 N/m, friction still 0.49): line flat+tilt
    clean (force RMSE 0.02 N, impedance 0.09 N flat / 1.41 N on the 3° tilt, max 12.4 N -> the predicted
    impedance weakness shows), but the circle still chatters (~0.36 s) and the 10/20 cm/s circles worse;
    2 cm/s circle clean (RMSE 0.004 N). dmin 0.0 softens friction (ratio 0.45), don't use.
  - Circle shrink prediction (1.0 mm) not checkable yet: chatter dominates (mean cross-track −0.8 mm pad).
  - **Next: isolation test.** Replace the arm with a free-floating probe body (6-DOF freejoint, same mass,
    Cartesian PD + gravity comp, no arm inertia/nullspace/Jacobian) pressing + sliding on the same contact.
    If it still chatters -> MuJoCo contact + friction behaviour (then compare with a MuJoCo forum/known issue
    and report as a sim limitation, use the pad, and say so). If it doesn't -> it's arm dynamics under my
    controller (then look at the nullspace term and the joint-level response at 85 Hz).
  - Bar (a) as registered failed on the rigid probe, and the M2 write-up says so.
- **M2 conclusion 2026-11-01 (isolation test done, `m2_isolate.py`, `results/m2_isolate.json`):**
  - A free-floating ball on 3 slide joints (no arm, no rotation, friction purely horizontal, vertical
    joint mechanically decoupled) with the same contact + control laws chatters the same: 4.6 kg, μ=0.5
    -> 1478 ms (line) / 2272 ms (circle) no contact vs the arm's 1388 / 2325. 0.86 kg: 339 / 572.
    Physically friction can't lift that probe, so the vertical motion comes from the contact model.
  - MuJoCo docs (Computation chapter, convex model): dropping strict complementarity means "the only way
    to initiate slip is to generate some motion in the normal direction", a resting object rises as it
    starts to slide. Matches: chatter starts at stick->slip breakaway, grows with μ and contact stiffness,
    is controller- and arm-independent. Force RMSE groups roughly by μ·v (μv=0.01 -> 5.3–6.7 N;
    μv=0.025 -> 8.5–9.8 N): consistent, not proof.
  - So: the rigid-probe chatter says nothing about a real Franka. M2 reportable results: friction lag
    physics (bar b), and with the compliant pad on the line: force control 0.02 N RMSE flat / 0.03 N on
    the unseen 3° tilt vs impedance 0.09 / 1.41 N (max 12.4 N), the predicted impedance weakness.
    Circle at 5 cm/s still chatters even with the pad -> reported as sim-limited, not as a controller result.
  - Not done on purpose: softening the contact further until the circle passes (that's tuning the
    environment to the bar).
- **M3 risk note:** spiral search + insertion are sliding contacts. Use the pad (dmin 0.2, 3.1e4 N/m) and
  slow speeds (2 cm/s circle was clean), report contact losses, and cross-check any surprising result
  with the floating-probe trick before blaming the controller.

### M3: peg-in-hole
- Square peg, hole clearance ~1 mm, hole position perturbed ±2–5 mm (unknown to the controller).
- Strategy: approach -> contact -> spiral search under force -> insert with compliance.
- Metric: success rate over 50 seeds per perturbation level, time to insert, max contact force.
- Baseline: same task with stiff position control (expect jamming / force spikes).

### M4: robustness (where the sim stops being perfect)
Model mismatch the real robot has: payload mass error (±0.5 kg), unmodelled joint friction,
F/T sensor noise + bias, 1–5 ms control delay. Re-run M1–M3 and plot how each degrades.

### M5: C++ 1 kHz control loop
- Port the impedance law to C++/Eigen against the MuJoCo C API.
- Measure controller compute time per tick and loop jitter vs a 1 ms deadline
  (plain Linux + WSL2 is NOT real-time, misses are reported, this is not a real-time claim).
- Check: C++ and Python controllers give the same torques on the same states (max abs diff).

### M6: README + demo GIFs
