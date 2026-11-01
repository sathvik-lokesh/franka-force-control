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

Run: `.venv/bin/python m0_smoke.py`, `.venv/bin/python m1_press.py`

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

### M2: wiping (force + motion together)
- Hold 10 N while tracing a line/circle on the table.
- Metrics: force RMSE while moving, path error, contact-loss count. Bar: zero contact loss at 5 cm/s.

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
