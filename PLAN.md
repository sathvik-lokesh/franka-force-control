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

Run: `.venv/bin/python m0_smoke.py`

## API notes
- MuJoCo 3.17: `mj_fullM(model, data, dst)`, old `mj_fullM(model, dst, data.qM)` is gone (`data.qM` doesn't exist).

## Milestones (pass bars fixed BEFORE running)

### M0: torque model + impedance sanity (done 2026-10-29)
- A: gravity comp only, 3 s at home -> max joint drift < 1e-3 rad. **Got 2e-14** (trivially exact in
  sim because qfrc_bias IS the model; proves wiring only, not realism, M4 tests this).
- B: k=500 N/m, 10 N push at TCP on x/y/z -> deflection = F/k = 20 mm within 5%. **Got <0.03% on all axes.**

### M1: force sensing + constant-force pressing
- Add a table + F/T sensor at the flange (MuJoCo `force`/`torque` sensors on a site).
- Task: come down, hold Fz = 10 N on the table for 5 s.
- Compare: pure impedance (force = stiffness × penetration offset) vs explicit force control
  (PI on Fz in z, impedance in x/y/rot).
- Metrics: force RMSE, overshoot at impact, settle time. Bar: explicit force RMSE < 1 N.

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
