# Searching posture and foot pressure before lifting a foot

This is an offline static calculation, not a new Gazebo experiment. It keeps the existing 14.42 kg model, ten joints and motor caps. It asks whether lowering the pelvis and moving pressure within the feet can make the next unloading stage feasible.

## What was varied

- Upright pelvis lowered by 0–60 mm in 10 mm increments.
- Pelvis shifted left by 0–120 mm in 5 mm increments.
- Right-foot share: 35%, 30%, 25%, 20%, 10%, 5%, 1% or 0% of weight.
- Pressure locations optimized within each flat rectangular sole, reserving 10 mm from every edge. The usable offsets are ±90 mm fore/aft and ±50 mm sideways.

Each leg's inverse kinematics adjusts its hip, knee and ankle targets to keep its sole fixed and horizontal. The model checks every joint limit and fixed-foot transform. The torso remains upright, and the two leg chains have symmetric angles. This does not search torso lean, asymmetric leg postures, narrower foot spacing or all possible configurations.

The search tried 175 geometric poses; four were unreachable. Across the remaining poses and requested load shares, 1,099 pressure/force optimizations had solutions. A solution means static equilibrium is possible under the pressure assumptions; its required torques may still exceed motor caps.

## Results

For each target below, the table shows the best sampled pose after optimizing pressure to minimize the largest requested motor torque divided by its cap.

| Right-foot share | Pelvis shift left | Pelvis lowering | Peak static torque / cap | Interpretation |
|---|---:|---:|---:|---|
| 35% | 50 mm | 0 mm | 30.1% | Static candidate; pressure feedback not yet implemented |
| 30% | 20 mm | 10 mm | 40.1% | Static candidate |
| 25% | 70 mm | 30 mm | 53.2% | Static candidate with substantial torque margin |
| 20% | 105 mm | 60 mm | 82.7% | Exceeds our 80% planning threshold |
| 10% | 30 mm | 60 mm | 151.6% | Exceeds motor caps |
| 5% | 40 mm | 60 mm | 183.3% | Exceeds motor caps |
| 1% | 50 mm | 60 mm | 211.0% | Exceeds motor caps |
| 0% | 50 mm | 60 mm | 217.3% | Exceeds motor caps |

These are discrete-grid results, not proven global optima or measured walking capability. Different targets can select different postures, so rows do not form a continuous trajectory. Some minima also have ties in pressure/posture choices; the reported optimizer solution is one candidate.

At the best sampled zero-load endpoint, the left hip-roll needs approximately **13.04 Nm** versus its **6 Nm** cap; the left knee needs **13.61 Nm** versus **10 Nm**, and left ankle-roll needs **6.98 Nm** versus **6 Nm**. Moving pressure did not remove the load: it changed which joints must provide its moment.

No new live motion was commanded. In particular, the 25% candidate is not ready to run under the current 25% live support floor: measurement variation needs margin above that floor, and an entire transition must be checked and controlled first.

## The calculation, step by step

### 1. Weight and vertical force balance

`W = 14.42 × 9.81 = 141.4602 N`

For a right-foot target fraction `r`, prescribe `F_R = r W` and `F_L = (1−r) W`. All forces are vertical, nonnegative, and act on a horizontal plane. Horizontal contact forces and yaw moments are set to zero in this model.

### 2. Pressure is allowed to move

A sole-center approximation uses the foot center as its reaction point. Here each foot instead has a pressure offset `(dx, dy)` inside the usable sole rectangle.

Whole-body moment balance requires:

`Σ F_i (foot_y_i + dy_i) = W COM_y`

`Σ F_i (foot_x_i + dx_i) = W COM_x`

All these coordinates are evaluated in the same pelvis-relative coordinate frame; translating the complete calculation into world coordinates gives the same equilibrium.

Instead of optimizing offsets directly, optimize their force-weighted first moments:

`Y_i = F_i dy_i`, `X_i = F_i dx_i`.

The sole constraints become linear: `|Y_i| ≤ 0.05 F_i` and `|X_i| ≤ 0.09 F_i`. At zero foot load, both first moments must become zero. Its pressure location is then undefined, so the report records `null`, not an invented center of pressure.

A nonnegative pressure distribution can realize these resultant locations on a rectangular sole in this ideal static model. Actual Gazebo contact behavior still needs measurement and feedback validation.

### 3. Calculate motor torque

For every joint, sum all downstream gravity moments and its leg's ground-reaction moment. The required static motor torque cancels that sum:

`tau_j = −axis_j · [Σ ((COM_link − joint_j) × gravity_link) + ((contact_point − joint_j) × reaction)]`.

With geometry fixed, torque is linear in the force and its first moments. This lets the optimizer minimize a common utilization variable `t`, subject to `|tau_j| ≤ t cap_j` for every motor.

An optimization with `t > 1` is a diagnosis of excessive torque demand, not permission to issue that command. We retain `t ≤ 0.8` as the static planning threshold.

### 4. Why crouching helps reach but can hurt torque

Lowering the pelvis restores knee bend and can permit more lateral travel. However, a bent knee may require a larger holding torque. Pressure shifts likewise change ankle and hip moments. The correct question is whether **all** joints can meet their torque budget together, not whether one joint becomes easier to control.

## Files added

- `src/my_robot_description/my_robot_description/support_feasibility.py`: fixed-sole geometry, linear torque model and pressure optimization. It is separate from the live controller.
- `scripts/search_support_postures.py`: performs the finite grid search and saves every solved candidate.
- `scripts/plot_support_search.py`: renders saved data without recomputing the search.
- `tests/test_support_feasibility.py`: compares sole-center torque predictions with the existing model, independently checks shifted-contact r×F torques and force/moment balance, checks zero-load moments, mirrors support sides and rejects invalid inputs.
- This guide and the README link explain the new result. No URDF, gains, damping plugin or live force guard was changed.

## Reproduce

From the workspace:

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/search_support_postures.py --output evidence/my-support-search-01
python3 scripts/plot_support_search.py evidence/my-support-search-01
```

Use a fresh output directory. The offline analysis uses the installed NumPy, SciPy and Matplotlib libraries; no simulation or workspace rebuild is needed. `search.json` contains the best candidates and complete solved grid. `comparison.png` shows the load/torque trade-off and a schematic 75/25 candidate; `feasibility.png` shows the near-zero-load grid.

## What to do next

A stronger balance controller can help track a feasible posture but cannot supply torque above the motor budget. Before changing motor caps, extend the posture analysis to **pelvis/torso roll and asymmetric leg configurations**, because this search kept the torso upright and leg angles symmetric. If that yields a near-single-support posture with reserve, verify a full transition, add controlled pressure/load feedback, and then test gradual unloading in Gazebo.

If no suitable posture exists, use the resulting joint torque requirements to justify a specific change to hip spacing, mass distribution or actuator capacity. We have not established the motor ratings needed by every possible walking gait, so the numbers above should not be used as final hardware-sizing specifications.

Validation: all 262 unit tests passed. The saved grid also passed force balance, lateral world-moment balance and pressure-bound checks. Physical-model, gain and motor-cap configuration hashes match the previous measured partial-unloading milestone. See `verification.json` and `unit-tests.txt` in `evidence/support-posture-search-02/`.

Follow-up: [torso lean and asymmetric legs](LEAN_SUPPORT_SEARCH.md) extends this calculation without changing live limits.
