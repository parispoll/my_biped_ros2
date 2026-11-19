# Torso lean and asymmetric legs: what changes?

We extended the offline support search to rotate the pelvis/torso about world X and solve each leg independently. This improves the static result, but the best sampled zero-load pose still exceeds the current motor caps. No new Gazebo motion was commanded.

## 1. The previous restriction

Previously the torso stayed upright and both leg chains used matching angles. Moving the pelvis sideways and down could not reproduce the asymmetric leg posture that occurs when someone leans toward a supporting leg.

The new model treats the torso and pelvis as the existing single rigid `base_link`. There is no additional waist joint. Its roll angle changes the orientation of both hip frames; the left and right legs then require different joint angles to keep both soles fixed and flat.

## 2. Inverse kinematics in a leaning body frame

Let the body rotation be `R = Rx(phi)`, body position be `p`, and the world-fixed ankle target be `a_i`. The ankle target relative to its hip, expressed in the body frame, is

`v_i = Rᵀ (a_i − p) − hip_origin_i`.

For each leg separately:

- Hip roll is `atan2(v_y, −v_z)`.
- The sagittal IK uses `x = v_x`, `z = −sqrt(v_y² + v_z²)`.
- Hip, knee and ankle pitch sum to zero for this flat-foot family.
- Ankle roll is `−phi − hip_roll`, cancelling body and hip roll.

Every joint limit is checked. Forward kinematics independently verifies the position and orientation of both world-fixed soles.

Gravity must remain vertical in the **world frame**. We rotate link frames, joint axes and inertial COM locations into world-aligned coordinates before calculating gravity/contact moments. Rotating gravity along with the body would produce an incorrect answer.

## 3. Search domain

The coarse search uses:

| Variable | Range | Spacing |
|---|---|---|
| Lateral pelvis shift | 0–120 mm left | 10 mm |
| Pelvis lowering | 0–60 mm | 20 mm |
| Pelvis/torso roll | −0.35 to +0.35 rad (about ±20°) | 0.05 rad |
| Right-foot load fraction | 25%, 10%, 5%, 0% | Separate optimizations |

For each requested fraction, the best coarse point is refined locally in shift, height and roll. This is a finite search with a local refinement, not a proof of the global optimum. Some refined best points lie at the search boundary, especially the −0.35 rad lean and 60 mm lowering for near-zero load.

The same pressure optimizer reserves 10 mm from every sole edge and minimizes peak static motor utilization. It assumes vertical ground reactions, zero yaw contact moment and flat feet. No mass, joint, motor cap, damping or live guard was changed.

## 4. Static results

| Lighter-foot fraction | Previous upright search | Lean + independent legs | Within motor caps? |
|---|---:|---:|---|
| 25% | 53.2% peak utilization | 48.8% | Yes, statically |
| 10% | 151.6% | 133.3% | No |
| 5% | 183.3% | 163.5% | No |
| 0% | 217.3% | 196.8% | No |

The best sampled zero-load pose has 45 mm lateral shift, 60 mm lowering and −0.35 rad roll. It requires approximately:

| Supporting-leg joint | Requested static torque magnitude | Current cap |
|---|---:|---:|
| Hip roll | 11.81 Nm | 6 Nm |
| Hip pitch | 1.40 Nm | 6 Nm |
| Knee pitch | 16.67 Nm | 10 Nm |
| Ankle pitch | 1.72 Nm | 6 Nm |
| Ankle roll | 6.98 Nm | 6 Nm |

This pose is a diagnostic, not a command candidate. It also exceeds the live tilt guard. The result does not establish that all possible postures or dynamic gaits are infeasible.

## 5. A less demanding partial-transfer candidate

The absolute best sampled 75/25 pose leans by 0.15 rad, essentially reaching the existing 0.15 rad live tilt guard. For planning a transfer, we therefore select a gentler candidate with roll limited to 0.10 rad. This changes only the offline selection, not a live guard.

| Quantity | Selected 75/25 candidate |
|---|---:|
| Pelvis shift left | 70 mm |
| Pelvis lowering | 20 mm |
| Pelvis roll | −0.10 rad (5.73° toward the supporting side) |
| Left vertical load | 106.09515 N |
| Right vertical load | 35.36505 N |
| Peak static motor utilization | 50.18% |
| Left knee angle | 0.72549 rad |
| Right knee angle | 0.51164 rad |

The unequal knee angles demonstrate the asymmetric posture.

We checked 101 static points along an interpolation from centered standing toward this pose. The right load decreases from 50% to 25%. Pressure begins at the sole centers, and each foot's fore/aft and lateral pressure offset may change by at most 2 mm between adjacent points. All 101 points satisfy static equilibrium with peak motor utilization at most 50.18%.

This is not yet a smooth timed walking trajectory. The pressure optimizer can change direction between samples; its plotted pressure targets have small zigzags. Bounded adjacent changes do not establish bounded acceleration, dynamic stability, actuator tracking or actual pressure regulation. Collision clearance has not been certified. Both feet remain geometrically on the ground even when an endpoint calculation assigns one zero force.

The selected endpoint is also exactly at the current 25% live measured-share floor. A real trial needs margin above that floor rather than commanding its boundary. A 70/30 intermediate point could be an earlier test after implementing and verifying the required feedback.

## 6. Why leaning helps only so much in this model

The robot's torso is one 7.5 kg rigid link whose inertial COM is only 20 mm above its frame. At 0.35 rad lean, rotating that COM moves it laterally by only `0.02 sin(0.35) ≈ 6.9 mm` relative to the base frame origin. Leaning also changes hip locations and leg geometry, but it does not provide the large movable torso mass distribution one might imagine from a human sketch.

Lowering creates reach while increasing knee bend. Consequently a pose that reduces hip-roll demand may increase knee demand. Correct posture control can redistribute torque; it cannot make an infeasible static torque budget disappear.

## 7. Files changed

| File | Change |
|---|---|
| `src/my_robot_description/my_robot_description/support_feasibility.py` | Added independent-leg leaning IK; shared world-frame gravity/contact assembly; optional bounded pressure-step constraints. Optimizer failures other than ordinary infeasibility are now reported explicitly. |
| `scripts/search_lean_support.py` | Added coarse/local static searches and the pressure-step-limited partial-transfer check. |
| `scripts/plot_lean_support.py` | Added actual-frame front projection, upright/lean comparison, per-joint demands and pressure-path plots. |
| `tests/test_lean_support.py` | Checks zero-roll regression, asymmetric knees, fixed/flat soles, direct world-frame torque sums, moment equilibrium, mirroring, input rejection and bounded pressure transitions. |

Existing upright analysis still uses zero body roll by default. The live controller does not import this optimizer.

## 8. Reproduce and inspect

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/search_lean_support.py --output evidence/my-lean-search-01
python3 scripts/plot_lean_support.py evidence/my-lean-search-01
```

Use a fresh output folder. The plot compares against the saved upright results in `evidence/support-posture-search-02`; `--upright OTHER_FOLDER` selects another compatible result folder.

Verified results are in `evidence/lean-support-search-04/`: `search.json`, `comparison.png`, `unit-tests.txt`, and `verification.json`. The search saved 2,130 solved static candidates, plus 101 transfer points. Earlier search folders preserve the coarse/refinement work, including the initial unsmoothed pressure solutions.

## 9. Development decision

This investigation does not justify attempting a foot lift under the current static assumptions. The next useful decision is a joint-torque and mass-layout review: compare a physically consistent change to hip spacing or body mass distribution against actuator requirements. Retain the measured standing and 65/35 baselines for comparison. Any new model should first repeat those established checks.

A dynamic gait can have different requirements from a static one-leg hold. We have not ruled out dynamic walking, but it would require a dynamic analysis and controller; the static motor numbers above are not complete hardware-sizing specifications.

Validation: all 269 unit tests passed. The saved 2,130 candidates and 101 transfer samples passed world-frame force/moment, joint-limit, sole-pressure and torque-utilization checks. The largest audited moment residual was below 2e-14 Nm. Hash checks confirm the physical model, actuator configuration, gains and live posture adapter remain unchanged from the measured partial-unloading baseline.
