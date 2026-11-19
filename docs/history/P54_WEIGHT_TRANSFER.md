# Resuming motion after the actuator upgrade

The P54-class model weighs 22.28 kg, compared with the legacy 14.42 kg model. A passing standing test does not automatically verify motion with the new mass distribution. This milestone repeats the established 65/35 transfer, then tests a 70/30 transfer in both directions, keeping both soles on the ground.

## What 70/30 means

Weight is `W = 22.28 × 9.81 = 218.5668 N`.

| State | Loaded foot | Lighter foot |
|---|---:|---:|
| Centered 50/50 | 109.2834 N | 109.2834 N |
| 65/35 target | 142.06842 N | 76.49838 N |
| 70/30 target | 152.99676 N | 65.57004 N |

For the current sole-center planning model, with feet centered at ±85 mm, moment balance gives `F_R/W = (1 − COM_y/0.085)/2`. A right-foot share of 30% therefore needs `COM_y = 0.085 × (1 − 2×0.30) = 0.034 m` toward the left foot. The planner uses the actual link masses to find the corresponding pelvis displacement; it does not equate pelvis motion with COM motion.

This experiment uses the existing upright, fixed-foot lateral program. It does not command the offline crouched/leaning poses or regulate pressure location with a new feedback controller.

## First finding: the COM audit still assumed the old mass

The initial new-profile 65/35 trial reached the measured load split and returned within the motion limits, but its overall `summary.json` reported failure. The reason was an audit defect in `my_robot_description/lateral_audit.py`:

```python
# Before: only correct for the legacy model
com_audit['passed'] &= abs(com_audit.get('mass_kg', 0) - 14.42) < 1e-9
```

The independent observer reported 22.28 kg, so the constant rejected a correct measurement. The fix uses a separate URDF-derived expectation:

```python
com_audit['coverage_passed'] = com_audit['passed']
com_audit['expected_mass_kg'] = program.planner.base['mass_kg']
com_audit['mass_matches_model'] = abs(
    com_audit.get('mass_kg', 0) - com_audit['expected_mass_kg']
) < 1e-9
com_audit['passed'] &= com_audit['mass_matches_model']
```

The expected mass comes from model planning, not from the observed value. The tolerance and all motion thresholds remain unchanged. Coverage and mass agreement are now reported separately so the failure is easier to diagnose.

The original failed `evidence/p54-unload-35-left-01/summary.json` is preserved. `reaudit.json` records the corrected checks on the same raw data, with hashes of the original summary, verified model/control sources and current auditor. It passes all checks. The observed lighter-foot share was 34.9931%.

To reproduce that audit in another compatible recorded folder:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/reaudit_lateral_shift.py evidence/YOUR_RECORDED_FOLDER
```

The script rejects incompatible physical-model/control sources and preserves the original summary. It creates `reaudit.json` exclusively, so an existing re-audit is not silently overwritten.

## The next target is deliberately limited

`partial_unload.py` now permits a 30% lighter-foot target only with `robotis_p54`. Legacy 40%/35% experiments retain their original limits. The CLI, launch argument and offline target-preview tool accept the new target consistently.

The measured-force guard still requires:

- At least 25% of total vertical support on each foot.
- Fresh bilateral force data with source age between −0.02 and +0.05 simulated seconds.
- Total vertical reaction within 5% of model weight.
- Requested motor torque no greater than 80% of each cap during unloading.

The 30% target leaves a five-percentage-point support margin above the guard. Those guards do not prove dynamic stability; posture, foot geometry, sensor coverage, return accuracy and physical damping are checked independently.

Joint gains, damping, motor ratings, actuator masses and motion timing are unchanged. After readiness, the program is baseline 3 s, outward motion 8 s, hold 8 s, return 8 s and recovery 10 s. The 37 simulated seconds take about 148 wall seconds at real-time factor 0.25, plus startup/settling.

## Code changes

| File | Purpose |
|---|---|
| `src/my_robot_description/my_robot_description/lateral_audit.py` | Compare observed COM mass with the model's mass; report coverage and mass agreement separately |
| `src/my_robot_description/my_robot_description/partial_unload.py` | Allow a 30% target only for P54; retain the measured 25% support floor and motor reserve |
| `src/my_robot_description/launch/roll_standing.launch.py` | Accept the corresponding launch argument |
| `scripts/run_lateral_shift.py` | Accept the 30% experiment target |
| `scripts/plan_partial_unload.py` | Accept the same target for offline preview |
| `scripts/reaudit_lateral_shift.py` | Recheck preserved raw evidence without rewriting its original result |
| `tests/test_p54_transfer.py` | Fresh-process tests for the new model mass, rejection of an incorrect measured mass, mirrored target geometry and legacy-profile rejection |

## Reproduce the live experiment

From the workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash

python3 scripts/run_lateral_shift.py --gui \
  --actuator-profile robotis_p54 --unload-fraction 0.3 --direction left \
  --output evidence/my-p54-70-30-left-01

python3 scripts/analyze_foot_loads.py evidence/my-p54-70-30-left-01
```

Use a fresh output folder. `--direction right` mirrors the transfer. The output contains raw contacts, independent IMU/model pose/COM data, joint commands, source hashes and separate force/motion/damping audits.

A passing bilateral transfer is still distinct from zero-load sensing, foot lifting or single-support balance. Measured forces currently act as safety guards; they do not continuously correct the target load split.

## Verified results

The build and **273 unit tests passed**. The 65/35 baseline passed the corrected raw-data audit; its original failed report remains intact. Fresh 70/30 runs passed every motion, sensor, COM, foot-force, guard and physical-damping check in both directions.

| Trial | Measured lighter-foot share | Measured COM shift | Peak motor utilization | Peak final COM offset |
|---|---:|---:|---:|---:|
| `p54-unload-30-left-01` | 29.9904% | 34.0166 mm | 13.51% | 0.2909 mm |
| `p54-unload-30-right-01` | 29.9902% | -34.0167 mm | 13.51% | 0.2894 mm |

Both runs recorded zero motor saturation. Both feet remained grounded. Model/actuator/damping hashes match the new-profile standing baseline, and recorded lateral gains match the 65/35 run. See `foot-loads.png` in each trial folder and `verification.json`, `build.txt` and `unit-tests.txt` in the mirrored trial folder.

The next control milestone is a small bounded correction driven by measured foot-load error, verified in double support before reducing the support floor or attempting a lift. These results do not establish single-leg balance.
