# Next milestone: 30 mm lowered bilateral transfer

Prepared for your tests; no 30 mm simulation has been run.

## Evidence from your 20 mm runs

| Direction | Job | Measured pelvis shift | Measured COM shift | Loaded-foot share |
|---|---|---:|---:|---:|
| Left | 20261104-125635-3637b8a5 | +20.008 mm | +12.904 mm | approximately 57.59% |
| Right | 20261104-195944-b6e4ac66 | -20.008 mm | -12.904 mm | approximately 57.59% |

Both passed the motion, sensor, measured-load, motor and return audits. Peak
motor command utilization was 20.39%, with zero saturated samples. Sources
remained unchanged during recording. The right run's COM display raised a
shutdown error after motion completion; its callback/context cleanup ordering
has been corrected and covered by offline lifecycle tests. Live shutdown
verification awaits your next runs.

## Run in the panel

1. Refresh http://127.0.0.1:8765/.
2. Choose **40 mm lowered transfer: 30 mm left** in the roadmap.
3. Confirm distance **30 mm**, direction **Left**, lowering **40 mm**.
4. Click **Run lowered transfer**, then wait for completion and the plot.
5. Repeat **40 mm lowered transfer: 30 mm right**, confirming **Right**.
6. Tell me when both finish so I can review their measurements and logs.

Allow roughly **15–16 real minutes per run** at the 0.25 real-time-factor target;
actual duration depends on computer load.

## What changes, and why

Pelvis travel increases from 20 to 30 mm. Each sideways movement increases
from 32 to 48 simulated seconds, preserving the slow reference speed scale.
The total program takes 201 simulated seconds after standing readiness:
baseline 0–3, lower 3–43, settle 43–47, shift 47–95, hold 95–101,
return 101–149, settle 149–153, rise 153–193, recover 193–201.

The static model predicts COM travel **19.348 mm**, a **61.38/38.62%** load
split and peak feedforward utilization **22.00%**. These are predictions,
not measured results. COM travels less than the pelvis because the feet and
parts of the legs remain near their original positions.

This brings us closer to unloading one foot. Both feet must still carry load;
this stage does not authorize a foot lift. Existing gains, geometry, motor caps,
contact guards and acceptance tolerances remain unchanged.

## Files changed

- `lowered_shift.py`: accepts ±30 mm using existing IK, speed scaling and load guards. Cache fractions are calculated before multiplying by displacement so the final pose retains the exact requested endpoint.
- `tools/force_panel/config.py`: validates and maps 30 mm GUI requests.
- `tools/force_panel/jobs.py`: separate left/right 30 mm roadmap verification.
- `tools/force_panel/static/index.html`: 30 mm distance option.
- `scripts/run_lateral_shift.py`: documents the new CLI value.
- `tests/test_lowered_shift.py`: adds both signs to geometry, speed, torque and audit checks.
- `tests/test_force_panel.py`: checks signed commands and independent milestone verification.
- `src/my_robot_description/scripts/show_center_of_mass.py`: keeps ROS context valid until callbacks finish, then stops timer/executor, closes observer and destroys node.
- `tests/test_com_observer_lifecycle.py`: checks cleanup ordering and propagation of real callback errors.

Preparation calculations and the previous-run review are saved under
`evidence/preparation/lowered-shift-30mm/`.
