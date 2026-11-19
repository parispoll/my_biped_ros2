# Your next tests: 10 mm transfer in the 40 mm lowered posture

Update: the user-run 10 mm left/right tests have passed. The current prepared
next milestone is documented in
[NEXT_LOWERED_TRANSFER_20MM.md](NEXT_LOWERED_TRANSFER_20MM.md). The original
10 mm preparation instructions below are retained for reproducibility.

Status: **prepared for user testing, not yet validated by a live experiment**.
No experiment was launched during this preparation. Offline calculations and
regression tests are separate from measured Gazebo evidence.

## Why this step

The 5 mm left and right transfers have measured passing results. The next
increment increases pelvis travel to 10 mm at the same 40 mm lowered height.
Both feet remain planted and positively loaded. This gives a larger measurable
load transfer before moving toward deeper unloading or foot lift.

The robot uses the existing URDF, actuator limits, PID gains, physical damping,
force guards and audit thresholds. The new value is an allowed planning target,
not a change in motor strength.

## How to run it from the GUI

1. Refresh <http://127.0.0.1:8765/> after this preparation.
2. In the roadmap, choose **40 mm lowered transfer: 10 mm left**.
3. Confirm **Lowered transfer distance = 10 mm**, direction **Left**, and
   **Show the Gazebo simulation** checked.
4. Click **Run lowered transfer**. Allow roughly **10-11 wall minutes**.
5. Let the motion, recovery, audit and plotting finish. Note the job ID and
   whether the result says Passed, Checks not met or Error.
6. Repeat using **40 mm lowered transfer: 10 mm right** as a fresh GUI job.
7. Share the two job IDs when finished so the saved measurements and logs can
   be reviewed. Results are saved under `evidence/gui/<job-id>/`.

The older 5 mm choices remain available as reference experiments. Each distance
and direction has its own verification status; old 5 mm evidence cannot verify
the new 10 mm entries. The new entries remain **ready to test** until their
own measured run passes.

## Expected sequence

| Phase | Program time (simulated seconds) |
|---|---:|
| Baseline | 0-3 |
| Lower 40 mm | 3-43 |
| Settle while lowered | 43-47 |
| Move sideways 10 mm | 47-63 |
| Hold | 63-69 |
| Return to center | 69-85 |
| Settle at center | 85-89 |
| Rise | 89-129 |
| Recovery | 129-137 |

Full standing readiness comes before this 137 s program. The simulator targets
0.25 times real-time speed. The side motion now takes 16 s each way instead
of 8 s: twice the distance, twice the time, preserving slow motion.

## Predictions to compare with your measurements

| Quantity | Leftward test | Rightward test |
|---|---:|---:|
| Requested pelvis shift | +10 mm | -10 mm |
| Predicted COM shift | +6.450 mm | -6.450 mm |
| Predicted held left load share | 53.794% | 46.206% |
| Predicted held right load share | 46.206% | 53.794% |

These are static model predictions, not measured outcomes. The largest static
feedforward demand across the planned poses is about **18.72%** of its motor
cap; actual motor commands also contain feedback corrections.

The same acceptance checks apply: fresh contact at both feet, load-share
floor, measured load redistribution, force/COM consistency, tracking, motor
headroom, joint speed, sensor coverage and final return. The result plots show
planned and measured height/lateral position, measured foot shares and speed.

## What changed in the code

| File | Change |
|---|---|
| `src/my_robot_description/my_robot_description/lowered_shift.py` | Allows ±5 or ±10 mm, scales lateral duration with distance, keeps cached pose spacing at 0.125 mm, derives analysis windows from the schedule |
| `tools/force_panel/config.py` | Validates a new `shift_mm` setting (5 or 10); constructs a signed metre value for the runner |
| `tools/force_panel/jobs.py` | Tracks measured verification separately for distance and direction; treats historical requests without `shift_mm` as 5 mm |
| `tools/force_panel/static/index.html`, `app.js` | Adds a distance choice and updates preview, timing, saved settings and history |
| `scripts/run_lateral_shift.py` | Documents both supported distances in CLI help |
| `tests/test_lowered_shift.py` | Checks both signs at both distances, foot fixation, COM, limits, speed, return and force-audit rejection |
| `tests/test_force_panel.py` | Checks distance-to-command conversion, input rejection and that old runs cannot verify a new distance |

At 10 mm the lateral table has **81 poses**, compared with 41 for 5 mm. The
analysis hold window is 67-69 s and the final return window 134-137 s. These
follow the schedule, rather than reusing the shorter experiment's windows.

Offline predictions and preparation logs are in
`evidence/preparation/lowered-shift-10mm/`. These are preparation artifacts,
not GUI experiment recordings.

Preparation checks: the build, JavaScript syntax and **293 unit tests passed**.
Both directions at both 5 and 10 mm were checked offline for planted/flat feet,
COM consistency, speed, static motor headroom, return and force-audit rejection.
The GUI controls were inspected and left on **10 mm left**, with no Run click.
The before/after GUI job lists were identical and the panel was idle at the
final check. The new roadmap entries remained **ready to test**.

Saved-result text now names the earlier experiment explicitly, so a previous
Passed result cannot be mistaken for validation of the newly selected motion.
