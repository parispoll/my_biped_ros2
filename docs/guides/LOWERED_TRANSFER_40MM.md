# 40 mm lowered bilateral transfer

This stage increases lateral pelvis travel to ±40 mm after lowering by 40 mm.
It uses the same robot, gains, motor caps, fixed-foot IK and acceptance limits.
The preceding measured milestone is 20 mm; 30 mm was prepared but has no
saved live runs. The larger increment is explicitly requested and is checked
offline before execution. Passing 40 mm does not mark an unrun 30 mm test verified.

## Why the movement still takes time

Sideways duration scales with distance: 8 simulated seconds per 5 mm.
Thus 40 mm takes 64 simulated seconds each way. The complete motion takes
233 simulated seconds after readiness, approximately 17–18 wall minutes
at the existing 0.25 real-time-factor setting. This isolates greater unloading
from faster acceleration. A future speed experiment should change timing at
a distance already verified by measured forces.

The schedule is: baseline 0–3 s, lower 3–43, settle 43–47, shift 47–111,
hold 111–117, return 117–181, settle 181–185, rise 185–225, recovery 225–233.
Hold measurements use 115–117 s; final return uses 230–233 s.

## Calculations and checks

The planner uses the actual mass-weighted URDF COM and solves pitch joints
while roll joints translate the pelvis with both feet flat and fixed.
Static foot reactions satisfy F_left + F_right = mg and
F_left/(mg) = 0.5 + COM_y / 0.17 m. These reactions are feedforward
predictions; the experiment independently records Gazebo contact wrenches.

The static predictions are ±25.795 mm COM travel, a 65.174/34.826% load
split, and 23.532% peak motor utilization. These are not measurements.
The preparation JSON contains both mirrored endpoint predictions and peak
static motor utilization. The planner rejects a pose below 30% predicted
lighter-foot share or above 80% static motor capacity. Runtime guards and
measurement audits remain unchanged. This stage is bilateral support, not
single-leg balance or foot lift.

## Files changed

- `src/my_robot_description/my_robot_description/motion/lowered_shift.py`: allows ±0.04 m. Existing formulas generate 321 cached lateral poses and 64 s movement segments.
- `tools/force_panel/config.py`: allows 40 mm GUI requests and maps direction to signed metres.
- `tools/force_panel/jobs.py`: separate 40 mm left/right roadmap milestones with their own measured verification.
- `tools/force_panel/static/index.html`: adds 40 mm distance choice.
- `scripts/experiments/run_lateral_shift.py`: documents ±0.04 m support.
- `tests/test_lowered_shift.py`: mirrored 40 mm geometry, COM, motor, speed, return and independent audit checks.
- `tests/test_force_panel.py`: signed 40 mm requests and separation of verification from previous distances.

## Reproduce through the GUI

Refresh http://127.0.0.1:8765/, choose **40 mm lowered transfer: 40 mm left**
or **40 mm lowered transfer: 40 mm right**, confirm lowering 40 mm and
transfer distance 40 mm, then Run lowered transfer. Each run saves its own
summary, plot, raw recordings, logs and source hashes under `evidence/gui/`.

Preparation artifacts: `evidence/preparation/lowered-shift-40mm/`.
Both directions were run and passed on 5 November 2026. See the measured results below.

The left run passed all measured audits but revealed a controller shutdown
race after recording: ROS invalidated the publisher context while cleanup
was sending its final zero command. Before the right run,
`src/my_robot_description/nodes/control/hold_roll_stance.py` was changed to own
SIGINT/SIGTERM handling, finish callbacks, cancel the supervision timer, stop
the executor, send zero/close recording, destroy the node, then shut down ROS.
`tests/test_roll_controller_lifecycle.py` checks this order and that genuine
callback failures remain visible. Control laws and trajectories are unchanged.


## Measured results — 5 November 2026

| Direction | GUI job | Pelvis travel (mm) | COM travel (mm) | Loaded-foot share | Peak motor utilization |
|---|---|---:|---:|---:|---:|
| left | 20261105-055958-82894a69 | +40.0037 | +25.7974 | 65.1740% | 23.5322% |
| right | 20261105-061847-6dd93bb5 | -40.0037 | -25.7974 | 65.1740% | 23.5322% |

Both passed every motion, sensor, load, motor and damping audit, with zero
saturated samples and unchanged package sources within each recording.
Both returned to approximately 50/50 support. The measured review JSON
includes raw-wrench-derived baseline, hold and recovery forces.

The first left run had a controller cleanup exception after completed motion;
its evidence is preserved. The right run verified clean controller and COM
observer exits after the cleanup fix. Source hashes show the controller script
was the only package source changed between the runs.

The remaining foot still carried about 34.83% of the weight: this is measured
bilateral transfer, not single support or walking. A suitable next study is
a shorter transfer time at this verified distance, with timing exposed and
recorded explicitly and dynamic acceptance criteria considered separately.
No faster-motion experiment has been prepared or run in this stage.
