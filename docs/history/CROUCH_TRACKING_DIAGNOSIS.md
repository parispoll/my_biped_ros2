# Crouch tracking investigation

## What the recordings show

Five fresh headless launches were run with the original zero-angle initialization.
Four passed; one failed. All used the original 0.08 rad path tolerance, 0.02 rad
goal tolerance, eight-second transition and 0.4 rad target knee bend.

In failed run 02, the left knee reference reached 0.08003167 rad while its measured
position stayed approximately zero (-4.6e-14 rad). The right knee behaved the same.
The hips and ankles followed their references. Controller output positions advanced
normally, as did simulation timestamps. The largest received controller timestamp
gap in that failed run was 0.008 s. Thus this trace does not resemble a general
clock jump or all-joint controller outage.

This is strong evidence of knee lock-up at the lower joint stop, not an incorrect
crouch angle or ordinary tracking lag. The knees were initialized at 0 rad, exactly
their declared lower limit. Gazebo's position interface internally generates
velocity commands. Similar joint-limit lock-up with velocity commands is documented
upstream. These observations identify a practical trigger; they do not establish
which low-level physics-engine branch caused our individual failed run.

![Desired and measured knee motion before and after startup clearance](../evidence/crouch-tracking-comparison.png)

The left plots show the failing trace; the right plots show a representative
successful clearance run. The bottom panels show desired-minus-measured errors
against the same red 0.08 rad tolerance. Both knee traces overlap closely.
The failing trace ends at its first tolerance violation, before the controller's
subsequent hold-position behaviour.

## Small startup correction

For `stance:=crouch`, initialize both knees at 0.02 rad (about 1.15 degrees), inside
their existing range. Compute matching hip and ankle angles using the same COM and
flat-foot equations used for the crouch. Measured initial angles in the first test:

```
hip   = -0.01118816 rad
knee  = +0.02000000 rad
ankle = -0.00881184 rad
```

Their sum is zero and the calculated COM remains over the foot centres. The robot
then follows the existing eight-second trajectory to the original 0.4 rad target.

The knee limit remains [0, 2.4] rad. Tracking tolerances, masses, geometry, gains,
IMU configuration, and target crouch angles are unchanged. Straight launch mode
still initializes at zero; it has not been converted to the new near-straight pose.
Moving away from that exact straight hard-stop configuration can still encounter
this issue unless a clearance initialization is used.

## Code changes

Paths below are relative to `src/my_robot_description` unless noted otherwise.

| File | Change |
|---|---|
| `my_robot_description/tracking.py` (new) | Records timestamped reference, actual, error, output and velocities; writes a failure/success summary |
| `scripts/move_to_stance.py` | Subscribes to controller state before sending the action; saves diagnostics on success or failure and logs measured initial joints |
| `launch/standing.launch.py` | Adds optional `tracking_log` and crouch-only `initial_knee_bend` arguments |
| `my_robot_description/startup.py` | Passes the diagnostic filename and selected initialization into the appropriate helpers |
| `my_robot_description/description.py` | Calculates balanced initial angles and supplies them as Xacro arguments |
| `urdf/my_biped.urdf.xacro` | Declares initial hip/knee/ankle arguments with zero defaults |
| `urdf/components/control.xacro` | Uses a reusable joint-interface macro and supplies each position state interface's `initial_value` |
| Workspace `scripts/repeat_crouch.py` (new) | Runs fresh, isolated simulations sequentially and preserves logs/results |
| Workspace `scripts/plot_tracking.py` (new) | Produces the comparison figure from raw recordings |
| Workspace `tests/test_initial_stance.py`, `tests/test_tracking.py` (new) | Verify clearance, unchanged limits/zero baseline, and preservation of failure traces |

The initial-value parameters are consumed by the Gazebo hardware interface to set
simulated joint positions. They do not change URDF joint origins or redefine what
zero means. The default standalone Xacro expansion still has zero initial angles.

## Use it

After building and sourcing this workspace:

```bash
ros2 launch my_robot_description standing.launch.py \
  gui:=true stance:=crouch tracking_log:=/tmp/biped-tracking.jsonl
```

The result summary is `/tmp/biped-tracking.summary.json`. Each JSONL row contains
all six joints in its `joint_names` order. Recorder output is optional; without a
filename, the client still prints a worst-error summary.

To reproduce the old hard-stop experiment explicitly:

```bash
ros2 launch my_robot_description standing.launch.py \
  stance:=crouch initial_knee_bend:=0 tracking_log:=/tmp/biped-zero-start.jsonl
```

To run fresh trials, use a new output directory each time:

```bash
python3 scripts/repeat_crouch.py --runs 5 --output evidence/my-new-crouch-trials
```

The runner uses separate ROS domains and Gazebo partitions for its own launches;
it does not stop an existing simulation. It stops each simulation it creates and
uses wall-clock deadlines. A failed diagnostic run is retained in the summary,
rather than automatically retried or hidden.

## Reading the diagnostics correctly

The path tolerance applies during motion; the goal tolerance applies to reaching
the final target. Desired/output positions reaching a joint interface do not prove
the simulated joint moved. Compare them with measured positions and velocities.

Received timestamp gaps can also reflect dropped/delayed diagnostic messages;
they are not by themselves proof that the physics/control loop stopped.
Repeated successful tests reduce concern about this startup trigger but cannot
prove that no other failure will ever occur.

Sources:
- Gazebo issue discussing joint-limit/velocity lock-up: https://github.com/gazebosim/gz-sim/issues/1684
- Position interface behaviour: https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html
- Initial-position handling in the installed plugin's release source: https://github.com/ros-controls/gz_ros2_control/blob/1.2.17/gz_ros2_control/src/gz_system.cpp

## Repeated-run results, 2026-10-26

| Configuration | Fresh launches | Successful | Failed |
|---|---:|---:|---:|
| Original zero-knee initialization | 5 | 4 | 1 |
| Balanced 0.02 rad knee initialization | 10 | 10 | 0 |

Maximum recorded joint error across the ten clearance trials was
0.003244 rad (about 0.186 degrees), with
the same 0.08 rad path tolerance. The raw JSONL recordings contain desired and
measured positions for all six joints. All 24 automated checks passed.

Evidence folders: `evidence/crouch-baseline`, `evidence/crouch-initial-clearance`,
and `evidence/crouch-clearance-confirmation`. Aggregated results are in
`evidence/crouch-investigation-summary.json`.

Final regression after the repeated runs also passed: the crouch completed, the body
stayed upright for 30.004 simulated seconds at approximately 0.5544873 m, and the
IMU delivered approximately 100 Hz with stationary readings. These results are in
`evidence/crouch-final-launch.log`, `evidence/crouch-final-standing-check.json`, and
`evidence/crouch-final-imu-check.json`. The test simulator was shut down afterward.
