# Why effort startup failed, and what changed

The reported run was `evidence/gui/20261028-082423-3e578d97`. Its reference
simulation failed before any push experiment began. The runner's readiness
exception was a consequence of the posture process exiting, not the original
cause. The original exception in `reference/launch.log` was:

```text
ValueError: Joint feedback timing invalid: source_age=-0.572000 s, source_gap=0.004000 s
```

## Reconstructing the failure

The controller computes joint-feedback age as:

```python
age = local_ros_time - joint_message_timestamp
```

Here, the joint timestamp was 0.572 simulated seconds and the local clock was
still zero: `0.000 - 0.572 = -0.572 s`. The joint stream had reached the node
before its `/clock` subscription had initialized local simulation time. ROS
time returns zero before initialization; see the
[ROS 2 time design](https://design.ros2.org/articles/clock_and_time.html).
The joint-message interval was a normal 0.004 s; this was not an oversized
integration step or a force large enough to knock down the robot.

Previously, `supervise()` checked that joint terms, an IMU and a command
subscriber existed, then proceeded toward requesting controller activation:

```python
if self.last_terms is None or self.imu_wall is None or self.pub.get_subscription_count() == 0:
    return
if self.parameter_future is None:
    # Check manager limits, then request effort activation.
```

Those conditions did not prove that the local ROS clock was initialized.
After activation, `feedback()` called `integration_timing(...)`, which correctly
rejected the apparent future timestamp. Launch then closed the simulation, and
the runner reported that standing never became ready. This explains why an
otherwise identical run could sometimes pass: discovery order could differ.

## The fix, file by file

### `src/my_robot_description/my_robot_description/feedback_timing.py`

Added the pure function `startup_timing_ready(stamp, receipt)`:

```python
if not all(math.isfinite(x) and x > 0 for x in (stamp, receipt)):
    return False
try:
    integration_timing(stamp, stamp, receipt)
except ValueError:
    return False
return True
```

First reject zero/uninitialized and nonfinite times. Then reuse the existing
feedback-age check rather than defining more permissive startup tolerances.
Using `stamp` for both timestamp arguments makes the interval zero: this is
only an age/readiness check, not a controller integration update.

### `src/my_robot_description/scripts/hold_effort_stance.py`

Before startup requests, `supervise()` now checks:

```python
if not self.active:
    ros_time = self.get_clock().now().nanoseconds / 1e9
    if not startup_timing_ready(self.last_stamp, ros_time):
        # Log clock-wait once.
        return
    # Log clock-ready once, then continue existing startup.
```

Returning keeps the node processing callbacks while waiting. It does not sleep
inside the callback, start the integral, or claim the controller is active.
The existing 30 wall-second startup deadline still stops a run whose clock
never arrives. Once active, the original timing exception still stops invalid
feedback. Active control math, gains, torque caps, balance law and robot
physics are unchanged. Feedback queue depth is updated separately below. Two flags prevent repetitive startup log messages.

### `src/my_robot_description/my_robot_description/experiment_failure.py`

New small diagnostic helper `launch_failure(path)` extracts the last Python
exception from the child launch log, removing terminal color codes. If there is
no Python exception, it directs inspection toward process exit/readiness timeout.
It does not change experiment results, retry failures or suppress exceptions.

### `scripts/run_effort_push.py`

The readiness error now includes that diagnostic in addition to the full log
path. Before: only `Effort standing did not become ready; inspect ...`.
After: `Effort standing did not become ready: ValueError: ...; inspect ...`.
The existing panel already displays runner output, so this improves its error
message without changing the running browser server.

### Tests

`tests/test_feedback_timing.py` adds the exact .572/zero-clock regression,
nonfinite/stale/future startup cases, and checks that the actual `supervise()`
method cannot request activation before synchronization. It also checks that
startup still times out if the clock never initializes.
`tests/test_experiment_failure.py` checks extraction of the underlying exception
even when clean Gazebo shutdown messages follow it, plus the no-exception case.
The built package and full suite passed: **141 tests**.

## Deliberately delayed-clock experiment

The validation-only harness `evidence/startup-clock/validate_delayed_clock.py`
wraps the existing clock bridge in a 12 wall-second launch delay. It uses ROS
domain 59 and a separate Gazebo partition. Production launch defaults are not
changed. The harness closes the simulation normally after 25 wall seconds.

The resulting log shows this sequence:

```text
[effort:clock-wait] ... clock=0.000000, joint=0.000000
[effort:clock-ready] ... clock=1.970000, joint=1.968000
[effort:activated] ...
```

There were 265 recorded active samples from simulation time 1.992 to 5.160 s.
Their source ages were approximately 0–0.001 s; there was no timing exception,
and the posture and Gazebo processes exited cleanly on requested shutdown.
This is a targeted startup test, not a complete recovery experiment. Full panel
recovery runs are recorded separately in `evidence/startup-clock/validation.json`.

The previous GUI validation source hashes remain a historical snapshot. This
fix intentionally changes the startup node and its timing helper. Comparisons
that demand identical hashes must start a fresh baseline rather than mixing old
and new controller-source snapshots, even though active control math is unchanged.


## Latest-sample feedback under GUI scheduling delays

The first full visible-Gazebo retest (`20261028-083131-70c754d1`) passed
startup and reached balance readiness, then failed the active age check with
`source_age=0.083000 s`. This is a distinct failure from uninitialized startup
time. Its logs remain preserved; it is not classified as a passing experiment.

The previous `qos_profile_sensor_data` retained up to five joint/IMU messages.
`hold_effort_stance.py` now subscribes to both with:

```python
feedback_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
```

After a scheduling pause, the controller needs the newest state rather than a
queue of older states. This queue policy can discard superseded messages. It
does not manufacture freshness, change timestamps or accept an actually stale
latest sample. The original age limits still reject a stale or future sample;
the existing gap handling still freezes the integral across long unknown gaps.
A trial with a frozen integration gap inside the measurement window remains
invalid under the existing runner audit.

An isolated real DDS test is saved as
`evidence/startup-clock/validate_latest_feedback.py`. Two samples (times 1.000
and 1.080) are delivered while callbacks are intentionally not serviced. A
depth-five subscription receives `[1.000, 1.080]`, so the first sample would
fail the unchanged 50 ms age guard at time 1.080. A depth-one subscription
receives only `[1.080]`, which passes that guard. See `latest-feedback.json`.
This demonstrates the queue behavior; it does not prove the exact OS scheduling
cause of the observed GUI delay or guarantee timing on every future machine.

The runner's child-exception diagnostic now also applies to a simulation that
exits during recording, in addition to one that fails before readiness.


## Completed visible-Gazebo recovery retest

Both fresh trials of GUI run `20261028-083455-f1aa4f08` used the updated
controller and passed: forward 1 N for 0.1 simulated seconds with Gazebo GUI
enabled, active IMU feedback and ROS domain 58. Both logged synchronized time
before activation. The push peak pitch change was
0.059 degrees, final return was within
bands, both feet had continuous fresh contact evidence, controller saturation
count was zero, and no integral-gap samples occurred in either measurement
window. The panel generated `response.png`. This validates the tested case,
not all forces, host loads or scheduling conditions.
