# Observer first-IMU discovery fix

The GUI run `20261028-163715-ab221281` stopped in its no-push reference, before
any force experiment. Its `reference/balance.jsonl` was empty. The launch log
reported `Balance observer IMU absent for two wall seconds`. The active posture
node had already received its own IMU and activated, so this evidence identifies
an observer-specific first-message discovery timeout, not a robot falling or
a globally absent sensor. Different subscribers need not discover a publisher
at the same instant.

## Before

`observe_balance.py` used the same deadline for startup and an established stream:

```python
self.last_wall = time.monotonic()  # assigned before the first IMU sample
# Later, in supervise():
if time.monotonic() - self.last_wall > 2:
    raise RuntimeError('Balance observer IMU absent for two wall seconds')
```

Thus, a subscriber that took slightly over two seconds to receive its first
sample was incorrectly treated as an already established stream that had failed.
The diagnostic observer's exit intentionally stops the audited simulation.

## After

`src/my_robot_description/scripts/observe_balance.py` now stores:

```python
self.started_wall = time.monotonic()
self.last_wall = None  # no valid IMU received yet
```

The pure function in
`src/my_robot_description/my_robot_description/imu_watchdog.py` implements:

```python
if last_receipt_wall is None:
    if now_wall - started_wall > 10:
        raise RuntimeError('... first IMU sample within ten wall seconds')
elif now_wall - last_receipt_wall > 2:
    raise RuntimeError('Balance observer IMU absent for two wall seconds')
```

The first successfully validated and recorded message sets `last_wall` and
logs `[balance:imu-ready]` with its discovery delay. Immediately from that first
sample, the original two-second deadline applies—even if the observer is still
within the ten-second startup period. Invalid frames, invalid sensor data and
non-increasing IMU timestamps retain their existing rejection behavior.

This first change modifies only the passive observer's startup policy. At that checkpoint the active controller was unchanged. A subsequent
joint-feedback startup correction is described below; gains, caps, contact gates
and force parameters remain unchanged. It is not an exemption for stale measurements inside
a recovery recording. The recorder and all measurement audits still have to
pass before an experiment is reported successful.

## Verification

`tests/test_imu_watchdog.py` tests the original 2.2-second discovery case,
startup timeout, established stream timeout, and the actual observer's
supervisor/first-valid-message wiring. Invalid first messages cannot mark the
observer ready. Build and the full suite passed: **162 tests**.

The isolated real ROS probe in `evidence/imu-discovery/validate_watchdog.py`
waited 3.2 seconds before publishing its first IMU message, delivered 30 valid
samples, then deliberately stopped publishing. The observer survived discovery,
recorded the samples, and exited with its expected two-second stream-loss
exception. That deliberate exit is a passing watchdog test, not a recovery run.
See `probe-result.json` and `probe.log`.

The GUI retest uses the user's original backward 2 N, 0.20 s request, with
Gazebo enabled. Its final outcome and source checks are recorded in
`evidence/imu-discovery/validation.json`. The original failed run remains in
GUI history.

Earlier duration studies remain historical evidence under their original
observer-source snapshot. Source-matched repeats that include observer source
hashes must use a fresh snapshot after this change, even though active control
and physics are unchanged.


## Separate joint-timing failure during the first GUI retest

`20261028-164043-7f6a03c8` passed its reference and the observer received IMU
in both worlds, but its push-world startup stopped at the existing active
joint-feedback guard (`source_age=0.083 s`, `source_gap=0.002 s`). This is not
the first-IMU timeout fixed here. It remains preserved as an invalid experiment.
After cleanup, diagnostics found no clock publishers or leftover robot
simulation processes. No controller timing bounds were widened or exceptions
suppressed. A further unchanged-profile GUI check uses
`20261028-164738-41085dd0`; the final outcome is recorded in validation.json.


## Joint feedback during pre-readiness settling

A further unchanged-profile retry (`20261028-164738-41085dd0`) also passed the
observer and reference, but its push-world startup encountered an 82 ms-old
joint sample. Both joint failures were before posture readiness.

`feedback_timing.integration_timing()` now has an explicit, default-off
`wait_for_startup` option. Its existing finite-time and backwards-time checks
still raise. When explicitly allowed before readiness, an age outside the
original [-0.02, +0.05] simulated-second bounds returns **no timing terms**.
The caller must discard that sample. The default behavior, used after
readiness, still raises the original timing exception. Age bounds themselves
are unchanged.

`hold_effort_stance.py` performs that check before advancing accepted timestamps
or computing/publishing from active feedback. During initial settling, rejected
samples cause no new torque command, no integral update, no accepted-timestamp
advance and no wall-watchdog refresh. They are counted and logged. Fresh
feedback then restarts the full settling interval; a long unknown interval
still freezes the integral for its first fresh update. Once `ready` is true,
timing violations remain fatal. All existing measurement-window audits remain.

While waiting, the previous bounded torque command remains at the interface.
The existing one-wall-second valid-joint-feedback watchdog still stops
prolonged loss; the IMU wall watchdog and tilt guard also remain active. This
is a change to startup handling, not permission to use stale feedback or to
continue a measured trial with missing control data.

`tests/test_startup_feedback_wait.py` exercises the actual feedback method:
no publishing/integrating from a rejected sample, full settling restart on
fresh data, integral-gap freezing, unchanged ready-state rejection, and no
watchdog refresh from repeated invalid startup samples. The build and full
suite passed **169 tests**. Final GUI validation uses
`20261028-165432-c2e2e047`; see validation.json for its outcome.

Together, the changed robot-package files are `scripts/observe_balance.py`,
`my_robot_description/imu_watchdog.py`, `scripts/hold_effort_stance.py` and
`my_robot_description/feedback_timing.py`. Earlier studies remain tied to
their earlier startup-source snapshots; fresh comparisons must capture these
new sources rather than mixing them silently.


## Recorder and query startup before the measured phase

The next retest (`20261028-165432-c2e2e047`) reached readiness but then hit an
83 ms joint-age violation as the reference recorder was starting. It remains
invalid; the ready-state timing exception was not suppressed. This demonstrates
that tolerating discovery only during settling was insufficient when new ROS
participants were still being started after readiness. The exact transport or
OS scheduling mechanism is not established by these logs.

The runner now starts its hardware-interface query and recorder during warmup,
following effort activation. The recorder subscribes early and waits for an
explicit `recording.ready` file. Only after the controller's existing readiness
marker and optional settling delay does the runner create that signal. The
recorder then requires its normal continuously fresh 0.5 simulated-second
window before beginning the unchanged baseline/push/recovery sequence.
No force can be published while the arm file is absent.

Changes are in `scripts/run_effort_push.py`, `scripts/effort_push_experiment.py`
and the shared `scripts/push_experiment.py`. Manual ungated recorders preserve
their existing default behavior. Only the recorder's total wall-clock budget
includes the additional warmup wait; sample-age limits and measurement audits
are unchanged. The saved summary records recorder connection, arm signal and
recording-start wall times for checking their order.

`tests/test_recorder_ready_gate.py` runs the actual recorder loop against fake
fresh sensor streams: a closed gate never publishes a force, opening it still
requires a fresh baseline, and an ungated manual run retains its behavior.
The diagnostic helper `experiment_failure.py` now selects the triggering Python
exception before secondary cleanup exceptions, with its own regression test.
The full suite passed **173 tests**. Final GUI test:
`20261028-170156-30c2ef7d`.


## Successful complete GUI validation

Run `20261028-170156-30c2ef7d` completed and passed with the user's original
backward 2 N / 0.20 s / Gazebo-enabled settings. Both reference and push
passed, with no saturated samples and no frozen integral-gap samples inside
their measurement windows. The peak pitch change was
0.389 degrees.

The reference actually reproduced a 2.899-second first-IMU delay, which the
old two-second discovery policy would have rejected. It discarded one startup
joint sample; the push world discarded three. Each returned to fresh data
and completed its restarted settling before readiness. The saved arming times
confirm `recorder connected < readiness signal < recording started` in both
worlds. No new process startup was required to begin measurement after the
signal. Every ready-state and measurement audit passed.

`evidence/imu-discovery/validation.json` links the original and intermediate
failed attempts and records the final source snapshot. Physics, gains and
torque caps match the duration-repeat snapshot; observer/startup/diagnostic
code intentionally changes as documented above. This validates the tested
case and startup conditions, not every possible host load or force.
