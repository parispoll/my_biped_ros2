# Step 6b: enable bounded IMU ankle feedback

This step adds an optional body-pitch correction to the existing joint PID.
It uses the same candidate law inspected in Step 6a, now connected to the motor
command calculation. It is enabled explicitly with `--balance-enabled`.

## Before and after

Before, at each joint:

`request = static_feedforward + joint_P + joint_D + joint_I`

After, when balance is enabled:

`request = static_feedforward + joint_P + joint_D + joint_I + balance`

`command = clip(request, -motor_cap, +motor_cap)`

Only the left and right ankles receive `balance`. Both hips and knees receive
zero additional balance contribution. Total motor caps stay ±6/10/6 N m per leg.
Passive physical damping still acts separately at each physics step.

## What the balance term does

For world-referenced pelvis pitch and its Euler pitch rate:

`raw_per_ankle = (20 * pitch + 4 * pitch_rate) / 2`

`balance_per_ankle = ramp * clip(raw_per_ankle, -0.5, +0.5)`

Pitch uses radians; pitch rate uses radians/s. Positive forward lean gives a
positive ankle motor correction. The foot/upper-body reaction sign and the
body-frame gyro conversion are derived in [the observer guide](BALANCE_OBSERVER.md).
The total body correction is split equally under a double-support approximation.
These gains are assessed through the experiments below, not a general stability
proof for this humanoid.

The 0.5 N m limit bounds the extra ankle request. The total motor clamp happens
**after** adding every contribution, so adding balance never raises a motor cap.

## Startup and measurement gates

1. The existing effort posture first meets its normal readiness checks.
2. The correction ramps from zero to full strength over one simulated second.
3. The experiment waits three simulated seconds after posture readiness before
   beginning its baseline, allowing the ramp to finish before measurement.
4. The correction requires fresh IMU orientation/rate and fresh collision
   evidence for both feet touching the ground. Maximum source age is 0.05 s.
   Up to 0.01 s of apparent future time is tolerated for ordering between the
   independent IMU/contact and joint-state streams.
5. An invalid gate produces zero extra correction. Any gated-off controller
   sample during the recorded measurement window makes the active audit fail.
   This is a double-support experiment, not a contact-loss recovery controller.

The existing tilt, joint-angle, joint-speed and feedback-gap shutdown checks
remain in place. Collision evidence does not measure load sharing or support
force, so equal force distribution is an assumption rather than a measurement.

The raw 1 kHz contact streams are sampled in a separate `sample_contacts.py`
node. It forwards the latest **original** contact messages at 50 simulated Hz,
without replacing their source timestamps. This keeps the high-rate contact
callbacks out of the motor controller executor; stale evidence still expires.

## Why the integral update changed too

The joint integral's anti-windup prediction must see the **combined** request.
Otherwise it could keep accumulating while balance has already pushed the motor
into saturation.

Before:

```python
integral.update(error, dt, pd_request, motor_caps)
```

After:

```python
base_with_balance = pd_request + balance
integral.update(error, dt, base_with_balance, motor_caps)
request = base_with_balance + integral
command = clamp(request, motor_caps)
```

The integral gains and ±1 N m contribution limits are unchanged. When balance
is disabled, its vector is all zero and the original joint PID calculation is
retained. A unit test checks the case where PD alone is below the cap but
PD + balance is above it: further integral windup must stop, while unwinding
remains allowed.

## Files and responsibilities

| File | Change |
| --- | --- |
| `my_robot_description/balance_control.py` | New pure gate/ramp calculation |
| `scripts/hold_effort_stance.py` inside the package | Read IMU pitch/rate and contact evidence; add ankle correction; combine saturation prediction; log each term |
| `scripts/sample_contacts.py` inside the package | New 50 Hz contact forwarding with original timestamps |
| Package `CMakeLists.txt` | Install the contact sampler |
| `config/balance_control.yaml` | Active body gains, proposal limit, upright target and ramp duration |
| `launch/effort_standing.launch.py` | New `balance_enabled` argument, default false; load active configuration |
| Workspace `scripts/run_effort_push.py` | New `--balance-enabled`; implies observer recording; waits for balance readiness |
| `my_robot_description/balance_audit.py` | Verify active formula, sensor gate, complete motor request and bounded motor command |
| `tests/test_balance_control.py` | Gate failures, ramp, ankle-only vector, combined anti-windup and active audit |

Package paths above are relative to `src/my_robot_description/`. The pure
proposal module `balance.py` and the read-only observer remain separate. The
observer still has no motor publishers. The posture node is the single command
publisher, which prevents competing controllers from writing to the same topic.

Each active controller log row includes IMU pitch/rate and source age, both
contact ages/flags, balance settings, time since readiness, ramp, gate state and
six additional requested torques. The audit independently recomputes that
contribution and the final motor-cap clamp.

## Reproduce

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash

python3 scripts/run_effort_push.py \
  --balance-enabled \
  --damping-model explicit --controller-mode pid \
  --real-time-factor .25 --force-x 2 --gui \
  --output evidence/my-active-balance-01
```

Use a new output directory each time. The runner creates a fresh no-push
reference followed by a fresh push trial and closes both. Expect several minutes
at quarter-speed pacing. Use `--force-x -2` with another folder for the backward
push. For an observer-only comparison, replace `--balance-enabled` with
`--balance-observer` and keep every other setting fixed.

Active balance currently requires explicit physical damping and PID posture.
The original recovery bands and the 10 s recovery observation are unchanged.

## Interpreting results

Compare peak pitch change, pitch rate, the last sample outside any recovery band,
remaining joint offsets, contact evidence and motor saturation. A smaller peak
alone does not establish better overall recovery if another criterion fails.
Published force windows must be comparable; Gazebo queue/clear logs confirm
handling, but are not an independent force measurement or exact actuation clock.

The plotted balance term is an additional **requested motor contribution**.
Physical torque is not independently measured. If the final motor clamp clips,
the effective change in the command can be smaller than that contribution.

## Timing and recorder isolation

Two timing problems appeared while integrating this step, in both active and
feedback-off runs. The fixes separate measurement infrastructure from motor
feedback rather than accepting stale control data:

- Raw contact callbacks moved to the separate 50 Hz sampler described above.
- The controller writes `controller.setup.json` beside its JSONL recording at
  startup. It contains the actual computed joint targets and motor limits.
  The normal runner passes it to `effort_push_experiment.py`, which reads it
  using `posture_setup.py`. It no longer requests the large robot-description
  parameter from the live Python controller while feedback is running. The
  direct experiment script retains its older RPC fallback for compatibility;
  the normal runner always uses startup metadata.
- `feedback_timing.py` distinguishes a fresh sample after a dropped-message gap
  from an old sample. A gap over 0.05 simulated second freezes the integral for
  that update: no error is integrated across the unobserved interval. The pure
  integral still accepts at most 0.05 s. A current sample more than 0.05 s old,
  or more than 0.02 s ahead of the received ROS clock, is rejected. Timestamp
  reversal is still rejected.
- A recorded experiment containing an integral-freeze gap is rejected. A startup
  gap before measurement is logged and does not masquerade as uninterrupted
  measured feedback. The normal readiness and recorder settling checks still
  precede the baseline.

The timing helper, startup-metadata loader and corresponding tests are additional
files for this step. The metadata query failures were observed at recorder
startup; eliminating that live query avoids work on the controller's default
callback group at that point.

## Run the matched comparison

```bash
python3 scripts/run_active_balance_comparison.py \
  --output evidence/my-active-balance-comparison-01
```

This runs four fresh experiments: forward on/off and backward off/on. Each
experiment has its own fresh zero-force reference and push simulation. At
quarter-speed pacing the complete comparison can take roughly 16–20 minutes.
Source hashes freeze the controller, physical model and relevant configuration
throughout the comparison. `compare_active_balance.py` checks the resulting
records and creates a JSON comparison and four-panel plot for each direction.

The off runs use `--posture-delay 3` to match the active mode's three-second
ramp-and-settle wait after posture readiness. Both then use the same baseline,
pulse and recovery durations. If comparing manually, include that delay in the
observer-only command.

A passing comparison requires recovery in both modes, continuous fresh foot
contact evidence, no motor saturation, valid command/gate audits and published
command windows differing by at most 5%. The reported peak-lean improvement
flag additionally requires at least a 5% reduction in this pair. These are
practical experiment checks, not a formal stability proof. We also report the
last time outside any return band, so a smaller peak cannot conceal slower
recovery or another failed criterion.

For a standalone active run, plot its reference/push response with:

```bash
python3 scripts/plot_effort_push.py evidence/my-active-balance-01
```

`plot_balance_observer.py` now rejects active runs so it cannot incorrectly label
an applied-correction experiment as observation-only. Use the active comparison
plot for the added correction and its effect on the total command. Active gains
come from `balance_control.yaml`; the independent observer retains its candidate
settings in `balance_observer.yaml`. Both are 20/4 in this validation, and actual
active settings are recorded in the controller log.

A 30-second no-push check with active feedback is also supported:

```bash
python3 scripts/run_effort_standing.py \
  --balance-enabled --damping-model explicit --controller-mode pid \
  --real-time-factor .25 --duration 30 \
  --output evidence/my-active-standing-01
```

The standing runner now accepts the active flag, waits for balance readiness and
checks joint records over the actual pose-verification window. Feedback-freeze
gaps in that window invalidate the test. Existing inactive-mode checks remain
available.

## Which evidence to use

The final matched study is `evidence/active-balance/paired-04/`. Earlier folders
are retained for debugging and learning, not pooled into that comparison:

| Folder | Interpretation |
| --- | --- |
| `forward-active-01` | Aborted reference while raw high-rate contact callbacks were on the controller |
| `forward-active-02` | Passing early active prototype, before the later timing/recorder isolation changes |
| `paired-01` | Aborted feedback-off reference, exposing that the feedback-gap problem was not specific to balance |
| `paired-02` | Startup failed because the new Python timing module had not yet been installed; fixed by rebuilding |
| `paired-03` | Aborted reference at recorder startup under the new source-freshness check |
| `preliminary-forward` | Illustrative comparison with earlier observer data; not the final matched study |
| `paired-04` | Fresh forward/backward off/on cases using one frozen source/configuration snapshot and matched settling |

The controller-written setup file contains actual targets and limits, rather
than assuming the nominal knee bend. Its schema is checked before subscriptions
are attached. A unit test makes the live parameter client raise if called and
verifies that the setup-file recorder path still initializes successfully.


## Final measured results

| Push | Peak lean off → on | Peak reduction | Last outside bands off → on |
| --- | --- | ---: | --- |
| +2 N | 0.335° → 0.286° | 14.6% | 0.712 s → 0.988 s |
| -2 N | 0.486° → 0.390° | 19.7% | 4.198 s → 0.706 s |

Times are measured from force-clear publication and refer to the last recorded sample outside **any** existing return band. They are sampled experiment metrics, not general settling-time guarantees.

All four pushes and their four fresh no-push references passed. Every measurement window had zero integral-freeze gaps, valid contact/IMU gates, fresh ground-contact evidence for both feet and zero motor saturation. Published command-window differences between off/on were zero in both pairs. The controller/configuration hashes still match the study snapshot.

The active ankle contribution peaked at 0.0938 N m forward and 0.1315 N m backward, below the 0.5 N m per-ankle bound. The total motor caps were unchanged.

**Lower peak lean did not mean uniformly faster recovery.** Forward return was slightly slower; backward return was much faster in these runs. The D term opposes motion in either direction, including motion back toward upright, so reduced peak motion need not imply a faster return. More repetitions and a gain study are needed before generalizing the differences.

![Forward comparison](../evidence/active-balance/paired-04/forward-comparison/comparison.png)

![Backward comparison](../evidence/active-balance/paired-04/backward-comparison/comparison.png)

The separate active standing check passed 30 simulated seconds at real-time factor 0.5. Height stayed at 0.554486231 m, maximum IMU tilt was 0.004839°, both feet supplied 30,000 fresh contact samples, and no motor command saturated. Joint feedback was gap-free in the verification window.

Package build and **119 tests** passed. The summary is `evidence/active-balance/validation.json`; the raw study is under `paired-04` and the long standing check under `standing-30s-01`.

One backward-on shutdown log contains ROS subscription teardown exceptions after the runner requested SIGINT, after recording and command verification had completed. The raw log is retained; this was not a measurement-window failure. The standing run subsequently exited cleanly.
