# Testing nearby force and duration settings

This step measures the existing damped controller. No controller gain, motor
limit, passive damping, robot geometry or posture guard is increased.

## Why this comes before another control change

Four successful 25 N / 0.20 s pushes are evidence for that tested condition.
They do not show how much extra force or pulse length the robot can withstand.
We vary one disturbance parameter at a time so the result can be interpreted.

| Condition | Force magnitude | Requested duration | Requested impulse | Change from prior condition |
| --- | --- | --- | --- | --- |
| Prior measured condition | 25 N | 0.20 s | 5.0 N·s | — |
| Force test | 27 N | 0.20 s | 5.4 N·s | Force +8%; duration fixed |
| Duration test | 25 N | 0.22 s | 5.5 N·s | Duration +10%; force fixed |

Impulse is force multiplied by time for an ideal rectangular pulse:

```text
J = F × Δt
27 × 0.20 = 5.4 N·s
25 × 0.22 = 5.5 N·s
```

These are requested impulses. ROS/Gazebo actuation delay and transport timing
are not calibrated force measurements. The recorded publish-to-clear window
is also reported, separately from the requested duration.

Similar impulse does not guarantee similar motion. Torque about the feet,
body angle and angular speed evolve during a pulse. The feedback and motor
limits can become active at different times. Two force-duration pairs cannot
be collapsed into a universal “maximum impulse” from this experiment.

## What remains fixed

- Total body angle gain 20 and pitch-rate gain 8.
- IMU ankle correction cap ±1.5 N·m per ankle.
- Hip/knee/ankle motor limits 6/10/6 N·m per leg.
- Full measured settling, active joint PID and IMU feedback.
- Explicit passive damping, 1 ms physics steps and 0.25× pacing.
- Fresh no-push reference and fresh push world for every case.
- Ten simulated seconds of recovery recording after the pulse.
- Existing 0.15 rad absolute pelvis-tilt guard and data/contact checks.

Each condition is repeated twice forward and twice backward. Direction order
reverses on repeat two. The previous 25 N / 0.20 s measurements provide a
historical comparison; they are not fresh baselines in this batch. Every new
push still gets its own fresh no-push reference.

## Reading the outcomes

**Recovered** means the full paired experiment completed and its checks passed.
A full recording can also complete without satisfying recovery criteria.

**Posture guard stopped partial trial** means the fresh reference passed, the
push was published and the controller hit its existing posture guard. This is
evidence that controlled standing exceeded the allowed posture in that trial.
It is not a complete recovery record, a measured falling threshold, or proof
that the robot would never recover with the guard removed.

**Incomplete requiring diagnosis** means an infrastructure or unclassified
failure needs inspection. The study stops instead of counting it as a physical
failure. Partial samples and logs are retained for every trial.

The plot uses pitch change relative to each pre-push baseline, with backward
sign reversed for comparison. The guard operates on absolute pelvis tilt,
not that plotted baseline-relative pitch. Transport and shutdown can leave
samples after the guard crossing; a partial trace maximum is not a full
controlled-response peak.

## Files and reproduction

`scripts/run_damped_neighborhood.py` defines the two conditions, repeat order,
fixed controller arguments and outcome classification. It reuses the existing
measured runner instead of implementing another controller or recorder.
It snapshots package sources and stops if they change during the study.

`scripts/analyze_damped_neighborhood.py` reads completed and partial artifacts,
uses recovery metrics only for completed trials, and creates `assessment.json`
and `comparison.png`. Analysis stays separate from experiment execution.

To run the complete batch in a new evidence directory:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/run_damped_neighborhood.py \
  --output evidence/my-damped-neighborhood-01
python3 scripts/analyze_damped_neighborhood.py \
  evidence/my-damped-neighborhood-01
```

Allow several minutes per fresh paired trial. Keep other simulations idle
while collecting the measurements to reduce scheduling interference.

## Force comparison observed in this batch

All four 27 N / 0.20 s trials reached the unchanged pelvis-tilt guard: two
forward and two backward. Their fresh no-push references passed. Published
push-to-clear windows were 0.200, 0.201, 0.202 and 0.201 s. These are consistent
guarded posture outcomes, not eight-degree full-recovery measurements.

The previous 25 N / 0.20 s successes remain valid at their tested condition.
The new results show that adding only 2 N removes the available guard margin
in these repeated tests. They do not identify an exact threshold between
25 and 27 N, or a physical falling limit with guards disabled.

## Final results

| Condition | Forward | Backward | Fresh references |
| --- | --- | --- | --- |
| Prior 25 N / 0.20 s | 2/2 recovered | 2/2 recovered | Passed in prior study |
| New 27 N / 0.20 s | 2/2 tilt-guard stops | 2/2 tilt-guard stops | 4/4 passed |
| New 25 N / 0.22 s | 2/2 tilt-guard stops | 2/2 tilt-guard stops | 4/4 passed |

All eight new pushes stopped on the first swing, before a full recovery window
was available. The longer-pulse publish-to-clear windows were 0.220–0.222 s.
The ankle motor command reached its existing 6 N·m cap in every new push.
The response recordings show fresh ground-contact evidence for both feet up
to the partial recording's end; no sampled balance gate turned off there.
This does not prove continuous load-bearing or full recovery contact coverage.

The evidence checker recomputed the balance correction and total motor clamp
from actual logged terms. Maximum motor composition discrepancy was below
1e-15 N·m. Actual gain 8, angle gain 20, correction cap 1.5 and motor limits
6/10/6 per leg were verified. Gazebo queued and cleared every pulse. Package
sources matched the frozen snapshot. The full existing suite passed 210 tests.
A delayed-feedback warning occurred during one longer-pulse trial before the push;
no response sample in these partial trials froze the integrator for a feedback
gap. The logs remain available rather than hiding this diagnostic.

`scripts/verify_damped_neighborhood.py` performs these saved-evidence checks.
Run it with the workspace sourced, or with
`PYTHONPATH=src/my_robot_description`. Its `passed` flag means the evidence
checks passed; it does not mean the guarded pushes recovered.

Artifacts are in `evidence/damped-neighborhood-01/`: `study.json` contains the
plan and classifications, `assessment.json` the diagnostics, `verification.json`
the independent checks, `comparison.png` the plotted responses, and
`unit-tests.txt` the test result. Every trial retains its reference, partial
push samples, controller trace, COM samples and launch logs.

## What this tells us to develop next

The next control investigation should inspect how available hip and ankle
torques are coordinated during the first swing, using the successful 25 N
trace as a comparison. Reaching the ankle cap is evidence of a constraint,
not proof that raising that cap is the correct fix. Keep the current motor
budget and tilt guard while evaluating a separate, bounded feedback candidate.
Any improvement needs fresh A/B measurements at the same failing conditions.
No controller or GUI file was changed by this measurement step.
