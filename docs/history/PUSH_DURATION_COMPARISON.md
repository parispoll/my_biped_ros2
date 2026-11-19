# Step 7a: increase the duration before increasing force limits

The next characterization keeps the backward force at **2 N** and compares
**0.20 s** with **0.30 s** pulses. Both are inside the existing effort profile.
The robot, controller gains, torque caps, timing guards and return bands stay
unchanged. A complete source snapshot is recorded for the robot package and
checked after each case.

Requested impulse magnitude is force multiplied by duration:

```text
2 N × 0.20 s = 0.40 N s
2 N × 0.30 s = 0.60 N s
Increase = (0.60 / 0.40 - 1) × 100 = 50%
```

This is nominal commanded impulse, not an independently measured force
integral. The measured command window is recorded separately. A 50% larger
requested impulse does not imply 50% more lean: motor control, gravity and
contact dynamics act throughout the pulse and recovery.

## Run it

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/run_duration_comparison.py \
  --output evidence/my-duration-comparison-01
python3 scripts/plot_duration_comparison.py evidence/my-duration-comparison-01
```

Use a new output folder. Add `--gui` to the runner to see Gazebo; headless is
the default. It uses ROS domain 63 and unique Gazebo partitions, so avoid another
manual experiment on that domain. Expect about nine minutes at quarter speed.

Each duration gets a fresh no-push reference and a fresh push trial. The script
checks the first case before escalating to the longer duration. A completed
case that fails recovery stops escalation and retains that physical result.
An incomplete simulation/sensor/clock run instead records `experiment_error`;
it cannot be used to claim a falling or recovery limit. There are no automatic
retries. Historical evidence is not overwritten.

## Files and what to read

- `scripts/run_duration_comparison.py`: fixed experimental plan, child runner
  lifecycle, source snapshots, stop rules and comparison table in JSON.
- `scripts/plot_duration_comparison.py`: recorded pitch and pitch-rate traces.
  Dotted lines show each actual command-clear publication time. Green bands
  show the pitch/rate components of the existing return criteria; the complete
  verdict also includes height, joints, contacts and source/controller audits.
- `tests/test_duration_comparison.py`: verifies nominal impulse/units and keeps
  incomplete experiments distinct from completed failed recoveries.
- `evidence/duration-comparison/backward-01/`: current experiment and all raw
  reference/push evidence, per-case logs and `comparison.json`.

One pair at each duration is descriptive characterization. It does not find the
robot's maximum safe force or establish a general stability boundary. If both
cases pass, repeat or extend this plan before increasing the validated force
limit. If a physical case fails, inspect the traces and contact/torque evidence
before deciding whether tuning or a different recovery strategy is needed.


## Recorded result

Both fresh reference/push pairs passed, with no controller command saturation
and no integral-gap samples in any measurement window. The package source
snapshots remained unchanged. The full unit suite passed 150 tests.

| Backward force | Duration | Requested impulse magnitude | Peak pitch change | Last outside bands after push / after clear | Recovery |
| --- | --- | --- | --- | --- | --- |
| 2 N | 0.20 s | 0.40 N s | 0.385° | 0.905 / 0.705 s | Passed |
| 2 N | 0.30 s | 0.60 N s | 0.568° | 0.928 / 0.627 s | Passed |

The larger requested impulse produced 47.7% more peak lean in this
comparison. Both feet retained fresh ground-contact evidence. Maximum pitch
rate was similar (0.0402 versus 0.0392 rad/s), so a longer disturbance can
increase displacement without increasing peak speed. The last-outside-band
time measured after clear decreased, but measured after push onset increased
slightly: the longer pulse was also cleared later. These timestamps denote
the last recorded violation, not exact settling to the initial pose.

![Recorded duration responses](../evidence/duration-comparison/backward-01/response-comparison.png)

This is one pair per duration, not repeat evidence or the robot's maximum
recoverable disturbance. A useful next characterization is the corresponding
forward-duration comparison before increasing the Newton limit.
