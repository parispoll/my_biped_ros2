# Step 7b: forward characterization and duration repeats

The protocol tests +/-2 N at 0.20 s and 0.30 s with IMU ankle feedback active,
explicit damping, joint PID and unchanged source snapshots. The earlier valid
backward comparison is reused in place. A fresh forward comparison establishes
its first pair; then each direction gets a fresh repeat with duration order
reversed (0.30 s then 0.20 s). Every individual duration still starts a fresh
zero-force reference and a fresh push world.

## Commands

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/run_duration_comparison.py --direction forward \
  --output evidence/my-forward-duration-01
python3 scripts/run_duration_repeat_study.py \
  --backward-seed evidence/duration-comparison/backward-01 \
  --forward-seed evidence/my-forward-duration-01 \
  --output evidence/my-duration-repeats-01
python3 scripts/plot_duration_repeats.py evidence/my-duration-repeats-01
```

Choose fresh output names. The first forward comparison takes about nine
minutes. The two reversed-duration comparisons take about eighteen more. Run
sequentially on an otherwise lightly loaded host. These runners use ROS domain
63 plus unique Gazebo partitions. The GUI panel uses domain 58.

`run_duration_comparison.py` defaults to backward as before; `--direction`
selects the force sign and `--reverse-order` reverses case order. Its plotter
now labels the chosen direction. No robot-package source, gain or torque limit
is changed for this study.

## Verification and statistics

`duration_repeat_analysis.py` rereads the raw trial summaries, checks direction,
duration, PID/explicit-damping settings, physics step, pacing, active balance,
measurement-gap/observer/damping audits and force queue/clear evidence. It
recomputes peak metrics instead of trusting the comparison table. Incomplete
runs or failed recoveries cannot be counted as successful repeats; identical
paths cannot be counted twice. The source snapshot must match both seeds and
remain unchanged throughout the study.

`run_duration_repeat_study.py` owns the sequential child comparisons and saves
its progress after each. A physical recovery failure stops later cases and is
retained as such. A simulation/sensor/timing error is an invalid experiment,
with its original logs preserved. Neither script silently retries. If a human
or this engineering chat explicitly runs a new attempt, its evidence gets a
new path and the excluded attempt is documented.

`plot_duration_repeats.py` shows mean peak lean, individual points and observed
min/max ranges for the two repeats per direction/duration. These are descriptive
ranges, not confidence intervals. Paired percentage increases compare 0.30 s
with 0.20 s within the same repeat. Two repeats do not establish a stability
boundary. Source timestamps after push and after clear must be distinguished
when discussing recovery time, as in Step 7a.

## Current attempts

`evidence/duration-comparison/forward-01` was invalidated in its first no-push
reference by the existing joint-feedback guard (`source_age=-0.024 s`,
`source_gap=0.068 s`). It contains no valid forward push result and is excluded
from statistics. No timing bound or controller source was changed in response.
The fresh attempt is `forward-valid-01`; the repeat study uses its verified
outcome only if it completes successfully.

New tests cover paired statistics, distinct repeat/path checks, wrong force,
nonfinite peaks, source mismatch and rejection of an invalid raw audit even
when the top-level summary says passed. The full suite passed **156 tests**
before the fresh simulation attempt began.


## Completed results

All eight valid pushes and eight fresh references passed the recorded checks.
Source snapshots stayed unchanged. Every measurement window had zero frozen
integration-gap samples and every trial had zero saturated controller samples.
Both feet retained fresh contact evidence. The original invalid forward
reference remains excluded and linked in the study and verification JSON.

| Direction | Peak lean at 0.20 s (observed range) | Peak lean at 0.30 s (observed range) | Paired peak increase |
| --- | --- | --- | --- |
| Forward | 0.285–0.288° | 0.419–0.420° | 45.74–46.94% |
| Backward | 0.385–0.387° | 0.566–0.568° | 46.41–47.67% |

The longer pulse produced more peak lean in both repeats of both directions.
Forward peak lean remained smaller than backward peak lean. The paired effect
ranges are about 46–48%, near the 50% increase in nominal requested impulse for
these particular conditions. This is descriptive two-repeat evidence, not a
general linear force law, a confidence interval, or the robot's failure limit.

![Observed duration-repeat summary](../evidence/duration-comparison/repeat-study-01/repeat-summary.png)

Detailed case records, seed links, source hashes, audit verification and test
logs are in `evidence/duration-comparison/repeat-study-01/`. Direction-specific
recorded trace plots also live in the original seed and fresh comparison
folders. The forward seed used is `forward-valid-01`.
