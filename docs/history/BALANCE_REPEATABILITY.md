# Step 6c: repeat the balance experiment before increasing difficulty

The first active-IMU experiment showed lower peak lean, with a forward return-time
tradeoff. One off/on pair in each direction cannot tell us how consistently the
result occurs. This step repeats that protocol without changing the controller,
gains, physical model, torque caps or recovery bands.

## Protocol chosen before the fresh repeat

- Reuse `evidence/active-balance/paired-04` only after checking its completed
  recordings, command/contact/damping audits and exact controller/model hashes.
- Run another four fresh push experiments, each with its own fresh zero-force
  reference. Reverse both direction order and off/on order relative to repeat 1:
  backward on/off, then forward off/on.
- Keep requested force ±2 N, pulse 0.2 s, baseline 2 s, recovery 10 s, physics
  step 1 ms and real-time factor 0.25. Off-mode settling remains matched to the
  active ramp/settle wait with `--posture-delay 3`.
- Report every completed pair, including worse peaks or failed recovery. An
  incomplete/invalid measurement is recorded as an error, not counted as a
  successful repeat or silently replaced with a favorable run.
- The descriptive consistency screen requires the existing recovery/contact/
  saturation/audit checks, off/on published command windows within 5%, and at
  least 5% peak-lean reduction in every pair, in both directions. The 5% peak
  screen is the same threshold used for the original paired comparison.

This gives **two off/on pairs per direction**, using four prior push trials and
four newly run push trials. Each has its own zero-force reference. Reversing
order reduces a simple order confound; it is not randomization or a guarantee
of statistical independence.

## What we summarize

Keep individual results alongside the mean, minimum, maximum and observed range
for peak pitch, peak pitch rate, final joint change and last out-of-band sample.
Calculate each pair's percentage peak reduction before summarizing percentages:

`reduction = 100 * (1 - peak_on / peak_off)`

Do not replace this with a ratio of pooled averages: that would answer a different
question. A null last-outside-band time means no recovery sample lay outside the
band; it is represented as zero seconds in the descriptive plot.

A lower peak can coexist with slower return. The forward return-time tradeoff
is explicitly retained, rather than hiding it in a single success label.

With n=2, ranges are descriptive observations, not confidence intervals, a
significance test or a general stability proof. Requested/published pulse timing
is checked; independently measured actuation impulses are not matched. Larger
pushes, noise, delay, terrain and lateral balance remain untested by this step.

## Files changed

| File | Responsibility |
| --- | --- |
| `scripts/run_active_balance_comparison.py` | Adds `--reverse-order` and records actual execution order; original default retained |
| `scripts/run_balance_repeat_study.py` | Verifies prior evidence, runs a fresh reversed repeat, freezes controller/model sources and saves the aggregate |
| `src/my_robot_description/my_robot_description/balance_repeats.py` | Pure descriptive statistics and consistency checks, without ROS |
| `scripts/plot_balance_repeats.py` | Individual-trial dots, observed ranges, paired peak reductions and return-time tradeoffs |
| `tests/test_balance_repeats.py` | Checks hand-calculated means/percentages, rejects malformed repeats, and verifies failures are not discarded |

No controller or physical-model files are changed in this step.

## Reproduce

From the workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash

python3 scripts/run_balance_repeat_study.py \
  --seed evidence/active-balance/paired-04 \
  --output evidence/my-balance-repeat-01

python3 scripts/plot_balance_repeats.py evidence/my-balance-repeat-01
```

Choose a new output name. The seed is referenced in place and never overwritten.
The new repeat takes roughly 16–20 minutes at quarter-speed pacing. The runner
owns and closes its simulations. Results live in `repeat-study.json`, with raw
new recordings under `repeat-02/`. A failed run leaves an incomplete report and
error message for inspection.


## Recorded results

| Direction | Mean peak off → on | Paired reduction range | Mean last outside bands off → on |
| --- | --- | --- | --- |
| Forward | 0.336° → 0.287° | 14.53–14.65% | 0.711 s → 0.988 s |
| Backward | 0.484° → 0.389° | 19.38–19.71% | 4.201 s → 0.700 s |

The predeclared consistency screen passed in every pair. All eight push trials and eight fresh zero-force references across the two studies passed the recorded checks. The controller/model hashes stayed unchanged, and the four Gazebo/DART/control vendor package versions matched the earlier physics audit.

The benefit repeated without retuning: lower peak lean in both directions. The tradeoff repeated too: forward return was slightly slower, while backward return was much faster. Final joint changes stayed within the existing 0.002 rad band, although they were not identical across trials.

![Repeat summary](../evidence/active-balance/repeat-study-01/repeat-summary.png)

Bars show means, dots show individual trials, and vertical lines show observed ranges. The percentage panel shows the paired improvements separately. The joint-offset panel retains the 2 mrad return-band reference.

The final build and all **123 unit tests** passed. See `evidence/active-balance/repeat-study-01/repeat-study.json` for individual comparisons/statistics and `verification.json` for the verification summary. The previously verified study was referenced in place; its files were not overwritten.

This supports proceeding to further characterization, not declaring the robot generally robust. Stronger pushes, varied pulse duration, measured actuation timing, sensor noise/delay, active-controller timestep sensitivity and support-force estimation remain separate work.
