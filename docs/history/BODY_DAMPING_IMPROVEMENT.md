# Reducing rebound with body pitch-rate feedback

The previous stronger-ankle profile recovered from 25 N pushes, but swung back
noticeably and approached the 0.15 rad (8.59°) tilt guard. We changed one control
parameter: the IMU body pitch-rate gain, from 4 to 8 N·m·s/rad total.

## What the calculation means

For each ankle, before freshness/contact gates and the startup ramp:

```python
# Before: stronger ankle response
correction = clamp(0.5 * (20 * pitch + 4 * pitch_rate), -1.5, 1.5)
# After: damped ankle response
correction = clamp(0.5 * (20 * pitch + 8 * pitch_rate), -1.5, 1.5)
```

These are equivalent explanatory expressions; the implementation retains its
existing shared function and reads the selected gain from BalanceConfig.
`pitch` is the pelvis angle in radians, and `pitch_rate` its angular speed in
rad/s. The angle term asks for upright posture. The rate term reacts to motion
and brakes the swing. The factor 0.5 shares the correction between two ankles.
The robot's joint-axis convention makes positive ankle correction oppose
positive body pitch through the reaction torque.

For pitch = 0.05 rad and rate = 0.1 rad/s, each ankle requests 0.7 N·m before,
and 0.9 N·m afterward. If motion reverses, the rate term changes sign; it can
start braking while the body still leans in the original direction.

The balance correction still clips at ±1.5 N·m per ankle. It is added to the
joint posture controller before the existing total motor clamp. Hip/knee/ankle
motor limits remain 6/10/6 N·m per leg. Physical joint damping, geometry, mass,
angle gain 20, antiwindup, data-freshness gates and tilt guard did not change.
Increasing this gain adds damping through feedback; it does not change the
simulator's passive joint damping.

## Measured results

Each row used a fresh no-push reference and a fresh push simulation, full
measured settling, 0.25× pacing, 1 ms physics steps and a 10 s recovery window.
The candidate was repeated twice in each direction. All references and pushes
passed recovery, contact, COM coverage, motor composition and damping checks.

| Trial | Rate gain | Main lean | Opposite rebound | Last outside return bands after clear | Result |
| --- | --- | --- | --- | --- | --- |
| baseline-forward | 4 | 8.193° | 5.288° | 2.816 s | Pass |
| candidate-forward-1 | 8 | 7.779° | 4.001° | 2.391 s | Pass |
| candidate-backward-1 | 8 | 7.917° | 3.775° | 3.187 s | Pass |
| candidate-backward-2 | 8 | 7.958° | 3.789° | 3.128 s | Pass |
| candidate-forward-2 | 8 | 7.774° | 4.004° | 2.394 s | Pass |

Angles are changes from each trial's own pre-push baseline. The combined return
bands include pitch, rate, height and posture; “last outside” is a sampled
measurement, not an exact theoretical settling time. The final second must
remain inside the bands. Command windows were 0.200–0.201 s. Gazebo logs confirm
force handling, not calibrated force or exact actuation timing.

Forward comparison has a fresh gain-4 baseline in this study. Backward gain-4
comparisons are historical runs from `evidence/balance-cap-study-01`: main lean
8.414–8.485°, rebound 4.957–4.959°, last outside 2.814–2.817 s. They used the
same model, caps and measured protocol, but were not rerun as fresh backward
baselines in this study.

The new gain reduces the forward rebound from 5.288° to about 4.002° and the
backward rebound to 3.775–3.789°. Forward COM excursion relative to the feet
also drops from 4.911 cm to about 4.602 cm. The worst candidate main lean is
7.958°, leaving about 0.63° to the unchanged tilt guard.

There is a tradeoff: backward motion is smaller, but a small residual pitch
persists longer. Its combined bands settle at 3.128–3.187 s, compared with about
2.815 s historically. This is not a claim that all aspects recover faster.

COM remains a read-only diagnostic, not a feedback input. Its ground projection
being inside the support geometry does not itself prove dynamic stability.
Two repeats per direction support this specific result, not a robust stability
envelope. Higher forces and longer pulses have not been verified here.

## Which files changed, and why

- `src/my_robot_description/launch/effort_standing.launch.py`: exposes the
  selected `balance_kd` to the existing controller; validates 4 or 8. Previously
  the runner could only use the default gain. Default remains 4.
- `scripts/run_effort_push.py`: adds `--balance-kd`, forwards it to launch,
  records it in the report and validates gain 8 requires active balance with
  the 1.5 N·m correction cap.
- `scripts/run_visual_push.py`: forwards and records the same selection for
  visual runs, using the same controller implementation.
- `tools/force_panel/config.py`, `app.js`, `index.html`: add the optional damped
  profile. Selecting either experimental profile checks full measured settling
  automatically in visual mode. Original/default remains cap 0.5 and gain 4;
  stronger uses cap 1.5 and gain 4; damped uses cap 1.5 and gain 8.
- `scripts/run_body_damping_study.py`: runs the fixed comparison plan, freezes
  package source hashes and stops if a required case fails.
- `scripts/body_damping_metrics.py`: independently computes baseline-relative,
  direction-aware main lean and opposite rebound from completed recordings.
- `scripts/analyze_body_damping_study.py`: writes assessment and the comparison
  plot, keeping analysis separate from the motor controller.
- Tests cover gain behavior, clipping, stale-data gating, launch defaults,
  direction-aware metrics and GUI argument forwarding. The suite passes all
  210 tests. `verification.json` checks actual logged gains and audits.

## Try it yourself

Open http://127.0.0.1:8765/ and choose **Visual only**, **25 N**, **0.20 s**,
and **Joint PID + IMU · damped ankle response (experimental)**. Leave **Use full
measured settling (slower)** checked to compare with the measured protocol.
Fast visual warmup intentionally differs and may produce a different outcome.
Stop closes the owned simulation. The panel's 2 N smoke test confirmed the
selected gain reaches the actual controller and the pulse clears successfully;
it was not counted as an additional measured recovery trial.

For a measured forward run in a fresh output directory:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/run_effort_push.py --gui \
  --force-x 25 --duration 0.2 --balance-enabled \
  --ankle-balance-limit 1.5 --balance-kd 8.0 --observe-com \
  --damping-model explicit --controller-mode pid --real-time-factor 0.25 \
  --ros-domain-id 74 --output evidence/my-damped-forward-01
```

Use `--force-x -25` and a new output directory for backward. The next useful
step is repeating controlled force-duration comparisons around this tested
point before increasing the claimed recovery range or adding stepping.
