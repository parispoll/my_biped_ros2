# First balance improvement: earlier ankle correction

The original controller reproduced its 25 N, 0.20 s forward tilt-guard stop in a fresh measured trial. Increasing only the per-ankle IMU correction allowance from 0.5 to 1.5 N m then passed four fresh measured recovery trials: twice forward and twice backward. The second round reversed the direction order. Every case had its own passing no-push reference.

## Why this change

The previous 25 N records show correction clipping at about 0.05 s forward and 0.06 s backward after force publication, while total ankle motor commands do not hit their cap until about 0.254 s and 0.240 s respectively. The smaller software allowance constrains the body correction before the motor budget is exhausted. That gave us a specific testable change, rather than an arbitrary gain increase.

## Before and after

```text
Per-ankle balance correction = clamp((20 * pelvis_pitch + 4 * pitch_rate) / 2, -B, B)

Original:     B = 0.5 N m
Experimental: B = 1.5 N m

Total joint request = feedforward + posture P + posture D + bounded integral + balance correction
Motor command = clamp(total joint request, -motor_cap, motor_cap)
```

The hip/knee/ankle motor command caps remain `[6, 10, 6] N m` per leg. Model mass, geometry, physics, controller gains, anti-windup, contact/IMU gates, measured settling and the 0.15-rad tilt guard were not increased or bypassed. The existing anti-windup prediction already includes the balance correction.

The controller still uses IMU feedback. COM is an independent diagnostic recording; this step does not introduce COM feedback or stepping.

## Measured results

| Case | Correction cap | Outcome | Peak pitch change |
| --- | ---: | --- | ---: |
| Original forward | 0.5 N m | Tilt-guard stopped partial trial | Full-response peak unavailable |
| candidate-forward-1 | 1.5 N m | Recovery checks passed | 8.25° |
| candidate-backward-1 | 1.5 N m | Recovery checks passed | 8.49° |
| candidate-backward-2 | 1.5 N m | Recovery checks passed | 8.41° |
| candidate-forward-2 | 1.5 N m | Recovery checks passed | 8.19° |

All four candidate pushes passed the return-band, fresh-contact, balance-composition, viscous-damping, timing and COM-coverage checks. Mass remained 14.1 kg throughout. COM recordings had maximum source-sample gaps of 0.063 s, within the COM coverage requirement. Both ankles still reached 6 N m; this improvement uses the existing budget earlier rather than strengthening the motors.

The first candidate forward trial reached about 4.96 cm of COM fore/aft displacement relative to its baseline and current foot centers, then returned. The original partial record reached about 5.42 cm before cutoff. The plot marks the original cutoff explicitly: it is not a completed original response or a measured final fall. Finite-difference COM speed estimates in assessment.json are descriptive, not independently calibrated velocity measurements.

The backward peak is close to the existing 8.59-degree guard, with roughly 0.1 degree of margin in the higher of its two observed runs. This is an experimental improvement for the tested pulse, not proof of a robust recovery envelope or of recovery at longer/stronger pushes. No other force/duration regression sweep was run in this step. The next useful tuning experiment is to reduce rebound and increase this margin, rather than jumping straight to a stronger force.

## Fast visual warmup is a different protocol

A panel-launched 25 N candidate visual run with fast warmup still tripped the tilt guard (`evidence/gui/20261029-111123-82c201da`). Its published command window was about 0.207 s, versus approximately 0.20 s in measured cases. Both the settling protocol and timing differ, so this is preserved as a distinct result; it does not reproduce the measured success.

The panel now exposes **Use full measured settling (slower)** in visual mode. Selecting the experimental balance option recommends/checks it, and the user can uncheck it to retain fast demonstration behavior. The original profile remains the default, with fast visual warmup still available. Full visual settling retains the visual-mode behavior of keeping Gazebo open after a controller exit. It does not turn the visual run into a measured pass/fail experiment.

## Use the profile

In http://127.0.0.1:8765/ choose **Joint PID + IMU · stronger ankle response (experimental)** under Balance controller. For a visual comparison, leave **Use full measured settling** checked, use 25 N and 0.20 s, and start a new world. Full settling is slower because it matches the measured readiness conditions.

For a measured single pair:

```bash
cd '/home/parispoll/Documents/ChatGPT/robotics carrer'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/run_effort_push.py --force-x 25 --duration .2 --balance-enabled --ankle-balance-limit 1.5 --observe-com --damping-model explicit --controller-mode pid --real-time-factor .25 --ros-domain-id 73 --output evidence/my-cap-check-01
```

Choose a new output folder. Omit `--ankle-balance-limit 1.5` to use the original cap. To reproduce the whole comparison run `python3 scripts/run_balance_cap_study.py --output evidence/balance-cap-study-02`, then `python3 scripts/analyze_balance_cap_study.py evidence/balance-cap-study-02`.

## Files and responsibilities

- `src/my_robot_description/launch/effort_standing.launch.py`: adds explicit bounded `ankle_balance_limit` launch input; passes it to the existing balance formula. Also adds the visual fast/full readiness selector; measured readiness remains unchanged.
- `scripts/run_effort_push.py`: forwards the correction cap, optionally records COM, and checks COM coverage. Defaults preserve the original correction cap and no extra observer.
- `src/my_robot_description/my_robot_description/com_audit.py`: validates COM coverage, finite coordinates and constant mass in completed measured windows.
- `scripts/run_balance_cap_study.py`: fresh baseline plus four fixed-pulse candidate checks, frozen source snapshot and stop-on-nonpass behavior.
- `scripts/analyze_balance_cap_study.py`: separate COM/torque diagnosis and comparison plot.
- `scripts/run_visual_push.py`: explicit correction-cap and full-settling options; records the captured settings.
- `tools/force_panel/config.py` and `static/app.js`, `static/index.html`: explicit original/experimental selection and visible settling choice; captured settings are restored during a run and labeled in history.
- Tests cover the larger correction, unchanged final motor clamp, stale-contact gating, COM coverage failures, original defaults, request validation and readiness routing.

Artifacts are in `evidence/balance-cap-study-01`: study.json, assessment.json, comparison.png, per-trial logs and COM recordings, plus the full test output. Robot sources were frozen during the measured comparison. Later GUI integration added only the visual-readiness routing to the package launch file; the measured branch still uses the same full readiness. Capture a new snapshot for any future comparison.

## Final panel verification

The full-settling forward visual check (`evidence/gui/20261029-111402-98d51b59`)
used the captured 1.5 N m profile, published a 0.201 s command window, reached
about 8.16 degrees maximum recorded tilt, and continued without a guard stop
for over ten simulated seconds after clearing. It was then stopped cleanly.
This verifies panel routing and the comparable warmup option; it is not an
additional measured pass/fail trial. The fast-warmup guard-stopped case remains
preserved separately. All 204 unit tests passed after final UI integration.
The panel is left open with the experimental profile and full settling selected.
