# Measured force escalation: 3, 5 and 10 N

All six requested cases passed, each with its own passing no-push reference. No failed recovery was found in this range. Controller gains, balance cap, robot model, motor caps and physics settings were unchanged throughout; the package source hashes match the starting snapshot.

Each trial used a fresh world, 0.20 s requested force, 10 simulated seconds of recovery, explicit damping, PID posture plus IMU correction, 1 ms physics and 0.25 real-time factor, on isolated ROS domain 68. The measured profile retained its full readiness protocol; visual fast readiness and fall continuation were disabled.

| Case | Peak pitch change | Last outside bands after clear | Balance clipped response samples | Motor saturated samples |
| --- | ---: | ---: | ---: | ---: |
| forward-3N | 0.557° | 2.932 s | 0 / 863 | 0 |
| backward-3N | 0.693° | 3.447 s | 0 / 856 | 0 |
| forward-5N | 1.165° | 2.631 s | 0 / 857 | 0 |
| backward-5N | 1.314° | 1.667 s | 0 / 857 | 0 |
| forward-10N | 2.810° | 2.054 s | 44 / 855 | 0 |
| backward-10N | 3.059° | 2.545 s | 44 / 855 | 0 |

All six had continuous fresh collision evidence for both feet on ground_plane. This is collision evidence, not force-plate validation. Gazebo logs confirmed queueing and clearing the persistent wrench in each trial; command timing does not measure calibrated applied force or exact actuation timing.

The last-outside-band value is descriptive: it is the last recorded excursion outside the return thresholds after clearing. It is not a statistical estimate of settling time. One trial per direction/magnitude cannot establish repeatability or a stability boundary.

The balance correction first clipped at 10 N (0.50 N m per ankle). The total motor commands did not saturate, and the robot still returned within the measured bands. This identifies a candidate constraint to inspect at higher forces; it does not prove that raising a gain or cap will improve recovery. No controller changes were made.

The next measured escalation would be 15 then 20 N at the same 0.20 s in both directions, stopping at the first nonpass and diagnosing it before tuning. Repeat a borderline case before treating it as a reproducible recovery limit. A tilt-guard abort remains an incomplete guarded trial, not a completed fall/recovery measurement.

## New files

- `scripts/run_force_limit_study.py`: fixed six-case escalation, source snapshot, fresh reference/push pairs, stop on first nonpass.
- `scripts/analyze_force_response.py`: separate pitch/contact/balance clipping/motor saturation diagnostics; supports partial recordings without calling them completed measurements.
- `scripts/plot_force_limit_study.py`: three-panel comparison and source verification.
- `tests/test_force_limit_study.py` and `tests/test_force_response_analysis.py`: completed/incomplete outcome classification and partial-trace analysis.

## Reproduce

```bash
cd '/home/parispoll/Documents/ChatGPT/robotics carrer'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/run_force_limit_study.py --output evidence/force-limit-study-02
python3 scripts/plot_force_limit_study.py evidence/force-limit-study-02
```

Use a new output folder each time. Preserve the original evidence. The runner intentionally takes roughly 25–30 minutes for six fresh reference/push pairs at 0.25× speed.

Artifacts: `evidence/force-limit-study-01/study.json`, `comparison.png`, `verification.json`, per-case `diagnosis.json`, full launch/controller/recorder logs, and `unit-tests.txt` (183 tests passed).
