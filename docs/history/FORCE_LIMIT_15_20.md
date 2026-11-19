# Force escalation: 15 and 20 N

All four measured pushes passed. Both directions at 15 and 20 N used 0.20-second requested pulses, a fresh no-push reference, and a fresh push world per case. The robot package source hashes match the preceding 3/5/10 N study and remained unchanged during this continuation.

| Case | Peak pitch change | Last outside bands after clear | Saturated motor-command samples |
| --- | ---: | ---: | ---: |
| forward-15N | 4.683° | 3.036 s | 0 |
| backward-15N | 5.046° | 3.306 s | 0 |
| forward-20N | 6.767° | 3.443 s | 16 |
| backward-20N | 7.119° | 3.454 s | 17 |

All references passed. All four push recordings retained fresh ground-plane collision evidence for both feet, passed the existing timing/damping/balance audits, and confirmed Gazebo handling of queued and cleared force commands. Collision evidence is not measured support force. Force-command windows and impulses are not independently calibrated applied force measurements.

## What we learned

At 20 N the total ankle motor commands first reached the existing 6 N m motor cap in this sequence: 16 controller samples forward and 17 backward. This is separate from the smaller 0.50 N m per-ankle IMU balance-correction cap, which already clipped at 10 N. The robot still met the existing final return-band and contact criteria.

No controller gains, balance caps, motor caps, robot geometry or physics settings were changed. These single trials show successful examples through ±20 N at 0.20 seconds, not a proven recovery boundary. The largest pitch response, 7.12 degrees, approaches the stationary controller’s 0.15-rad (8.59-degree) tilt guard. Exceeding that guard would yield an incomplete guarded trial, not a completed measurement of falling or recovery.

The next useful check is to repeat ±20 N before extending the range or tuning control. Brief saturation is evidence to inspect, not proof that a larger cap or gain improves performance.

## Startup timeout and explicit retry

The first attempt is preserved at `evidence/force-limit-study-15-20-01`. Its reference passed, but its push world reached posture readiness only near the runner’s 100-second startup deadline. It was stopped before the balance-ready marker, baseline recording, or force publication. It is excluded from physical recovery results.

`scripts/run_effort_push.py` now allows 150 wall seconds for PID startup instead of 100. The fixed readiness thresholds, balance ramp, source freshness guards and measurement windows are unchanged. Recorder startup allowance follows this budget. `scripts/effort_push_experiment.py` now uses `rclpy.try_shutdown()` so an interrupt does not produce a misleading second shutdown exception. The fresh retry uses a new folder and all four cases passed.

## File changes

- `scripts/run_force_limit_study.py`: added ordered `--forces` input; default 3/5/10 N preserved; duplicate/descending escalation is rejected.
- `scripts/plot_force_limit_study.py`: reports actual recovered-case counts instead of a fixed six-case label.
- `scripts/run_effort_push.py`: PID startup wall allowance 100 → 150 seconds.
- `scripts/effort_push_experiment.py`: idempotent ROS cleanup.
- `tests/test_force_limit_study.py`: verifies invalid escalation is rejected before creating output.

## Reproduce

```bash
cd '/home/parispoll/Documents/ChatGPT/robotics carrer'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/run_force_limit_study.py --forces 15 20 --ros-domain-id 68 --output evidence/force-limit-study-15-20-03
python3 scripts/plot_force_limit_study.py evidence/force-limit-study-15-20-03
```

Use a new output folder each time. Stop-on-first-nonpass remains enabled. The suite passed 184 tests after this continuation. Results, per-case diagnoses, original logs and comparison chart are in `evidence/force-limit-study-15-20-02`.
