# Repeated 25 N push verification

Four fresh measured trials used a 25 N requested pulse for 0.20 simulated seconds: forward/backward, then backward/forward. All four no-push references passed. All four pushes were queued and cleared according to Gazebo debug logs. All four push worlds then stopped at the existing pelvis tilt guard (0.15 rad, about 8.59 degrees).

These are repeatable guard-stopped partial trials, not completed measurements of falling or eventual recovery. The controller stops and releases its commands when the guard trips; measured launch then closes the world. Visual mode has different continuation behavior and cannot replace a measured trial.

| Direction | Repeat 1 | Repeat 2 | Published command windows |
| --- | --- | --- | --- |
| Forward | Tilt guard stopped trial | Tilt guard stopped trial | 0.201 s, 0.201 s |
| Backward | Tilt guard stopped trial | Tilt guard stopped trial | 0.201 s, 0.202 s |

Both ankles reached their existing 6 N m command cap in every partial response. Recorded samples retained fresh ground-contact evidence up to their cutoffs. This is collision evidence, not measured support force. Short recordings and asynchronous callbacks can include samples beyond the guard crossing; their maximum recorded pitch is not a measured full-response peak.

Robot package source hashes remained unchanged and match the preceding 15/20 N study. Model, gains, balance cap, motor caps, sensor timing checks and physics settings were unchanged. Tests ran only after the measured study finished. All 187 tests passed.

## What this establishes

The existing measured controller recovered in the earlier single examples at ±20 N for 0.20 seconds. At ±25 N for 0.20 seconds the tilt guard stopped both repeats per direction. This brackets observed behavior of this guarded protocol, not the physical recovery boundary. Two repeats are a descriptive consistency check, not statistical certification. A longer pulse was not mixed into this check.

The next useful engineering step is to inspect ankle saturation and pitch-rate response before tuning. Raising the guard or motor cap would change the question; the current evidence does not justify calling either a controller improvement.

## Files added

- `scripts/run_force_repeat_check.py`: repeats one fixed pulse, reverses direction order for repeat 2, requires a passing reference and published push before classifying a posture-guard stop, stops on infrastructure errors.
- `scripts/summarize_force_repeat_check.py`: verifies the evidence and plots partial traces with explicit cutoffs.
- `tests/test_force_repeat_check.py`: checks that transport/startup failures cannot be mislabeled as physical guard stops.
- `scripts/start_force_panel_background.sh`: starts the loopback panel as a transient systemd user service so it survives terminal/chat closure.

## Reproduce

```bash
cd '/home/parispoll/Documents/ChatGPT/robotics carrer'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/run_force_repeat_check.py --force 25 --duration .2 --ros-domain-id 69 --output evidence/force-25N-repeats-02
python3 scripts/summarize_force_repeat_check.py evidence/force-25N-repeats-02
```

Choose a new output folder. Each repeat creates its own no-push reference and push world. All evidence from this run is under `evidence/force-25N-repeats-01`, including `repeat-study.json`, `assessment.json`, `repeat-response.png` and full logs.

## Panel repair

The server was stopped, so port 8765 refused connections. It is now running as `biped-force-panel.service` in the user service manager. The page loaded and ran visual pulses again. The server remains loopback-only and does not start automatically after reboot or logout. Close panel stops it deliberately. Reopen it using:

```bash
cd '/home/parispoll/Documents/ChatGPT/robotics carrer'
./scripts/start_force_panel_background.sh
```

Open http://127.0.0.1:8765/ and select Visual only. To inspect service logs use `journalctl --user -u biped-force-panel.service`; to stop it use the Close panel button or `systemctl --user stop biped-force-panel.service`. Existing active user visual runs were left untouched during the measured study.
