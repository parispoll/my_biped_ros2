# Step 5d: a small push during effort-controlled standing

We now have a measured disturbance experiment for the torque-controlled robot.
The first request is **1 N in world +X for 0.2 seconds**, followed by 10 seconds
of observation. The controller gains and torque caps are unchanged.

The important result: the robot stays standing, but settles with a small remaining
lean instead of returning within the specified bands. This is a completed
experiment with a failed recovery criterion, not a recording failure.

## 1. What happens when you run it

The runner creates two fresh simulations, one after the other:

1. **Reference:** the robot holds the crouch; a zero-force command passes through
   the same publication/clear sequence. This establishes the background motion.
2. **Push:** the same controller receives the 1 N disturbance. We compare its
   motion with its own pre-push baseline and with the no-push reference.

Each recording has these phases:

```
+------------------+----------+----------------------------------------+
| 2 s baseline     | 0.2 s    | 10 s observation after clear command  |
| measure rest     | request  | measure motion and remaining offset   |
+------------------+----------+----------------------------------------+
```

There is also a discovery/readiness period before this timeline. The recorder
requires fresh measurements, a settled pelvis, both feet touching the floor,
and Gazebo force/clear subscribers before it begins.

The command targets model `my_biped`, so Gazebo applies it to the canonical link
`base_link`. The force is expressed in world coordinates. We use the existing
ApplyLinkWrench persistent and clear topics. See the
[official Harmonic system documentation](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1ApplyLinkWrench.html).

## 2. Reproduce it in the GUI

Open a new terminal:

```bash
cd '/home/parispoll/Documents/ChatGPT/robotics carrer'
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash

python3 scripts/run_effort_push.py \
  --gui \
  --output evidence/my-first-effort-push
```

You do not need to open Gazebo separately. It opens for the reference, closes,
then opens for the push trial. The runner uses ROS domain 57 and a different
Gazebo partition for every simulation. Do not run another domain-57 experiment
at the same time. Leave the simulator unpaused; the controller stops on missing
feedback for one wall second.

The push is deliberately small. Expect a subtle movement rather than a dramatic
fall. The recorded graph makes it easier to see.

Generate that graph after both runs finish:

```bash
python3 scripts/plot_effort_push.py evidence/my-first-effort-push
```

Open `response.png` in that folder. Use a new folder name on every run. The new
experiment accepts forces up to ±2 N and durations 0.02–0.3 s. Those narrow bounds
belong to this first effort-mode experiment; the older position-mode force
settings have not been changed. No larger-force sweep has been validated here.

## 3. Read the results in the right order

Start with `summary.json` in the root output folder:

- `completed: true` means both trials ran and produced their measurements.
- `passed: true` would mean the push response was clearly above the reference,
  contacts stayed confirmed, and the final second met all recovery bands.
- `passed: false` with `completed: true` means the experiment ran, but a recovery
  criterion was not met. This is the observed result for the initial 1 N tests.
- An `error` field or a Python traceback indicates a setup, recording or baseline
  failure instead. Those should be investigated before interpreting recovery.

For each trial, `recording/summary.json` contains the detailed measurements.
`response_detected` alone is not proof of an independently calibrated force; it
means the observed motion exceeded a small detection threshold. The root result
also requires peak pitch change to exceed both 0.00001 rad and ten times the
reference peak change.

### Recovery means close to the original measured pose

The reference is the robot's actual settled pose before the push, rather than
assuming perfectly zero initial pitch or exactly the commanded joint angles.
Every sample during the final second must be within:

| Quantity | Required range |
| --- | --- |
| Pelvis pitch change | ±0.1 degree from baseline |
| Pelvis pitch rate | ±0.01 rad/s |
| Pelvis height change | ±2 mm from baseline |
| Each joint position change | ±0.002 rad (about 0.115 degree) from baseline |

The final second also needs adequate sample coverage, and all recorded contact
snapshots must have fresh ground-contact evidence for both feet. Missing contact
messages are unknown evidence, not proof that a foot lifted. Horizontal position
change is reported separately and is not a return-band condition.

These conditions are stricter than merely staying upright. They were set before
the first push run and were not widened after the robot failed them.

## 4. What the first tests showed

In the initial headless run, peak pitch change was about **0.49 degree**. In the
GUI repeat it was about **0.50 degree**. The ankle angles retained a corresponding
offset after the clear command. The pelvis moved horizontally by roughly 4.4 mm.
Both feet retained fresh ground-contact evidence.

The ankle torque command changed from approximately **+0.122 N m** to about
**+0.64 N m per ankle**. This is far below its ±6 N m cap. The knee command stayed
near -2.97 N m. Thus a remaining lean does not, by itself, mean a motor hit its
command limit.

The no-push reference had negligible motion. The GUI trial's Gazebo debug log
confirmed both the queued persistent wrench and its clearing. That distinguishes
this observation from an experiment where a push might accidentally remain on.
There is still no independent force sensor or exact actuation-duration measurement.

The current controller regulates joint angles. Pelvis tilt is used only as a
shutdown check, not as a balancing correction. The evidence establishes a
remaining equilibrium offset under this small disturbance. Contact distribution,
joint friction and the controller's finite stiffness may contribute; we have not
isolated their individual contributions, so the guide does not claim a single
proven cause.

## 5. Follow the code without getting lost

The force application/recording loop already existed for position-mode tests.
Rather than copy that loop into a second file, it now accepts two optional hooks:

```python
run(config, folder, joint_subscription=None, analyzer=summarize)
```

The defaults preserve its position-mode behavior. Effort mode supplies:

```python
run(config, folder,
    joint_subscription=attach_effort_stream,
    analyzer=summarize_effort)
```

The stream adapter reads `robot_description` and `knee_bend` from the running
posture node. It maps named `/joint_states` measurements into the six-joint order,
subscribes to `/standing_effort_controller/commands`, and checks those commands
against the effort-interface limits. It does not publish joint commands or
change the controller.

The analyzer computes baseline-relative motion, the tighter recovery decision,
command changes and contact quality. Keeping this calculation separate from ROS
lets us test false-positive recovery cases without running Gazebo.

### Added files

| File | Job |
| --- | --- |
| Workspace `scripts/run_effort_push.py` | Owns the two simulations, checks startup, saves evidence and stops its processes. |
| Workspace `scripts/effort_push_experiment.py` | Adapts the effort controller's joint/command streams to the shared recorder. |
| Package `my_robot_description/effort_push.py` | Experiment limits and recovery calculations. |
| Workspace `scripts/plot_effort_push.py` | Shows requested force, pelvis change, ankle change and torque commands. |
| `tests/test_effort_push.py` | Checks successful return, persistent offsets, missing contact evidence, sparse data and the zero-force reference. |

### Modified files

| File | Change |
| --- | --- |
| Workspace `scripts/push_experiment.py` | Optional stream/analyzer hooks; optional command freshness checks and recording. Its default position-mode path remains available. |
| Package `my_robot_description/simulation.py` | Optional Gazebo verbosity argument and live debug output at verbosity 4; default position-mode behavior is unchanged. |
| Package `launch/effort_standing.launch.py` | Exposes verbosity so this runner can capture force queue/clear debug messages. |
| README and effort-standing guide | Link this experiment and replace the earlier pending-step note. |

No robot masses, contact parameters, torque caps, gains, or existing position-mode
push bounds were changed for this experiment.

## 6. Understand what was measured

`recording/samples.jsonl` combines the latest fresh IMU, body pose, joint positions,
contact evidence and observed torque commands. These streams are asynchronous;
the row is a fresh snapshot rather than a precisely synchronized sensor packet.
Core IMU/joint/pose/command freshness must stay within 0.1 simulated second;
contact freshness is limited to 0.05 s.

Torque command messages have no timestamp. Their recorded `command_reception_clock`
is the most recent ROS clock when the callback executes, not a measured actuation
time. The separate `controller.jsonl` records the controller's feedforward, angle
correction, damping, unclipped request and bounded command at its joint-feedback
timestamps. The runner reports whether these recorded requests saturated.

`push_published` and `clear_published` are publication events. Their separation is
the **command window**, not a precisely measured force duration. Multiplying force
by that window gives a nominal command impulse, not measured physical impulse.
The Gazebo debug acknowledgement confirms queue/clear handling, with no force
calibration or exact simulation timestamp for when each took effect. The runner
requires the clear log to be observed while the recorder is still running and
within two wall seconds of clear publication. Debug runs use a pseudo-terminal
to prevent piped-output buffering. `clear_log_observation_lag_wall_s` includes
the runner’s 0.2 s polling interval and logging delay; it is not an actuation-delay
measurement.

## 7. Validation and compatibility

All 63 Python tests passed. The final headless recording is in
`evidence/effort-push/validated-1N`, with `completed: true`, `passed: false`,
and a measured pitch response clearly above the no-push reference. It reports
zero saturated controller samples and confirms Gazebo wrench queue/clear handling.
The final GUI run is in `evidence/effort-push/verified-1N`. It also confirms
`clear_observed_while_recording: true`: the runner observed the clear log about
0.106 wall second after publication, well before the observation window ended.
An earlier GUI repeat is in `evidence/effort-push/gui-confirmed-1N`.

The original position-mode zero-force recorder also completed successfully in
`evidence/effort-push/legacy-position-reference-alone`. A first compatibility
attempt made while another simulation ran stopped correctly on stale body-pose
data; that failed attempt is preserved separately. The 0.1 s freshness threshold
was not relaxed.

![Measured response](../evidence/effort-push/verified-1N/response.png)

## 8. What this tells us to do next

First inspect the residual ankle angle, body tilt and torque together. Then design
and test a correction for the remaining offset. Raising force, widening the return
band or increasing gains until the report turns green would skip that diagnosis.
This experiment provides the baseline needed to evaluate the next controller change.
