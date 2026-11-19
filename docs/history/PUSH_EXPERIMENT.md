# Step 4: small controlled push and baseline recording

This experiment measures the existing position-controlled robot's response. It
adds no balance-feedback controller and changes no joint gains, limits, mass,
geometry, crouch target, or tracking tolerance.

## Experiment design

Each trial starts a fresh isolated simulator, waits for the verified crouch, then
records approximately:

1. Two simulated seconds of baseline.
2. A requested +X force of 5 N for 0.2 simulated seconds.
3. Five simulated seconds after the force-clear command.

There is also a no-push control trial using 0 N with the same command sequence.
Two identical 5 N trials were recorded for comparison.

The world X direction is forward in our initial robot orientation. The target is
model `my_biped`; Gazebo applies the wrench to its canonical pelvis link,
`base_link`. No explicit torque or force offset is requested.

The script bounds requests to |force| <= 10 N and durations 0.02–0.5 s for this
milestone. A force is published once as a persistent wrench, then explicitly
cleared; repeatedly publishing it would accumulate persistent wrenches in Gazebo.
The final cleanup also clears it on exceptions or interruption. The owning runner
stops its simulator even when recording fails.

### Timing precision

Force switching uses advancing simulation time. The measured command windows in
the main trials were 0.201 s. Transport and simulator scheduling add unmeasured
delay between publication and application. These are requested command windows,
not independently measured force durations or calibrated delivered impulses.
The nominal command impulse is approximately 1 N·s; it is not a force-sensor result.

## Recorded signals

`samples.jsonl` stores snapshots capped at approximately 100 Hz (the achieved rate can be lower under GUI load):

- IMU pitch, pitch rate and horizontal acceleration.
- Independent Gazebo model height and X position.
- Desired and measured joint angles, plus controller tracking errors.
- Latest foot/ground contact evidence and its age for both feet.
- Individual source timestamps and the experiment phase.

`events.json` records the simulation/wall times when recording, push publication
and clear publication occurred. `summary.json` stores metrics and requested settings.

Sensor validity, frame, timestamps and freshness are checked. Initial subscription
acquisition must remain fresh and settled for 0.5 simulated seconds before the
two-second experimental baseline begins. This avoids arming on transient discovery
gaps; it does not relax the freshness thresholds during recording. Both feet must have
fresh ground evidence immediately before publication. Data older than 50 ms are
labelled `null`/unknown rather than called liftoff. IMU/joint/body data stale by more
than 100 ms stop the experiment, as do excessive body tilt or reduced height.

These checks protect the experiment's interpretation. They are not a substitute
for a future real-time sensor-health policy in a balance controller.

## What changed

Paths are relative to `src/my_robot_description` unless specified otherwise.

| File | Change and responsibility |
|---|---|
| `worlds/standing.sdf` | Adds Gazebo's built-in ApplyLinkWrench world system; it applies no force unless explicitly commanded |
| `my_robot_description/disturbance.py` (new) | Validates bounded experiment settings and summarizes the recorded response |
| Workspace `scripts/push_experiment.py` (new) | Collects ROS sensor/controller data and Gazebo body pose, publishes/clears the requested force, saves the recordings |
| Workspace `scripts/run_push_trials.py` (new) | Owns fresh isolated simulations for the control and push trials, with deadlines and automatic shutdown |
| Workspace `scripts/plot_push_trials.py` (new) | Plots the saved command, pitch, joint-error and contact-freshness traces |
| Workspace `tests/test_disturbance.py` (new) | Checks bounded requests, ROS/Gazebo timestamp field differences, baseline subtraction, return-band interpretation and unknown-contact handling |

The only change to the running robot/simulator configuration is:

```xml
<plugin filename="gz-sim-apply-link-wrench-system"
        name="gz::sim::systems::ApplyLinkWrench">
  <verbose>true</verbose>
</plugin>
```

It exposes Gazebo transport topics under `/world/standing/wrench`. Python uses the
installed `gz.transport13` and `gz.msgs10` bindings directly; no additional ROS
command bridge or controller is introduced. On this machine those bindings are
provided by the installed Gazebo Python packages.

One integration correction during development was distinguishing ROS time's
`nanosec` field from Gazebo protobuf time's `nsec` field. A test now covers both.
The unsuccessful recorder attempt is retained in `evidence/push-timestamp-diagnostic`.
A later pre-push sensor freshness abort is retained in `evidence/push-final-smoke`;
it motivated the continuous acquisition check described above, not relaxed limits.
No push occurred in that attempt because it never obtained the required body data.

## Reproduce the test

From the workspace root, after building and sourcing:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
python3 scripts/run_push_trials.py --output evidence/my-push-trials
python3 scripts/plot_push_trials.py evidence/my-push-trials
```

Choose a new output directory each time. The runner launches one no-push control
and two 5 N trials sequentially, with separate Gazebo partitions. It uses ROS domain
190 for its own simulations and does not stop or push an existing user simulation.
Do not run multiple trial runners concurrently with that same domain.

Each trial contains `launch.log`, `experiment.log`, and a `recording/` directory.
The root `summary.json` combines the trials; `response.png` is the plot.

For future manual GUI experiments, `push_experiment.py` can target a running
simulation with matching ROS domain and Gazebo partition, but the owning runner is
preferred: a hard kill of a standalone publisher cannot execute its cleanup, whereas
the runner also shuts down the simulation it owns.

## Results from the main trials

| Trial | Requested force | Recorded command window | Peak pitch change |
|---|---:|---:|---:|
| No-push control | 0 N | 0.201 s | ~1.8e-14 rad |
| Push 1 | 5 N | 0.201 s | ~9.79e-8 rad |
| Push 2 | 5 N | 0.201 s | ~9.85e-8 rad |

The two pushes produced similar responses. Their peak pitch changes are about
0.000006 degrees, far below a substantial tipping motion. Maximum joint tracking
error in the push trials was approximately 1.81e-11 rad. The model stayed near its
predicted 0.55449 m pelvis height and remained within the specified final-second
return band (0.5 degrees pitch, 0.02 rad/s pitch rate, 5 mm height difference).

Crucially, the response never meaningfully left that band. We cannot interpret
this as a demonstrated active-balance recovery time. The current model has stiff
ideal position tracking and broad flat feet, and this small load can be resisted
without appreciable tipping. This is a measured small-disturbance baseline, not
proof of realistic motor capability or recovery from a larger disturbance.

The recordings show no fresh non-ground foot-contact samples. They do contain brief
pre-push contact-data gaps, up to approximately 97 ms, labelled unknown. Those gaps
are visible in the contact-age plot and are not evidence of foot liftoff. A future
feedback experiment must continue distinguishing unavailable evidence from physical
loss of support.

![Small-push baseline recordings](../evidence/push-baseline/response.png)

## Appropriate next work

Before tuning a body-balance controller, define how much motion the experiment needs
to excite and how realistic the actuators should be. The present 5 N trial barely
moves the model. A useful next discussion is whether to test progressively larger
bounded disturbances or first move to validated torque-limited actuation. Either
change deserves its own experiment; neither was silently added here.

References:
- https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1ApplyLinkWrench.html
- https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/apply_link_wrench/ApplyLinkWrench.cc

## Final verification with fresh-data acquisition

After adding continuous fresh-data arming, an additional control and 5 N trial both
completed. The push pitch change was 9.82e-8 rad, consistent with the two main push
trials. Neither final recording had unknown or fresh non-ground contact samples.
All 37 automated tests passed. Every test simulator was shut down after its trial.

Final evidence: `evidence/push-final-verified/summary.json`, per-trial raw JSONL and
event files, and `evidence/push-final-verified/response.png`.

![Final verified control and small push](../evidence/push-final-verified/response.png)

## GUI recorder callback correction, 2026-10-27

A manual GUI attempt in `evidence/visible-push-01` aborted before the push: the
recorder compared a newer clock value against IMU/joint updates that could still
be waiting for callback dispatch. The last ages were IMU 105 ms, joints 99 ms,
body pose 82 ms. The 100 ms limit was correctly enforced, but the recorder had
processed only one callback before evaluating a multi-topic snapshot.

The correction keeps a SingleThreadedExecutor attached to the node for the run
and processes one waiting callback followed by up to 32 already-ready callbacks
with zero waiting time before evaluating the data. This reduces callback-order
artifacts and avoids repeatedly adding/removing the node through the global
spin_once helper. The batch is bounded so force clearing and deadlines still run.
It does not fabricate timestamps or raise the 100 ms freshness limit. Genuine
staleness after processing available callbacks still aborts and clears the force.

Changes: `scripts/push_experiment.py`, the `process_sensor_callbacks` helper in
`my_robot_description/disturbance.py`, callback-order/bounded-work tests in
`tests/test_disturbance.py`, and an optional `--gui` flag in
`scripts/run_push_trials.py`. The experiment now prints stage messages as it runs.

All 39 automated tests passed. A separate GUI control and 5 N push trial both
completed; evidence is in `evidence/push-gui-callback-fix`. This validates the
correction on those GUI trials, not every possible machine-load condition.
Your existing GUI simulation does not need restarting for this recorder change.
Use a new output directory when retrying.
