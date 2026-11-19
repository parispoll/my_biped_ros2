# Biped code map

The workspace remains `/home/parispoll/Documents/ChatGPT/robotics carrer`.
The active ROS package is `src/my_robot_description`.

For the calculated crouch milestone, see [Bent-knee stance](BENT_KNEE_STANCE.md).

For the first sensor integration, see [Pelvis IMU](PELVIS_IMU.md).

For crouch startup reliability and recording joint tracking, see
[the tracking investigation](CROUCH_TRACKING_DIAGNOSIS.md).

For foot contact sensor integration and its limits, see [Foot contacts](FOOT_CONTACTS.md).

For the first small-disturbance recordings, see [Push experiment](PUSH_EXPERIMENT.md).

For the separate finite-torque single-joint experiment, see [Torque bench](TORQUE_BENCH.md).

## What to edit

| Concern | File relative to the package |
|---|---|
| User-facing launch options | `launch/standing.launch.py` |
| Startup ordering, validation and failure handling | `my_robot_description/startup.py` |
| Expanding Xacro and temporary YAML workaround | `my_robot_description/description.py` |
| Starting Gazebo, creating the robot, bridging time | `my_robot_description/simulation.py` |
| Starting an individual controller | `my_robot_description/controllers.py` |
| Dimensions, masses and passive dynamics | `urdf/components/properties.xacro` |
| Foot contact sensor definitions | `urdf/components/foot_contacts.xacro` |
| Raw ground-contact interpretation | `my_robot_description/contacts.py` |
| Pelvis IMU definition | `urdf/components/imu.xacro` |
| ROS sensor bridges | `my_robot_description/sensors.py` |
| Robot colours | `urdf/components/materials.xacro` |
| Central body geometry and mass distribution | `urdf/components/base.xacro` |
| Leg geometry, joints and foot contact settings | `urdf/components/leg.xacro` |
| Joint command and measurement interfaces | `urdf/components/control.xacro` |
| Gazebo control plugin and configuration path | `urdf/components/gazebo.xacro` |
| Composition of the complete robot | `urdf/my_biped.urdf.xacro` |
| Small-push settings and response metrics | `my_robot_description/disturbance.py` |
| Controller reference/measurement recording | `my_robot_description/tracking.py` |
| Standing geometry and COM calculation | `my_robot_description/stance.py` |
| Monitored crouch motion | `scripts/move_to_stance.py` |
| Controller types, joint list and update rate | `config/controllers.yaml` |
| Gravity, physics systems, ground and lighting | `worlds/standing.sdf` |

The Python folder contains launch helpers, not robot balance control. Future feedback
controllers should be separate nodes/packages with their own inputs and outputs.

## Startup contract

1. Validate launch options and expand the robot description.
2. Start Gazebo, the robot-state publisher, the clock bridge, and robot creation.
3. Successful creation starts joint-state broadcaster activation.
4. Successful broadcaster activation starts trajectory-controller activation.
5. Successful trajectory activation prints `[biped:ready] Both controllers are active.`

An unsuccessful creation or controller process stops launch instead of proceeding.
Exit of an essential long-running process also stops launch. Expected shutdown
exits do not trigger new failures. This checks process lifetime and activation;
it does not continuously prove sensor freshness or robot stability.

| Last stage / symptom | First place to inspect |
|---|---|
| `[description]` exception | Xacro syntax, include paths, controller YAML file |
| Gazebo exits or plugin cannot load | `simulation.py`, `gazebo.xacro`, installed libraries |
| `[biped:spawn]` failure | World/model names, description topic, Gazebo UserCommands system |
| `[biped:joint-states]` failure | Broadcaster registration and controller-manager availability |
| `[biped:trajectory]` failure | Joint names and interface agreement between YAML and Xacro |
| Ready, but robot falls | Geometry, initial pose, contacts, mass distribution and control |
| Simulation time stops | Gazebo physics, pause state, `/clock` bridge |

## Commands

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
export ROS_DOMAIN_ID=42
export GZ_PARTITION=biped_recovery
ros2 launch my_robot_description standing.launch.py gui:=true
```

Optional arguments: `spawn_height:=0.57`, `controller_timeout:=60` (seconds).
The default is headless (`gui:=false`). Use matching domain and partition in all terminals.
World and model names are deliberately fixed for this baseline; changes must also
update `simulation.py` and the pose checker.

In a second terminal, after sourcing and setting the same environment:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /clock --once
timeout 95 python3 scripts/check_standing.py
```

`[biped:ready]` means controllers activated. Only the pose check establishes the
specific measured standing criteria. It currently checks the straight, undisturbed
position-controlled stance. It does not verify disturbance recovery or real motor limits.

## Checks before and after a change

```bash
python3 -m unittest discover -s tests
xacro src/my_robot_description/urdf/my_biped.urdf.xacro > /tmp/biped.urdf
```

Build after changing package metadata, dependencies, or adding installed files.
Change one physical assumption at a time and record the expected effect before testing.
The refactor comparison found the expanded robot unchanged (ignoring formatting and
comments). Unused inertia macros and `hip_rad` were removed. Legacy contact settings
were retained so this refactor did not silently change the physical model.

Known inherited limitations: legacy appearance/contact options need backend verification;
base centre of mass and shin inertia are approximations; root-link inertia produces a
KDL warning; actuator-limit enforcement and lateral balance remain unverified.

## Refactor verification, 2026-10-26

- Package build passed.
- Expanded robot XML matched the pre-refactor model after removing comments/whitespace.
- Four startup success/failure/shutdown checks passed.
- Both controllers activated in the refactored launch.
- Standing check: 1,766 samples over 30.007 simulated seconds; minimum base height
  0.564999 m, maximum measured tilt 0 rad.
- The bounded simulation shut down cleanly at the end of verification.

Evidence: `evidence/refactor-checks.json`, `evidence/refactor-launch.log`, and
`evidence/refactor-standing-check.json` at the workspace root.


## Effort experiment startup gate (updated)

The effort runner starts hardware-interface inspection and the recorder during
controller warmup. The recorder connects/subscribes early but waits for a
ready-file signal. Controller/balance readiness and optional settling open
that gate, followed by the recorder's existing fresh-baseline window and push.
Passive IMU discovery has a separate initial deadline; invalid joint samples
during pre-readiness startup are discarded without new commands and restart
settling. After readiness, timing violations still stop the run. See
[the startup fix guide](IMU_DISCOVERY_FIX.md) for code, invariants and evidence.
