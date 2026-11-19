# Step 5c: hold the humanoid using joint torque

This mode starts directly in the calculated 0.4 rad crouch. It does not yet move
from straight standing into the crouch. It controls six joint angles with bounded
torques. It does not use pelvis tilt to calculate balancing torques.

## What changed in the robot's behavior?

Previously the position interface received an angle and Gazebo drove the joint
through its internal position/velocity mechanism. That was useful for checking
geometry, sensors and launch order. We now send a torque in N m. The joint angle
becomes a result of that torque, gravity, contact and the rest of the robot.

The old position mode remains available through `standing.launch.py`. This new
experiment uses `effort_standing.launch.py`. Both use the same masses, shapes,
inertias, joint axes, joint travel, damping, friction and sensors. A test compares
all physical link and joint XML between their expanded models.

Official interface behavior and the temporary unclaimed-joint hold are documented
by [gz_ros2_control for Jazzy](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html).

## The whole controller in one equation

```
error = target angle - measured angle
requested torque = holding torque + Kp × error - Kd × measured speed
command = limit(requested torque, -maximum torque, +maximum torque)
```

The equation is evaluated separately for every joint, whenever a joint-state
message arrives. The six resulting commands are sent together, in the order
left hip/knee/ankle, then right hip/knee/ankle.

Read the terms as four separate jobs:

1. **Holding torque (feedforward):** start with what the static calculation says
   should be needed. At knee bend 0.4, this is about -2.5221 N m at each knee,
   and approximately zero at the hip and ankle. It is calculated once for the
   target pose, not recomputed as if a moving robot were in static equilibrium.
2. **Angle correction:** if the knee bends too far, `target - measured` is negative,
   so this term adds an extension torque. Kp has units N m/rad.
3. **Speed correction (derivative damping):** oppose motion, rather than amplify
   it. Positive speed gives a negative correction. Kd has units N m s/rad.
4. **Torque cap:** stop the request exceeding this experiment's allowed range.
   Increasing an angle error cannot make the commanded torque grow without limit.

This is proportional-derivative joint control plus static feedforward. There is
no integral term. A small remaining position error is therefore possible.

### A worked knee example

Suppose the target is 0.400 rad, the measured angle is 0.410 rad, and the knee
is bending at +0.020 rad/s:

```
error        = 0.400 - 0.410 = -0.010 rad
holding      = -2.522112 N m
angle term   = 100 × (-0.010) = -1.000000 N m
speed term   = -0.15 × 0.020 = -0.003000 N m
request      = -2.522112 - 1.000000 - 0.003000
             = -3.525112 N m
command      = -3.525112 N m (inside the knee's ±10 N m cap)
```

If the request were -14 N m, the command would be -10 N m. The unclipped request
and the transmitted command are both recorded, so saturation is visible.

## Gains and limits for this experiment

| Joint, per leg | Kp (N m/rad) | Kd (N m s/rad) | Command cap (N m) |
| --- | ---: | ---: | ---: |
| Hip | 80 | 0.10 | ±6 |
| Knee | 100 | 0.15 | ±10 |
| Ankle | 60 | 0.05 | ±6 |

These are simulation experiment settings, not specifications for a real motor.
The existing physical URDF effort ratings remain 80/80/60 N m; the new effort
interfaces declare stricter 6/10/6 limits. Python reads those limits from the
expanded URDF instead of keeping a second controller-side list of caps.
Controller-manager command-limit enforcement is enabled and checked at startup.
Our independent torque bench established that this installed manager enforces
an effort cap. We do not infer actual motor torque from `joint_states.effort`.

The controller manager updates at 250 Hz. Python reacts to received joint
feedback; this is an educational ROS controller, not a hard real-time motor loop.
Existing passive joint damping remains part of the physics as well.

## New files, one responsibility each

| File | Responsibility |
| --- | --- |
| `urdf/effort_standing.urdf.xacro` | Separate entry point; composes the same geometry and sensors with effort interfaces. |
| `urdf/components/control_effort.xacro` | Declares six effort interfaces, their limits, and initial joint states. |
| `config/effort_standing.yaml` | Controller types, joint ordering, limit enforcement and PD gains. |
| `my_robot_description/posture_effort.py` | Pure torque equation, limit parsing and IMU tilt calculation; independently testable. |
| `scripts/hold_effort_stance.py` inside the package | Receives measurements, runs the equation, publishes commands, checks startup/failure conditions and records each term. |
| `launch/effort_standing.launch.py` | Wires the experiment together and orders startup/shutdown. |
| Workspace `scripts/run_effort_standing.py` | Runs an isolated simulator, checks pose/IMU/contacts/tracking and stops its own processes. |
| `tests/test_posture_effort.py` | Checks signs, clipping, invalid inputs, orientation and identical physical models. |

The package's `CMakeLists.txt` now installs the new executable, and `package.xml`
adds `controller_manager_msgs` for switching controllers. Documentation links are
updated. The original position launch/controller files and push settings are
unchanged.

### Before and after the interface

Old, still used by position mode:

```xml
<command_interface name="position"/>
```

New, in the separate effort component (the knee macro expands to this):

```xml
<command_interface name="effort">
  <param name="min">-10.0</param>
  <param name="max">10.0</param>
</command_interface>
```

A position command means "go to this angle". An effort command means "apply this
turning effort". The new Python code must therefore supply the angle feedback
that the position mode previously handled internally.

## Startup: avoid dropping the robot before control is ready

1. Calculate the crouch and initialize all six joint positions to it.
2. Spawn the robot with its flat soles at ground level, at the calculated height.
3. Activate the joint-state broadcaster.
4. Load/configure the effort controller, leaving it **inactive**.
5. Start the Python node and obtain joint/IMU feedback and a command subscriber.
6. Verify that controller-manager command limits are enabled.
7. Ask the manager to activate the effort controller, while publishing commands.
8. After two simulated seconds, require position errors below 0.03 rad and joint
   speeds below 0.05 rad/s before logging `[effort:ready]`.

Before the effort interfaces are claimed, Gazebo's default `hold_joints=true`
holds unclaimed joints. That temporary startup aid is explicitly part of this
sequence. The validated standing interval begins after effort activation. Saved
hardware-interface output shows the six **effort** interfaces claimed.

If the Python process exits, launch stops the simulator. Invalid measurements,
joint errors above 0.2 rad, joint speeds above 3 rad/s, pelvis tilt above 0.15 rad,
or joint/IMU feedback absent for one wall second also stop this experiment.
Tilt is only a shutdown check; it does not alter the torque equation. Pausing
Gazebo for more than a second will trigger the missing-feedback shutdown.
These process checks are not an independent actuator watchdog for physical hardware.

## An important failed test we kept

The first gains used Kd = 8, 8, 5. The robot remained upright, but joint commands
oscillated and repeatedly saturated. The IMU failed the stationary check. The
failed evidence remains in `evidence/effort-standing/initial-01`.

Reducing Kd to 0.10, 0.15, 0.05 removed that vibration in the following test, with
no relaxation of torque caps or the sensor checks. Large damping gains are not
always better: feedback delay, sampled commands, small inertias and the existing
physical damping all matter. The successful intermediate evidence is in
`evidence/effort-standing/tuned-02`.

## Run one checked experiment

Use a fresh terminal:

```bash
cd '/home/parispoll/Documents/ChatGPT/robotics carrer'
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
python3 scripts/run_effort_standing.py --gui --output evidence/my-first-effort-stand
```

The runner chooses its own ROS domain and Gazebo partition, records evidence,
observes for 30 simulated seconds, and closes that simulation. Use a new output
folder for each run; existing evidence is not overwritten. The pose, IMU and
both-contact checks run concurrently for the full observation duration.

For a simulation that stays open until Ctrl+C:

```bash
export ROS_DOMAIN_ID=56
export GZ_PARTITION=biped_effort_manual
export ROS_LOG_DIR="$PWD/log/runtime"
ros2 launch my_robot_description effort_standing.launch.py gui:=true
```

Wait for `[effort:ready]`. Small stationary differences from the commanded angles
are expected with finite stiffness. For a compatible small disturbance test, use
[the effort-mode push runner](EFFORT_PUSH.md); it reads the effort controller's
streams and compares its response against a fresh no-push reference.

## What to inspect in the recordings

`joints.jsonl` stores target and measured positions, velocity, angle error,
feedforward, proportional correction, damping correction, requested torque,
clipped command, activation flag, time and latest pelvis tilt. Torque units are
N m, angles radians, speeds rad/s, and timestamps simulation seconds.

The pure calculation is the best code entry point: read `posture_command` in
`posture_effort.py`, then trace one recorded knee command using the same addition.
The summary reports maximum settled tracking error, peak commands, saturation,
body pose, IMU measurements and ground-contact evidence. Requested torque is
not an independently measured actuator force, and the contact check proves
collision evidence rather than the exact pressure location used in our statics.

## Verified result

The 30-second check in `evidence/effort-standing/validated-30s` passed body-pose,
IMU, both-contact and joint-tracking checks. Maximum settled joint error was
0.0044625 rad (0.256 degrees), pelvis tilt approximately 0.104 degrees, and no
recorded active command saturated. The GUI run in `evidence/effort-standing/gui-10s`
passed the same checks for 10 seconds. All 56 Python tests passed.

The final knee angle was 0.4044625 rad rather than exactly 0.4 rad. Its command
shows the equation at work:

```
error = 0.4 - 0.4044625 = -0.0044625 rad
command ≈ -2.522112 + 100 × (-0.0044625) - 0.15 × 0
        ≈ -2.968362 N m
```

The static estimate is the starting term; angle feedback supplies the remaining
correction for this simulated equilibrium. The exact simulated contact-force
distribution and friction were not measured, so we do not attribute the whole
difference to a single cause.

![Torque terms and joint errors](../evidence/effort-standing/validated-30s/response.png)

Regenerate this plot with:

```bash
python3 scripts/plot_effort_standing.py evidence/effort-standing/validated-30s
```
