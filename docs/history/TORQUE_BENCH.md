# Step 5a: one joint with finite torque

The biped still uses its working position controller. This step adds an independent
fixed-base pendulum bench to establish effort-control behaviour before changing
whole-body standing.

## What is different from position control?

The old interface receives an angle target. The new bench interface receives a
torque, in newton-metres (N·m). That is different from the push experiment's force,
measured in newtons (N).

The bench joint must accelerate according to torque, gravity, inertia and damping:

```
I*q_ddot = tau_motor - m*g*l*sin(q) - b*q_dot
```

Here `l` is the distance from hinge to centre of mass. The angle is zero when the
rod hangs vertically; positive rotation is about +Y.

The rod has mass 1 kg, length 0.26 m and radius 0.03325 m, derived from the current
shin dimensions. It is a uniform cylinder with a consistent inertia tensor; it is
not an exact isolated copy of the biped knee, shin and foot assembly. Bench damping
is deliberately 0.05 N·m·s/rad, travel is ±0.8 rad, speed limit is 6 rad/s and the
hard effort limit is ±2 N·m. These are experiment parameters, not selected humanoid
motor specifications.

## Code changes

All package paths below are relative to `src/my_robot_description`.

| File | Purpose |
|---|---|
| `urdf/torque_bench.urdf.xacro` (new) | Anchored base, uniform hanging rod, one revolute joint, effort interface and explicit effort/state limits |
| `config/torque_bench.yaml` (new) | Separate controller manager settings, command-limit enforcement and effort forwarder |
| `launch/torque_bench.launch.py` (new) | Starts the independent bench, with checked spawn/controller activation and cleanup |
| `my_robot_description/torque.py` (new) | Reads pendulum physical values from URDF, computes gravity/dynamics, and clamps requested motor torque |
| `package.xml` | Declares `forward_command_controller` and `std_msgs` dependencies, already installed on this machine |
| Workspace `scripts/torque_bench_experiment.py` (new) | Runs one test, commands torque, records joint data and checks the result against the model |
| Workspace `scripts/run_torque_bench.py` (new) | Owns a fresh isolated simulator per case, with timeouts, zero-command cleanup and simulator shutdown |
| Workspace `scripts/plot_torque_bench.py` (new) | Plots torque limits, position-holding comparison and travel-stop results |
| Workspace `tests/test_torque.py` (new) | Checks torque clamping, sign, gravity compensation, inertia and separation from biped position control |
| Workspace `tests/test_disturbance.py` | Updates an obsolete test value: after your allowed push limit became 100 N, 100 N is no longer an invalid request; the test now uses 10,000 N |

The user's 100 N push allowance and ten-second recovery setting are preserved.
A hash comparison checks the existing biped model, controller YAML, standing launch,
startup, description helper and push configuration have not changed in this step.

## Read these parts first

### 1. Effort interface in the new URDF

```xml
<command_interface name="effort">
  <param name="min">-2.0</param>
  <param name="max">2.0</param>
</command_interface>
```

The physical joint also declares:

```xml
<limit lower="-0.8" upper="0.8" effort="2.0" velocity="6.0"/>
```

These declarations alone would not establish enforcement. We enable it and test
the motion produced by commands outside the declared torque range.

The state interfaces expose position, velocity and effort. The reported effort is
a joint-transmitted-wrench projection from Gazebo's plugin; we do not assume it is
an independently calibrated motor torque sensor.

### 2. Separate controller configuration

```yaml
controller_manager:
  ros__parameters:
    update_rate: 250
    enforce_command_limits: true
    bench_effort_controller:
      type: forward_command_controller/ForwardCommandController
bench_effort_controller:
  ros__parameters:
    joints: [bench_joint]
    interface_name: effort
```

The forwarder does not calculate position feedback. It forwards the input torque
command to an effort interface. The bench experiment explicitly queries the
controller manager to ensure enforcement is enabled before applying commands.
Startup logs also show the position, velocity and effort limits were loaded.

### 3. Torque-based position holding

The Python experiment computes:

```
tau_requested = Kp*(q_target-q) - Kd*q_dot + m*g*l*sin(q)
tau_command   = clamp(tau_requested, -software_limit, +software_limit)
```

- The proportional term corrects angle error.
- The derivative term damps angular motion.
- The gravity term supplies the known static holding requirement.
- Clamping limits strength: requesting a position does not override that limit.

For this bench Kp=12 N·m/rad and Kd=0.5 N·m·s/rad. The normal holding test uses a
2 N·m software allowance. The restricted test changes only that software cap to
0.25 N·m. Both still have the same enforced 2 N·m hardware-interface ceiling.

These are simulated feedback experiments, not a production motor driver. The Python
client is not hard real-time and no electrical motor, gearbox, thermal model or
speed-dependent torque curve is represented. The controller manager requests 250 Hz;
the recorded command/feedback loop in the headless tests ran about 171–175 Hz.
All sample timestamps are saved, rather than claiming an exact 250 Hz Python loop.

## Tests and results

Seven fresh headless cases passed:

| Case | Observation |
|---|---|
| Positive torque pulse | Raw +4 N·m request; dynamics imply about +2.008 N·m |
| Negative torque pulse | Raw −4 N·m request; dynamics imply about −2.010 N·m |
| Positive stop | Joint reached +0.8 rad and did not pass it beyond numerical roundoff |
| Negative stop | Joint reached −0.8 rad and did not pass it beyond numerical roundoff |
| Position hold, 2 N·m allowance | Reached and held 0.5 rad, using about 0.612 N·m |
| Position hold, 0.25 N·m cap | Settled at 0.197319 rad, below the 0.5 rad target |
| Gravity release | Returned toward vertical after torque was removed from the 0.3 rad pose |

The pulse cases deliberately bypass the Python soft clamp and request ±4 N·m to
test the independent enforced cap. Short central-difference velocity windows,
away from command edges and joint stops, estimate torque via:

```
tau_inferred = I*q_ddot + m*g*l*sin(q) + b*q_dot
```

The estimate is model-based and affected by finite sampling/command delay. It is
not a precision torque-sensor calibration. Both signs agreeing with ±2 N·m rather
than ±4 N·m provide evidence that the declared cap affects actual motion.

### Why the weak controller stops short

At the requested 0.5 rad angle, gravity requires:

```
tau_gravity = 1 * 9.81 * 0.13 * sin(0.5) ≈ 0.6114 N·m
```

A 0.25 N·m cap cannot supply that torque. Its static equilibrium is predicted by:

```
q_equilibrium = asin(0.25 / (1 * 9.81 * 0.13))
              ≈ 0.197310 rad
```

The measured 0.197319 rad agrees closely. This is the most important result: a
finite actuator cannot reach every requested pose, even with a position-feedback
controller. Increasing gains cannot remove a fundamental torque shortage.

![Single-joint torque bench results](../evidence/torque-bench/trials/response.png)

## Run the visible comparison

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
python3 scripts/run_torque_bench.py --gui --cases hold hold-limited \
  --output evidence/my-torque-demo
```

This shows a pendulum, not the humanoid. First it moves toward 0.5 rad with sufficient
torque; the second fresh simulation shows the same target with the weaker cap.
Each test shuts down its simulator when finished. Choose a new output folder each
time. The runner uses its own ROS domain 55 and unique Gazebo partition; it does not
change or stop an existing humanoid simulation.

For all seven tests and a plot:

```bash
python3 scripts/run_torque_bench.py --output evidence/my-full-torque-test
python3 scripts/plot_torque_bench.py evidence/my-full-torque-test
```

A visible idle bench can also be launched with `torque_bench.launch.py gui:=true`
in its own domain/partition. Without a running torque client it is not a
position-controlled joint. The stock forward controller retains the last torque
command if a publisher disappears; the test harness zeros commands and stops its
own simulator. A future robot effort driver needs an appropriate watchdog and
real-time control design before being used beyond this isolated bench.

## Evidence and next step

Raw JSONL and summaries are in `evidence/torque-bench/trials/<case>/recording/`.
The root summary aggregates the seven cases. Preservation hashes are saved alongside
this folder. The normal biped `standing.launch.py` still selects position interfaces.

Next we can derive the six-joint gravity/torque requirements and design an explicit
effort-control standing controller as a separate mode. Single-joint success does
not establish whole-body stability: the floating base and contacts add coupling.
We will retain the existing position baseline for comparison.

References:
- Effort forwarding: https://control.ros.org/jazzy/doc/ros2_controllers/effort_controllers/doc/userdoc.html
- Gazebo effort interface: https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html
- Command-limit enforcement: https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html

## Final verification, 2026-10-27

All 46 automated checks passed. Seven fresh headless cases passed, including
recorded angle and speed limits, both torque directions, normal/restricted holding
and the unforced gravity model. The GUI holding case also passed; its recording
is in `evidence/torque-bench/gui-hold`. Existing biped/push preservation hashes
matched in `evidence/torque-bench/biped-preservation.json`. The tests shut down
their own simulators afterward.
