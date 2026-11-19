# Step 2: a calculated bent-knee standing pose

This step uses the original physical model and position interfaces. It adds a
calculated symmetric crouch, a slow monitored transition, and separate body-pose
verification. It does not implement active body-balance feedback or torque control.

## Run it

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
export ROS_DOMAIN_ID=42
export GZ_PARTITION=biped_recovery
ros2 launch my_robot_description standing.launch.py gui:=true stance:=crouch
```

The robot starts nearly straight with 0.02 rad knee clearance to avoid lower-stop lock-up.
See [the tracking investigation](CROUCH_TRACKING_DIAGNOSIS.md). After both controllers activate, it moves to the crouch
in about eight simulated seconds. Wait for `[biped:standing]` in the terminal.
The default launch without `stance:=crouch` still holds the straight stance.

Before launching, you can inspect the calculation without moving anything:

```bash
ros2 run my_robot_description plan_stance.py --knee 0.4
```

The launch accepts `knee_bend:=0.4` and `transition_duration:=8.0`.
Only the default 0.4 rad bend has been validated in Gazebo in this milestone.
Keep the existing default until we deliberately test another value.

In another terminal with the same sourced environment, ROS domain and Gazebo partition:

```bash
python3 scripts/check_standing.py \
  --expected-height 0.5544868763 \
  --height-tolerance 0.005 \
  --max-tilt 0.05
```

Run this after the crouch completes. It observes 30 simulated seconds, with a
wall-clock deadline even if no pose data arrive. It checks height within 5 mm of
the prediction and total body tilt below 0.05 rad (about 2.9 degrees).
The height prediction above is specific to the default model and knee bend.

## Why these angles?

Angles rotate around +Y. With upright pelvis and parallel joint axes:

```
foot pitch = hip + knee + ankle
```

We select knee = 0.4 rad. A horizontal foot then requires:

```
ankle = -hip - knee
```

This leaves the hip angle to solve. We choose it so the combined centre of mass
(COM) projects onto the centre of the feet, accounting for the heavy base and
every segment's declared mass. Total mass is 14.1 kg.

For thigh length L1, shin length L2, hip angle h and knee angle k,
with X measured relative to the pelvis:

```
thigh_COM_x = -(L1/2) sin(h)
knee_x      = -L1 sin(h)
shin_COM_x  = knee_x - (L2/2) sin(h+k)
foot_x      = knee_x - L2 sin(h+k)
```

The feet are horizontal and centred on their ankle frames in X, so foot_COM_x
is foot_x. The base COM has X=0 when the pelvis is upright. With identical legs:

```
M = m_base + 2(m_thigh + m_shin + m_foot)

COM_x = 2(m_thigh*thigh_COM_x + m_shin*shin_COM_x + m_foot*foot_x)/M
```

We solve COM_x - foot_x = 0. Define:

```
A = L1(m_thigh/2 + m_shin + m_foot)
B = L2(m_shin/2 + m_foot)
C = L1 - 2A/M
D = L2 - 2B/M
```

Then:

```
C sin(h) + D sin(h+k) = 0
h = atan2(-D sin(k), C + D cos(k))
```

For the current robot:

| Joint | Radians | Approximate degrees |
|---|---:|---:|
| Hip | -0.2240797 | -12.84 |
| Knee | +0.4000000 | +22.92 |
| Ankle | -0.1759203 | -10.08 |

These add to zero. Both legs receive the same angles.

Predicted pelvis height above the ground is:

```
z = L1 cos(h) + L2 cos(h+k) + foot_thickness
  = 0.5544868763 m
```

The COM and foot centres are both 0.0122707 m ahead of the pelvis. Thus, the
pelvis need not be directly over the foot centres: the whole robot's mass matters.
The predicted fore/aft static margin is 0.10 m to either foot edge. This is a
geometric static result, not a measured dynamic stability or disturbance margin.

## Where the new code lives

All paths below are relative to `src/my_robot_description`:

| File | Responsibility |
|---|---|
| `my_robot_description/stance.py` | Read dimensions/masses from expanded URDF, validate assumptions, solve stance, predict geometry |
| `scripts/plan_stance.py` | Print the calculation without starting Gazebo |
| `scripts/move_to_stance.py` | Read initial joints, construct trajectory, send action, check result |
| `my_robot_description/startup.py` | Run stance motion only after controller activation; stop launch on failure |
| `launch/standing.launch.py` | Expose straight/crouch and transition options |

The workspace-level `scripts/check_standing.py` independently observes body pose.
Tests are in `tests/test_stance.py` and `tests/test_pose_measurement.py`.

## Why an action instead of repeatedly publishing a pose?

The FollowJointTrajectory action reports acceptance, success, cancellation and
failure. The client sets explicit position tracking tolerances, checks the result,
and cancels on a wall-clock timeout. It does not repeatedly replace the trajectory.

The motion samples 81 calculated poses along a quintic knee-angle schedule.
Each waypoint has flat feet and predicted COM over the foot centres in the static
upright model. Position-only waypoints are interpolated by the controller; this
is an approximate slow/quasi-static transition, not a full dynamic balance plan.

## Assumptions we deliberately check

The calculation rejects changed joint axes, asymmetric leg properties, unsupported
joint connections, extra links/joints, invalid angles and joint-limit violations.
It assumes an upright pelvis, symmetric legs, centred foot boxes, and the current
link COM locations. A future torso, arm or lateral joint requires extending the
calculation rather than silently reusing these equations.

Tiny measured knee angles within 1e-6 rad of zero are rounded to zero when reading
the starting state. This handles physics roundoff at the stop; requested target
angles still undergo normal limit checks.

## What the result does and does not establish

Controller action success demonstrates joint tracking within configured tolerances.
The separate pose check demonstrates the measured body stayed above ground, near
predicted height, and below the tilt threshold for the observation period.
Neither proves recovery from a push, realistic torque limits, foot contact forces,
or walking. Those need their own sensors, controllers and experiments.

Reference for the monitored action interface:
https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html

## Recorded result, 2026-10-26

The monitored eight-second transition completed successfully. The separate settled
pose check passed over 30.008 simulated seconds (1,766 samples):

- Predicted pelvis height: 0.5544868763 m.
- Measured pelvis height: 0.5544877719 m.
- Maximum measured tilt: approximately 0 rad.

This close agreement is specific to the simplified ideal position-controlled model.
It is not a precision claim about real robot hardware. The pose check started after
the transition; it does not quantify transient tilt during the motion.

Evidence is in `evidence/crouch-prediction.json`, `evidence/crouch-launch.log`,
and `evidence/crouch-standing-check.json`. Fourteen automated tests passed.
The test simulator was stopped after verification.
