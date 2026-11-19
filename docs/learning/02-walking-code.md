# 2. Explain one walking step from the code

Read this alongside `src/my_robot_description/my_robot_description/motion/alternating_walk.py`.

## 1. Remember where both feet are

The planner stores world-frame XYZ positions for left and right feet. After landing, the new position replaces the old one. It never restarts the original symmetric stance while a forward foot is on the floor.

The first step moves one foot from 0 to 50 mm relative to its initial X. The other foot is then 50 mm behind support and must land 50 mm ahead, so it travels `50 + 50 = 100 mm`. This is why foot travel differs from nominal step length.

## 2. Exchange support before lifting

`AlternatingWalkProgram.__init__()` builds a sequence of tables. The newly landed foot becomes `support`; the opposite foot becomes `swing`. During transfer, desired COM moves toward support and planned swing-foot load decreases from one half of body weight to zero.

`walk_pose()` solves pelvis X/Y for that desired COM. It uses the two legs' inverse kinematics through `FirstStepProgram.angles()`. The root solver changes two pelvis coordinates until the computed mass-weighted COM matches the target. The explicit Jacobian estimates how COM changes for a 1 µm change in each pelvis coordinate.

## 3. Convert a foot position into joint angles

For an upright pelvis and a flat sole:

```text
dy = world_foot_y - pelvis_y - hip_lateral_offset
z  = world_foot_z - pelvis_height
hip_roll   = atan2(dy, -z)
ankle_roll = -hip_roll
```

The sagittal two-link solver uses foot X and `-sqrt(dy² + z²)` to calculate hip pitch and knee pitch. Ankle pitch cancels their sum so the sole stays flat. `model/sagittal_step.py` contains the two-link mathematics; `model/roll_geometry.py` checks the resulting forward kinematics and joint limits.

## 4. Estimate gravity-support torque

Planned left and right vertical reactions sum to `mass × 9.81`. Their weighted pressure center must match COM projection in static equilibrium. `walk_pose()` accounts for small pressure offsets, then calculates static joint torques. These become feedforward values, not sensor measurements.

The planner rejects sampled poses that exceed the static reserve, lack sole margin, violate joint limits, or fail conservative collision checks.

## 5. Move smoothly through each phase

`sample(elapsed)` converts simulated elapsed seconds into a phase and interpolates its target table. The quintic interpolation is:

```text
u(s) = 10s³ - 15s⁴ + 6s⁵,  0 ≤ s ≤ 1
```

Its endpoint velocity and acceleration are zero. Lift, horizontal advance and lowering are separate phases, which makes a failure easier to locate. The planner samples targets; the ROS node publishes no command until live safety checks also pass.

The complete walking timeline is in `setup['steps']`. The inherited `setup['schedule']` describes the original first-step prefix only.

## 6. Turn targets into bounded efforts

In `control/roll_posture.py`, for each joint:

```text
error     = target_angle - measured_angle
P         = Kp × error
D         = -Kd × measured_velocity
I_next    = bounded(I_previous + Ki × error × dt)
requested = gravity_feedforward + P + D + I_next
command   = clip(requested, -motor_cap, +motor_cap)
```

Anti-windup prevents the integral from accumulating further torque in a direction that is already saturated. The code logs requested and clipped values separately. During near-zero-load phases, the established gain schedule follows the current supporting foot; the next placement swaps its role.

## 7. Confirm that motion actually happened

`guard()` checks live force, freshness and sole positions. A foot planned at least 0.5 mm above the floor must have fresh explicit no-contact evidence. The supporting foot must remain loaded and COM must retain the required margin. Landing contact must return before reload.

After recording closes, `validation/alternating_walk_audit.py` checks each placement separately. Shared `validation/motion_audit.py` recalculates the PID torque equation and checks body sensors. A plausible animation alone is insufficient.

Questions to test your understanding:

1. Why does the second foot travel 100 mm if step length is 50 mm?
2. Which file decides the supporting side? Which file publishes torque?
3. Why does a missing contact packet not mean the foot is airborne?
4. What distinguishes planned feedforward torque from measured contact force?
5. Why can a robot complete its motion while the recording still fails an audit?

## Extension: count, distance and speed

Read `motion/walking_profile.py` first: it names the three settings. `step_length_m` means the distance **ahead of the supporting foot**, so later foot travel is twice that number. Count means individual placements.

Next read `motion/profiled_walk.py`. Its `knots` pair each old phase boundary with a new boundary. For a movement phase, `new_duration = old_duration × time_scale`; for a protected measurement hold it is `max(minimum_duration, old_duration × time_scale)`. `sample(t)` maps actual elapsed simulation time back onto the geometric planner's clock. This reuses the same smooth path at a different speed. The independent recorder and freshness checks always use actual simulation timestamps.

For a smooth path, halving phase time approximately doubles joint velocity and quadruples joint acceleration. That is why passing a static torque calculation does not prove a faster run will pass force and tracking checks. The failed 4× transfer and successful 2× three-step trial illustrate this distinction; see [the experiment record](../guides/LONGER_FASTER_WALK.md).

Finally inspect the cycle cache in `motion/alternating_walk.py`. It stores the two alternating support patterns after constructing and checking them. Translating both feet, pelvis and COM by the same world-X distance changes the robot's location but leaves relative geometry, joint angles and gravitational torques unchanged on flat ground. The cache is specific to this straight, level-ground experiment; slopes, obstacles and turning would need new planning checks.
