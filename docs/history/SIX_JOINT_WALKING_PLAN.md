# Walking first with the existing six joints

We will keep the current hip, knee and ankle pitch joints in each leg. The first
target is repeated forward/backward-plane steps. Adding joints for sideways
and turning motion is deferred until after that demonstration.

## What this choice means physically

All six joint axes are +Y. They control motion in X/Z, but cannot actively move
the pelvis sideways relative to a flat support foot. Hip attachments are at
Y = ±0.085 m and each foot is 0.12 m wide. With an upright symmetric pelvis,
the centered COM is at Y = 0, while a single flat foot spans Y = 0.025 to 0.145 m
(or the negative equivalent). The centered COM projection is outside that
single-foot area by 2.5 cm. This static geometry is not a complete dynamic
stability analysis, but it explains the missing lateral control authority.

Our initial six-joint walking demonstration will therefore need explicitly
identified lateral support, such as a planar simulation guide. A guide would
constrain sideways drift, body roll and yaw while allowing forward motion,
vertical motion and pitch. It must not hold up the robot or drive it forward.
It would be an assisted planar walking demonstration, not free 3D walking.
No guide has been added yet, and the free-standing push tests remain separate.

## The next milestones

1. **Checked foot geometry — prepared here:** convert foot positions into hip,
   knee and ankle targets; reject unreachable or out-of-limit positions.
2. **Explicit planar support:** implement and verify a separate guided mode,
   including evidence that it does not support vertical weight or propel X.
3. **Support transitions:** distinguish both-feet support, left-only support,
   right-only support and unexpected contact loss. Keep the current standing
   controller as the unchanged double-support baseline. Its IMU ankle correction
   currently requires both contacts and must not be reused unmodified in swing.
4. **One lift and landing:** command a small swing under controlled support,
   checking contact, orientation, motor budgets, COM and pelvis motion.
5. **One advancing step and weight transfer:** coordinate pelvis motion with
   the swing and landing. A fixed-pelvis foot path alone is not a walking step.
6. **Alternating steps:** connect transitions, then demonstrate repeated steps
   before reconsidering a model with additional axes.

We do not need arbitrarily strong standing-push recovery before attempting
these staged experiments. The recent guard stops remain useful constraints,
not a reason to change the robot's dimensions or silently raise motor caps.

## The geometric preview added now

The preview moves one ankle 0.03 m forward, lifts the flat sole by 0.02 m and
returns it to its starting height over a planned 2 s. The pelvis is fixed in
this calculation. The other leg retains the calculated knee=0.4 standing pose.
The planner does not start Gazebo, publish commands or claim a dynamically
feasible gait. It checks only kinematic reachability and joint angle limits.
It does not establish collision clearance, motor speed/torque feasibility,
ground reaction forces or walking balance.

Let `h`, `k`, `a` denote hip, knee and ankle angles, and `L1`, `L2` the segment
lengths. In the pelvis frame, the ankle joint origin is:

```text
x = -L1 sin(h) - L2 sin(h + k)
z = -L1 cos(h) - L2 cos(h + k)
foot_pitch = h + k + a
```

The negative signs follow the actual URDF +Y axes and links extending down -Z.
Inverse kinematics solves the same geometry:

```text
k = acos((x² + z² - L1² - L2²) / (2 L1 L2))
h = atan2(-x, -z) - atan2(L2 sin(k), L1 + L2 cos(k))
a = desired_foot_pitch - h - k
```

We choose the positive-knee solution because this model's knees bend from
0 to 2.4 rad. Straight or fully folded singularities are rejected before
solving, and each resulting angle is checked against the expanded URDF limits.

For normalized swing phase `s` between 0 and 1:

```text
u = 10s³ - 15s⁴ + 6s⁵
x(s) = start_x + advance × u
z(s) = start_z + clearance × sin²(πu)
```

The quintic phase starts and ends with zero velocity and acceleration. The
lift stays nonnegative, reaches clearance halfway and returns to the original
height. Smooth targets alone do not prove smooth or stable physical tracking.

## Files added and how they interact

- `my_robot_description/sagittal_step.py` has four separate pure functions:
  `ankle_position` checks forward geometry, `leg_angles` solves inverse geometry,
  `swing_target` creates the foot path, and `preview_step` assembles checked
  samples. It imports no ROS execution code.
- `scripts/preview_sagittal_step.py` expands the actual URDF, calls the planner,
  checks the targets by forward kinematics and writes JSON plus a plot.
- `tests/test_sagittal_step.py` checks the existing stance, bent/nonflat poses,
  target reconstruction, ground clearance, smooth endpoints, limits and invalid
  targets. The full suite now passes 215 tests.

No existing controller, launch, robot geometry or GUI file was modified.

## Run the preview

From the workspace root, with a new output folder:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/preview_sagittal_step.py \
  --advance 0.03 --clearance 0.02 --duration 2 \
  --output evidence/my-step-preview-01
```

This source-tree preview requires no rebuild. Inspect `preview.json` and
`preview.png`. Our first preview is in `evidence/sagittal-step-preview-01`;
all 101 targets reconstructed within approximately 1.1e-16 in the respective
coordinate units (metres for X/Z and radians for pitch). The final ankle is
forward relative to a fixed pelvis; it is not yet a translated robot or an
executed walking step.

## Updated project direction

After reviewing the single-leg balance goal, we chose to add hip roll and
ankle roll before attempting unsupported foot lifts. The six-joint model
remains the tested baseline. The new geometry milestone and revised sequence
are documented in [ROLL_JOINT_MODEL.md](ROLL_JOINT_MODEL.md); a planar guide
is no longer the intended path toward the initial unsupported balance tests.
