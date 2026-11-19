# Step 5e: why does a lean remain?

**Follow-up:** [the physics torque audit](PHYSICS_TORQUE_AUDIT.md) now accounts for
the residual described below. DART's split-step implicit passive damping uses an
intermediate predicted velocity, which the preliminary final-speed estimate omitted.
The steps below preserve the initial audit and its assumptions.

This audit uses your recordings in `evidence/my-first-effort-push` and
`evidence/my-second-effort-push`. It compares their pre-push baselines with their
final recorded second. It does not change the controller's gains or add an
integrator.

The result is narrower than a complete root-cause claim: **the controller is still
requesting an ankle correction; its logged command is not fully accounted for by
our simple static gravity-plus-friction model.** We need physics-side force and
constraint evidence before assigning the remaining discrepancy to one cause.

## 1. What the controller is actually doing

From your second run, final-second mean values for the left ankle are:

| Quantity | Value |
| --- | ---: |
| Target angle | -0.175920294 rad |
| Measured angle | -0.186568053 rad |
| Target minus measured | +0.010647759 rad (about 0.610 degree) |
| Holding/feedforward term | 0 N m |
| Proportional correction | +0.638865530 N m |
| Speed-damping correction | about +0.000000000002 N m |
| Final command | +0.638865530 N m |
| Torque cap | ±6 N m |

The right ankle agrees to rounding. The equation is:

```
error   = -0.175920294 - (-0.186568053)
        = +0.010647759 rad
P term  = 60 × 0.010647759 = +0.638865530 N m
D term  = -0.05 × speed ≈ 0 N m
command = 0 + P term + D term = +0.638865530 N m
```

The controller has not stopped issuing a correction. The *motion* has stopped.
Because its equation contains only the current error, speed and fixed feedforward,
those terms become constant once the robot stops. There is no integral term that
keeps accumulating the remaining angle error over time.

This is not proof that adding an integral term is the right repair: first check
that the simulated forces match our model and commands.

The 0.610 degree ankle error is measured against the target. The roughly 0.494
degree change reported by the push experiment is measured against the actual
pre-push pose. They differ because there was already a small pre-push ankle error.

## 2. Separate the three meanings of friction/damping

- **Joint friction:** the URDF specifies `joint_fric = 0.2` at each joint. For this
  static check we treat it as a nominal ±0.2 N m generalized friction bound at
  each ankle, not as a measured friction torque.
- **Passive joint damping:** the model specifies ankle damping of 2.5 N m s/rad.
  Using measured final speed around 4e-11 rad/s gives a *naive final-speed estimate*
  around 1e-10 N m; this omits DART's intermediate implicit-damping calculation.
- **Controller speed damping:** Kd = 0.05 N m s/rad; its logged contribution is
  around 2e-12 N m in the final second.

The Python Kd term is effectively zero at rest; the follow-up shows why DART's
implicit passive damping contribution is not. Foot/ground friction
is separate again: its dimensionless coefficient is 1.4. It was not changed.

Gazebo's DART construction maps SDF joint damping and friction into the joint's
properties in [SDFFeatures.cc](https://github.com/gazebosim/gz-physics/blob/gz-physics7/dartsim/src/SDFFeatures.cc).
DART implements a bounded Coulomb-friction constraint in
[JointCoulombFrictionConstraint.cpp](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/JointCoulombFrictionConstraint.cpp).
These sources support the nominal model interpretation, not a measurement of
what every constraint contributed in this particular installed simulation.

## 3. Calculate the gravitational ankle load from the measured lean

The earlier holding-torque calculation assumed an upright pelvis and the planned
joint angles. We cannot reuse its zero ankle torque once the body has leaned.

Imagine cutting through **both ankle joints** and keeping the pelvis, thighs and
shins above the cut. The feet are outside this free-body calculation. For symmetric
planar rest, the ankle axes share a common fore-aft location; reaction forces at
the cut therefore do not contribute pitch moment about that common axis.

The upper body's mass is:

```
7.5 + 2 × (1.4 + 1.0) = 12.3 kg
```

Using the measured joint angles, pelvis pitch, segment lengths and the pelvis
COM offset of +0.02 m, the upper-body centre of mass is approximately **3.720 mm
forward of the ankle axis** in your second run. Therefore:

```
total gravity moment = upper mass × gravity × horizontal COM offset
                     = 12.3 × 9.81 × 0.003719884
                     = 0.448852347 N m

required motor torque per ankle = total / 2
                               = +0.224426174 N m
```

Positive joint torque acts on the foot; its equal-and-opposite reaction acts on
the body above it. That is why this positive motor torque can oppose the positive
gravity moment on the upper body.

This estimate assumes equal left/right contributions, negligible acceleration,
sagittal motion and no continuing external wrench. The script rejects asymmetric
poses or appreciably non-flat feet and rejects non-stationary analysis windows.
It is an estimate from current URDF geometry, not an independent actuator sensor.

## 4. The simple torque balance does not close

| Final-second quantity, per ankle | Your run 1 | Your run 2 |
| --- | ---: | ---: |
| Logged command | +0.645692 N m | +0.638866 N m |
| Estimated gravity load | +0.227382 N m | +0.224426 N m |
| Difference | +0.418310 N m | +0.414439 N m |
| Nominal friction bound | ±0.200000 N m | ±0.200000 N m |

The difference is larger than that nominal friction bound. Consequently,
**"the configured 0.2 N m friction explains the whole lean" is not established.**
The residual is not a friction measurement. It could indicate a gap between
requested and applied torque, unaccounted physics constraints, or a limitation of
our assumed planar static model. We have not yet isolated those alternatives.

The code equation itself closes: holding + proportional + damping equals the
logged command, without clipping. The physics-side moment balance is the unresolved
part. That distinction tells us where to investigate next.

For intuition only, the local gravity slope with hip and knee angles held fixed
is about 25.98 N m/rad per ankle, while ankle Kp is 60 N m/rad. This is not a full
coupled-system stability analysis, and does not justify choosing gains from those
two numbers alone.

![Ankle command and conditional gravity audit](../evidence/ankle-diagnosis/ankle-audit.png)

## 5. The zero-joint-friction diagnostic

I added an optional `joint_friction` Xacro/launch argument. Its default is still
0.2. Only a deliberately requested diagnostic uses zero. Tests verify that this
override changes only the six joints' friction values; masses, geometry, foot
contact settings, damping, gains and effort interfaces stay the same.

The isolated zero-friction attempt is recorded in
`evidence/ankle-diagnosis/zero-joint-friction`. It failed the existing startup
readiness check: two seconds after activation, the ankle error remained about
0.0434 rad, above the allowed 0.03 rad. The controller stopped the simulation.
No push was applied.

That is **not a valid post-push recovery comparison**. It also does not prove that
the zero-friction robot could never settle with more time. It shows that removing
friction is not a drop-in solution under the current startup protocol. The default
model and the readiness threshold were preserved rather than changed to obtain a
passing report.

To reproduce that diagnostic deliberately:

```bash
python3 scripts/run_effort_push.py \
  --joint-friction 0 \
  --output evidence/my-zero-friction-diagnostic
```

It is expected to stop during startup with the current settings. For ordinary
experiments, omit `--joint-friction` so the default remains 0.2.

## 6. Files added or changed

| File | Purpose |
| --- | --- |
| Package `my_robot_description/ankle_diagnosis.py` | Computes upper-body gravity moment and its local slope from measured posture. No ROS runtime needed. |
| Workspace `scripts/diagnose_ankle.py` | Reads your existing recordings and reports baseline/final controller terms and conditional moment residuals. |
| Workspace `scripts/plot_ankle_diagnosis.py` | Plots the command terms and compares them with the simple gravity/friction range. |
| `tests/test_ankle_diagnosis.py` | Checks ideal balance, energy/torque consistency, gravity slope, rejected assumptions and the isolated friction override. |
| Package `urdf/effort_standing.urdf.xacro` | Adds optional joint-friction argument, default 0.2. |
| Package `my_robot_description/description.py` | Accepts and validates that optional override. Existing callers retain their defaults. |
| Package `launch/effort_standing.launch.py` | Passes the optional friction setting into Xacro. |
| Workspace `scripts/run_effort_push.py` | Exposes the diagnostic option and records its value on successful recordings. |

All 67 tests passed. The gravity expression also agrees with the derivative of
potential energy, checking its sign and magnitude. No gains, target angles or
normal-mode torque limits were changed.

## 7. Recreate the audit without another simulation

```bash
cd '/home/parispoll/Documents/ChatGPT/robotics carrer'
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash

python3 scripts/diagnose_ankle.py \
  evidence/my-first-effort-push \
  evidence/my-second-effort-push \
  --output evidence/ankle-diagnosis/my-audit.json

python3 scripts/plot_ankle_diagnosis.py evidence/ankle-diagnosis/my-audit.json
```

The report uses the current default expanded URDF, so its physical parameters
must match those used for the recordings. The separate zero-friction attempt is
not input to this audit because it never produced a valid push recording.

The next investigation should observe physics-side joint commands/transmitted
wrenches and constraints, with their frames and points of application established.
Until that balance is checked, increasing gains or adding integral action would
change the response without resolving the unexplained moment.
