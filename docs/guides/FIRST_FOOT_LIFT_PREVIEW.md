> Follow-up: [the guarded live lift](LIVE_FOOT_LIFT.md) is now implemented and
> passed its first left-support measured trial. The offline work below remains
> a separate preparation milestone; its original future-work wording is historical.

# First foot lift: offline preparation, 11 November 2026

The first 2 mm lift is now a **kinematic/static preview**, selectable through
the existing experiment panel. It does not launch Gazebo, publish motor commands
or claim a measured foot lift. The existing zero-load experiments and their
live controller are unchanged.

## What we added and why

The verified zero-load endpoint leaves one foot touching with about 0.0035 N.
Before removing that contact, we need an independently moving swing leg and a
single-support static model. Continuing to use a prescribed two-foot pressure
split for a lifted foot would be the wrong physical assumption.

The new planner holds the pelvis and supporting leg at the verified zero-load
endpoint. It raises the other flat foot **2 mm**, without moving it forwards or
sideways, then returns it. It solves the swing leg separately:

```text
dy = desired_ankle_y_in_pelvis − hip_y
hip_roll = atan2(dy, −desired_ankle_z_in_pelvis)
pitch_chain_z = −sqrt(dy² + desired_ankle_z_in_pelvis²)
hip_pitch, knee_pitch, ankle_pitch = existing checked sagittal IK
ankle_roll = −hip_roll
```

Both soles remain flat in the target geometry. The supporting foot must not move;
all four swing-sole bottom corners must rise by the requested amount. The planner
uses the URDF collision box, not an ankle-height shortcut. Forty-one height
samples are checked against URDF joint limits, a 10 mm support-COM margin and
the existing 80% static motor budget.

The preview trajectory begins **at the already transferred zero-load pose**, not
at startup. It is 10 s lifting, 2 s holding and 10 s lowering, using quintic time
scaling. These are proposed trajectory times, not elapsed time in a running
simulation. Sampled target joint speeds remain below the existing 0.05 rad/s
slow-motion reference check. This is not a global interpolation-error or dynamic
torque bound.

## Single-support statics

Assume flat support, no acceleration, no horizontal ground force and no yaw
moment. The swing foot supplies **zero reaction**. The supporting vertical force
is the full weight, about 218.5668 N. Whole-body moment balance requires its
center of pressure to lie under the actual whole-body COM projection:

```text
F_support_z = M*g
F_swing_z = 0
CoP_support_xy = COM_xy
tau_joint = −axis · (sum(distal gravity moments) + support reaction moment)
```

The supporting CoP moves slightly when the swing leg bends; we do not keep it
artificially fixed at the sole center. The calculation rejects a COM projection
closer than 10 mm to a supporting-sole edge. CoP here is a required static
prediction, not a new pressure controller or measured value.

An independent check reconstructs total gravitational potential while the
supporting foot is held fixed. For each joint:

```text
U(q) = sum(m_i*g*z_i(q))
tau_static_i = dU/dq_i
```

Central finite differences are compared with the separate contact-moment torque
calculation. The runner checks the start, midpoint and full lift; the regression
test checks five heights per side. This can expose axis/sign/moment mistakes
that a mirrored kinematic path alone would miss.

## Files and responsibilities

| File | Responsibility |
|---|---|
| `src/my_robot_description/my_robot_description/motion/foot_lift.py` | Pure flat-foot IK, collision-box sole corners, single-support static loads, preview timing and independent gravity-gradient check |
| `scripts/planning/preview_foot_lift.py` | Expand current P54 model, generate numerical checks/source snapshot and labelled prediction plots; no ROS nodes or simulation |
| `tools/force_panel/config.py` | Explicit offline request; rejects force, Gazebo and active balance options |
| `tools/force_panel/jobs.py` | Same-side verified zero-load prerequisite, owned preview execution, separate offline roadmap labels |
| `tools/force_panel/static/index.html`, `app.js` | Preview selector, supporting-side choice, predicted metrics and clear offline labels |
| `tests/test_foot_lift.py` | Both-side FK, fixed support, sole-corner motion, virtual work, mirror consistency, return and invalid target rejection in an isolated P54 process |
| `tests/test_force_panel.py` | Offline command routing, invalid combinations, prerequisite rejection and prevention of a misleading live-verified label |

No existing gain, cap, URDF dimension, damping coefficient, live controller,
launch file or previous experiment acceptance threshold was changed.

## Using the GUI

1. Open or refresh `http://127.0.0.1:8765/`.
2. In the roadmap choose **2 mm foot-lift OFFLINE preview: left support** or
   its right-support counterpart.
3. Click **Generate offline lift preview**.
4. Inspect planned clearance, predicted COM margin and the saved prediction plot.

The result is **Offline checked**, not a measured lift. Its roadmap entry says
that live lift remains unimplemented. Previous measured zero-load milestones
retain their own verified labels. A new output folder/history entry is created
for every preview.

CLI alternative, from the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/planning/preview_foot_lift.py --support left \
  --output evidence/my-foot-lift-preview-left-01
```

Use `--support right` for the mirror and a new output folder. A build is useful
after adding the new package module. The preview explicitly selects P54 before
loading robot modules, preserving legacy defaults elsewhere.

## What remains before a live lift

This preview does not check full-body self collision, impact/landing forces,
nonlinear single-support stability, realistic sensors or actuator dynamics.
It must not be connected to the live effort publisher as if those checks had
already passed.

The proposed live phase sequence is recorded in the preview metadata:

```text
lower pelvis → transfer → confirm unloaded → lift
→ confirm measured clearance AND fresh no_contact → lifted hold
→ lower foot → confirm recontact → reload → center → rise
```

Next implementation must introduce a separate guarded lift program and
independent world-sole-clearance/support/contact/return audits. The current
bilateral audit assumes both soles stay planted, so it cannot be reused unchanged.
Supporting-foot contact remains mandatory. Absence of swing contact must be a
fresh explicit physics-state measurement, never inferred from silence.

Live collision/landing rules and a lifted-posture gain assessment remain to be
implemented before the first measured lift trial. This stage establishes a
consistent candidate geometry and torque model, not walking or executed balance.


## Checked outcomes

Both previews were generated through the actual GUI, not injected into its
history. Left support: `20261111-153041-84de52b1`; right support:
`20261111-153226-924eeed9`. Each completed successfully, kept its package
sources unchanged and generated the labelled prediction plot.

| Predicted quantity | Both supporting sides, approximately |
|---|---:|
| Full swing clearance | 2.000 mm |
| Minimum support-sole COM margin | 59.9957 mm |
| Peak static motor utilization | 53.0580% |
| Peak sampled target joint speed | 0.00397491 rad/s |
| Contact-moment vs gravity-gradient error | below 1.3e-8 N·m |

The package built successfully, all 30 panel tests passed, and the isolated
mirrored geometry/virtual-work/rejection regression passed. The full historical
suite was not rerun. The existing 96 hashed package files from the last live
zero-load trial are unchanged; `foot_lift.py` is a new module.

`evidence/preparation/foot-lift-preview/verification.json` records the jobs,
source checks and test scope. `gui-offline-checked.png` shows the real page's
predicted clearance and explicit “Offline only; no measured lift” label.
The panel is idle and available for more previews or existing measured trials.
