# Adding sideways joints: the geometry milestone

We are moving from three pitch joints per leg to five joints per leg, before
attempting unsupported single-leg balance. The six-joint model and all its
existing controllers, launch files, evidence and GUI settings are preserved.
This separate prototype is named `biped_roll`. Hip yaw and turning are deferred.

## What changed in the joint chain

Before, each leg was:

```text
base_link → hip (+Y) → thigh → knee (+Y) → shin → ankle (+Y) → foot
```

The prototype is:

```text
base_link
  → hip_roll (+X)
  → hip_roll_carrier
  → hip_pitch (+Y)
  → thigh
  → knee_pitch (+Y)
  → shin
  → ankle_pitch (+Y)
  → ankle_pitch_carrier
  → ankle_roll (+X)
  → foot
```

X points forward, Y sideways and Z upward. Pitch rotates about Y; roll rotates
about X. Both left and right joint axes use the same positive coordinate axes.
A mirrored outward leg motion therefore requires opposite roll signs.

Hip roll and hip pitch are co-located at the original hip attachment. Ankle
pitch and ankle roll are co-located at the original ankle position. Rotations
are applied in chain order; they do not generally commute. This is a simplified
universal-joint arrangement, not a CAD model of a particular robot's actuators.

With roll at zero, the old thigh/shin/foot transforms match at straight and
bent standing poses. The segment lengths and foot dimensions stay the same.
Mass distribution changes slightly because the carriers are physical links.

## Why two extra links per leg?

URDF represents a chain of one-axis joints connected by links. An additional
rotation needs an intermediate link. We use finite physical mass/inertia for
each carrier, rather than a massless dynamic link.

Each carrier is provisionally an 80 g, centered 5 × 5 × 4 cm box. Its principal
moments follow the box equations:

```text
Ixx = m (width² + height²) / 12
Iyy = m (length² + height²) / 12
Izz = m (length² + width²) / 12
```

There are four carriers: mass increases from 14.10 kg to **14.42 kg**. The
values are explicit assumptions, not measured actuator data. Carriers have
visuals and inertias but no separate collision geometry, because these small
internal housings overlap existing segment geometry in this simplified model.
Housing collisions and mechanical clearances have not been validated.

New roll-joint parameters are provisional:

| Parameter | Value | Meaning |
| --- | --- | --- |
| Angle range | ±0.35 rad (about ±20°) | Initial model limit |
| URDF effort limit | 6 N·m | Initial declared limit; not validated actuator sizing |
| URDF velocity limit | 2 rad/s | Initial declared limit |
| Passive damping | 0.2 N·m·s/rad | Modeling assumption; not a tuned feedback gain |
| Coulomb friction | 0 | Initial simplifying assumption |

The pitch joint limits retain the original model values. Their URDF effort
limits are not the bounded 6/10/6 N·m motor command profile used by our old
standing experiments. No new standing motor profile is implemented here.

## Files added

- `urdf/biped_roll.urdf.xacro` is the new entry point. It composes existing
  dimensions/materials/base with the new leg macro. No controller plugin is
  attached, so existing six-joint command vectors cannot silently drive it.
- `urdf/components/leg_roll.xacro` defines the five-axis chain and finite-mass
  carriers. Its segment geometry is initially copied from the stable leg macro
  to isolate this prototype; changes do not alter `leg.xacro`.
- `scripts/planning/roll_model_geometry.py` checks the tree, names, axes, limits and
  principal inertias. Separate functions calculate link transforms and COM.
  This offline utility does not publish motor commands.
- `scripts/checks/check_roll_model.py` expands the actual Xacro and writes an expanded
  URDF, a model-check JSON and a front-view diagram.
- `tests/test_roll_model.py` checks physical inertias, model topology, original
  neutral/crouched geometry, roll direction, mirrored symmetry, level soles
  in the demonstrated straight-leg roll poses, and invalid inputs.

The new joint names explicitly include `_pitch` or `_roll`. Porting control
must use a named joint map, not assume the old six-element array order.

## What was verified

- Ten revolute joints and eleven connected links.
- Correct +X roll / +Y pitch axes and positive finite limits.
- Positive finite masses and diagonal inertias satisfying physical triangle
  inequalities.
- Matching original segment transforms when roll is zero, including crouches.
- Opposite hip/ankle rolls of ±0.10 rad can keep the foot orientations level
  in the demonstrated straight-leg poses while moving the feet sideways.
- Mirrored leg poses retain symmetric feet and centered lateral COM.
- Gazebo's URDF-to-SDF converter retains all ten joints and eleven links, with
  an empty conversion diagnostic log.
- The full test suite passes 221 tests.
- Every pre-existing package source file matches its pre-change hash.

These are model/kinematics checks. They do not establish physical standing,
contact loading, collision clearance, motor feasibility or single-leg balance.
The front-view diagram holds the pelvis fixed for calculation; the wider pose
is not a commanded weight shift or a planted-foot stance.

## Reproduce the geometry check

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/checks/check_roll_model.py --output evidence/my-roll-model-check
```

This source-tree checker does not require a rebuild. Inspect `model-check.json`,
`biped_roll.urdf` and `front-view.png`. To check Gazebo conversion:

```bash
gz sdf -p evidence/my-roll-model-check/biped_roll.urdf
```

The first artifacts, conversion result and test log are in
`evidence/roll-model-01/`. The verification file includes source hashes so the
preserved baseline can be checked independently.

## What comes next

The next milestone is a separate ten-joint **two-foot standing** setup. First
calculate the new mass/COM and gravity loads, define named bounded motor
commands, and adapt sensing to eleven links. The current six-joint stance
model deliberately rejects extra links. The COM probe already iterates over
all inertial links, so its new use needs a mass/coverage check rather than a
seven-link rewrite. Model assumptions must be checked explicitly.

Start with neutral roll references and verify two-foot contacts, pelvis pose,
IMU freshness and motor limits. Active roll feedback and weight shifting come
after that baseline. Then unload one foot before attempting a small lift.
No standing or walking capability is claimed for this prototype yet.

The separate standing implementation is described in [ROLL_STANDING.md](ROLL_STANDING.md).
