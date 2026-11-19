# Guarded 2 mm foot lift

This stage adds a separate live lift mode after the offline preview. A prepared
mode is not a verified lift: only a completed measured trial whose independent
audits pass marks the corresponding supporting side verified in the roadmap.

## Motion and control

Keep the verified zero-load transfer unchanged up to 117 simulated seconds.
At that endpoint keep the pelvis and supporting-leg target fixed, independently
bend the swing leg to lift its flat sole 2 mm, then return it:

| Time after readiness, simulated s | Phase |
|---|---|
| 0–117 | Existing baseline, 40 mm lowering, transfer and zero-load hold |
| 117–127 | Smooth 2 mm lift |
| 127–129 | Lifted hold; audited window 127.5–129 |
| 129–139 | Smooth foot lowering |
| 139–141 | Recontact settling; require contact during 140–141 |
| 141–257 | Existing centering, rise and recovery, shifted 24 s later |

No forward foot advance or force pulse. The whole sequence takes about 19–20
wall minutes including startup at 0.25× simulation pacing. All clocks above
begin at the controller's measured readiness, not at the GUI click.

The single-support feedforward from `foot_lift.py` sets the swing reaction to
zero and supporting CoP under the whole-body COM. Its initial torque matches
the zero-load feedforward to numerical precision. The previously validated
supporting-ankle and free-ankle gain schedules remain at their zero-load values
during lifting, then restore on centering. No motor caps, passive damping or
robot dimensions are changed.

## New live measurements and guards

`hold_roll_stance.py` subscribes to the existing read-only `/biped/com_state`
physics probe only in lift mode. That supplies actual world sole corners and
COM, independent of the commanded angles. Stale/malformed probe data prevent
readiness or stop a ready experiment.

In addition to existing timing, IMU, posture and 80% motor-reserve guards:

- Sole observation source age −0.02 to +0.10 s and expected model mass.
- Every sole corner within 0.5 mm of its planned world height.
- Each foot's horizontal center stays within 1 mm of its original position.
- Supporting foot remains freshly loaded and total reaction within 5% of weight.
- During final unloading and the lift/landing insertion, swing reaction ≤0.5 N
  and COM projection at least 10 mm inside the supporting sole.
- Once planned clearance is at least 0.5 mm, fresh explicit `no_contact` is
  mandatory on the swing foot. Silence never becomes zero support.
- During the last recontact second, a real foot/ground manifold must return
  before the existing reload/centering phase begins.

The live load and geometry evidence is saved alongside torque components. The
offline lift audit separately pairs raw physics-state contacts with recorded
COM/sole observations and replays the guards. The held minimum actual corner
height must be at least 1.5 mm; held contact states must all be `no_contact`.
The original pose/IMU/force/CoP/command/damping/return audits remain required.

The generic planted-foot audit has a narrowly scoped optional foot-height
reference: only lift mode compares the moving swing sole against that planned
height. Other modes keep their original zero-motion foot reference.

## Collision check scope

Before live activation, all 41 lift-height table points are checked with
conservative oriented boxes enclosing each URDF box/cylinder collision shape.
Separating axes test non-mounted body pairs. Bodies joined by one or two tree
edges are treated as mounting interfaces (the intermediate carriers have no
collision geometry) and excluded. Non-foot envelopes must stay above the floor.

The candidate's smallest separating-envelope gap is about 50 mm. This is a
sampled geometric check, not swept-volume collision certification or impact
prediction. It does not change self-collision flags or robot geometry.

## File map

| File | Responsibility |
|---|---|
| `my_robot_description/live_foot_lift.py` | Separate program, phase/contact/sole guards, collision-envelope checks and independent measured lift audit |
| `my_robot_description/foot_lift.py` | Reuse the precomputed zero-load program in the preview calculation, without changing default preview behavior |
| `scripts/hold_roll_stance.py` in the package | Opt-in physics geometry subscription, guarded program selection and live evidence recording |
| `launch/roll_standing.launch.py` | `foot_lift_enabled`, state probe/bridges and controller wiring |
| `my_robot_description/lateral_audit.py` | Optional planned swing-sole height reference, preserving old defaults |
| Workspace `scripts/experiments/run_lateral_shift.py` | `--foot-lift`, isolated lifecycle, measured lift audit and complete evidence |
| Workspace `scripts/plots/plot_lowering.py` | Additional measured-versus-requested sole-clearance plot |
| `tools/force_panel/` | Live selector, same-side zero-load/preview prerequisites, measured clearance metric and separate roadmap entries |
| `tests/test_live_foot_lift.py`, `tests/test_force_panel.py` | Both-side planning/guards, false-clearance/stale-data/contact/landing rejection and GUI routing |

Package-relative paths above are under `src/my_robot_description/`.

## Reproduce

In the panel choose **2 mm measured foot lift: left support** or the mirrored
entry. Both an existing same-side measured zero-load pass and an offline preview
pass are prerequisites. The opposite foot is the one that lifts. Gazebo can be
shown for visual inspection. The simulation closes after the bounded trial.

CLI equivalent, with a new output folder:

```bash
cd '/home/parispoll/Documents/ChatGPT/robotics carrer'
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
python3 scripts/experiments/run_lateral_shift.py --actuator-profile robotis_p54 \
  --foot-lift --direction left --gui --output evidence/my-live-lift-left-01
```

The CLI does not consult panel history; use it only after checking the same
prerequisite evidence yourself. A failing trial retains logs and partial data.
Do not edit package sources during recording. Right-support verification is
separate; neither side should be inferred from the other.

Walking and single-support disturbance recovery remain later milestones even
if this slow vertical lift, contact loss and return succeed.


## First measured result — 11 November 2026

GUI job **20261111-160340-2d4a7aff** passed the complete 257 simulated-second
sequence with **left support and the right foot lifting**. Gazebo was visible.
The panel recorded 19 minutes 51 seconds of wall time including startup/audits.

| Independently reviewed measurement | Result |
|---|---:|
| Minimum held right-sole corner height | 2.002620 mm |
| Held fresh empty physics contact packets | 75 |
| Held right ground reaction | 0 N, explicit no-contact packets |
| Minimum held left-sole COM margin | 59.992074 mm |
| Peak recorded motor utilization | 53.059642% |
| Peak measured joint speed in the program | 0.0223214 rad/s |
| Saturated controller samples | 0 |

Landing contact returned before reloading. All six shift, independent sensor,
physical damping, lowering/pressure, lift and load-guard audits passed.
The original gains were restored. Controller, COM observer and Gazebo exited
cleanly. Package source hashes remained unchanged during the recording and
matched the raw-data review after completion.

The measured lift is a new milestone beyond touching-unloaded zero load.
It is one small, slow vertical lift and return, with no push or forward step.
At completion of the first trial the right-support live lift was prepared but unrun.
The subsequent mirrored result is recorded below.

Build passed; 49 focused tests passed (31 panel, 12 existing lateral, 5 existing
zero-load, and one isolated live-lift regression exercising both sides and
invalid evidence). The full historical suite was not rerun.

Evidence: `evidence/gui/20261111-160340-2d4a7aff/` contains the summary, plot,
raw contacts/IMU/pose/COM, joint terms and launch logs. Independent raw-packet,
clearance, support margin, gain restoration and shutdown checks are in
`evidence/preparation/live-foot-lift/measured-review.json`.

For this new optional height-reference mode, the legacy generic subreport
fields `shift.max_abs_sole_height_m` and `max_foot_center_drift_m` mean residual
relative to the planned swing height; they do not say the lifted foot stayed on
the ground. The explicit `foot_lift.held_minimum_clearance_m` and raw COM corners
are the actual clearance measurements. The top-level legacy `outcome` string
still names the enclosing lowered-transfer runner; use the mode, `foot_lift`
audit and source snapshot to identify this trial accurately.


## Mirrored right-support result — 11 November 2026

GUI job **20261111-174104-8f03aa19** passed the full 257 simulated-second
sequence: right support, left foot lifting, fresh contact loss, landing,
reloading, centering and rise. Gazebo was enabled. All six audits passed,
all owned processes exited cleanly and the original gains were restored.

| Raw-data review | Left support / right swing | Right support / left swing |
|---|---:|---:|
| Minimum held sole-corner clearance | 2.002620 mm | 2.002252 mm |
| Minimum held supporting-sole COM margin | 59.992074 mm | 59.992077 mm |
| Held fresh empty contact packets | 75 | 75 |
| Peak motor utilization | 53.059642% | 53.059640% |
| Saturated samples | 0 | 0 |
| Peak joint speed over complete motion | 0.0223214 rad/s | 0.0223572 rad/s |

All 98 package-source hashes match across the two trials, and no package
sources changed within either recording. No robot control/model code was
changed between sides. This is a successful example on each supporting side,
not a repeatability study or proof of dynamic walking robustness.

Independent review: `evidence/preparation/live-foot-lift/mirrored-right-review.json`.
Combined evidence: `mirrored-comparison.json` in that same preparation folder.
Both GUI roadmap entries now show verified, and the server is idle.

Next development should check a larger vertical clearance offline before
introducing forward swing/placement. A future advancing step must coordinate
new landing geometry and body motion; it cannot simply reuse the old fixed-foot
return after moving a landing point. No forward-placement controller has been
implemented by these two vertical-lift tests.
