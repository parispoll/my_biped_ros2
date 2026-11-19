# Calculated step clearance: 30 mm vertical test

## What height does a first step need?

A perfectly flat floor has no single nonzero geometrically required swing
height. We must choose a design clearance and then check whether the actual
robot can reach it with sufficient support, collision and motor margins.

For a first **50 mm forward swing**, we selected these explicit design reserves:

| Contribution at mid-swing | Value |
|---|---:|
| Desired clearance reserve | 20.000 mm |
| Toe/heel pitch allowance, half sole length × sin(5°) | 8.716 mm |
| Existing vertical tracking error tolerance | 0.500 mm |
| Total | 29.216 mm |
| Selected vertical test height, rounded up | **30 mm** |

The foot is 200 mm long, so the conservative toe/heel allowance is
`100 mm × sin(5°)`. This is a hypothetical geometric design reserve, not an
allowance to tilt the foot by 5° during our test. The live foot-flatness and
0.5 mm corner-height tracking checks are unchanged. The 20 mm reserve is a
design choice, not a measured terrain requirement. Near takeoff and landing,
clearance must return to zero; the budget above describes the mid-swing target,
not a minimum throughout the entire foot path.

This is **15 times the previously verified 2 mm lift**, rather than another
tiny increment. The optional 10 mm setting remains prepared but unverified;
passing it is not a prerequisite for this explicitly calculated 30 mm study.

## Actual-model offline swing calculation

`scripts/planning/calculate_step_clearance.py` expands the current P54 URDF and checks
101 points of a fixed-pelvis forward swing on each supporting side:

```text
u = 10s³ − 15s⁴ + 6s⁵
forward travel = 50 mm × u
sole height = 30 mm × sin²(πu)
```

For each point it independently solves the swing leg with flat sole orientation,
checks URDF joint limits/FK, computes whole-body COM and single-support static
torques with the required CoP under COM, and checks conservative collision
envelopes. The supporting leg remains fixed. Both sides passed the sampled
calculation:

- Peak static motor utilization: approximately **53.058%**.
- Minimum supporting-sole COM margin: approximately **59.749 mm**.
- Minimum conservative collision-envelope separation: approximately **50 mm**.

These are static/kinematic predictions for a future swing. They do not implement
the landing/body-transfer controller needed to execute a walking step. The
current live test isolates the **30 mm vertical lift and landing**, with no
forward advance. That lets us verify the larger clearance physically in Gazebo
before introducing a new landing position and body movement.

Calculation and sampled targets:
`evidence/preparation/step-clearance-30mm/calculation.json`.

## Live-test changes

The planner and GUI now select explicit **2, 10 or 30 mm** targets. The default
remains 2 mm, keeping earlier runs identifiable. Every height has its own GUI
roadmap and matching-clearance offline-preview requirement. Larger heights
require a same-side measured 2 mm pass; a 2 mm preview cannot unlock a 30 mm run.

The model's motor caps, gains, passive damping and geometric error tolerances
are unchanged. The 30 mm reference uses **15 simulated seconds up and down**
instead of 10, retaining the existing 0.05 rad/s slow-motion reference bound.
We increased the height, not the allowed motor speed or tracking error.

At 30 mm the schedule from readiness is:

| Simulated seconds | Phase |
|---|---|
| 0–117 | Existing lower/transfer/unload |
| 117–132 | Lift 30 mm |
| 132–134 | Lifted hold; audit 132.5–134 |
| 134–149 | Lower foot |
| 149–151 | Recontact; contact required during 150–151 |
| 151–267 | Existing reload/center/rise/recovery |

The held minimum measured sole-corner height must be at least **29.5 mm**,
alongside fresh explicit no-contact evidence and valid supporting-foot load.
The 2 mm acceptance threshold cannot accidentally pass this test. Landing,
source coverage, motor accounting, pressure/COM, IMU and full return still need
to pass independently.

The 50 micrometre height-table spacing is retained, giving 601 checked vertical
poses for 30 mm rather than coarsening the old 41-point 2 mm table. Phase times
and audit windows are derived from the selected trajectory duration.

## File changes

| File | Change |
|---|---|
| `scripts/planning/calculate_step_clearance.py` | New explicit clearance budget and two-sided 50 mm offline swing study |
| Package `foot_lift.py` | Selected height, fixed table spacing and height-specific duration |
| Package `live_foot_lift.py` | Derived phase boundaries, selected-height sampling and acceptance minimum |
| Package launch / `hold_roll_stance.py` | Pass the selected clearance into the existing opt-in lift program |
| `scripts/experiments/run_lateral_shift.py` | `--lift-mm {2,10,30}`, recorded motion plan |
| `scripts/planning/preview_foot_lift.py` | `--clearance-mm`, complete selected-height trajectory and gradient checks |
| `tools/force_panel/` | Height selector, explicit history labels, separate milestones and matching evidence gates |
| `tests/test_lift_clearance.py` | Height-specific command/gate rejection, both-side larger geometry/virtual work and rejection of a 2 mm observation for a larger target |

Package modules are under `src/my_robot_description/my_robot_description/`.

## Run through the panel

Choose **30 mm foot-lift OFFLINE preview: left support**, generate it, then
choose **30 mm measured foot lift: left support** and Run. The right foot lifts.
The mirrored supporting side remains a separate measured milestone.

CLI equivalents after sourcing ROS and the workspace, each with a fresh folder:

```bash
python3 scripts/planning/preview_foot_lift.py --support left --clearance-mm 30 \
  --output evidence/my-30mm-preview
python3 scripts/experiments/run_lateral_shift.py --actuator-profile robotis_p54 \
  --foot-lift --lift-mm 30 --direction left --gui \
  --output evidence/my-30mm-lift
```

The CLI does not enforce panel-history prerequisites; inspect the relevant
evidence before using it. Do not edit package sources during recording.


## Measured 30 mm result — 11 November 2026

GUI job **20261111-181819-9790fdb3** passed the full 267-second sequence with
left support and the right foot lifting. The same-side 30 mm offline GUI
preview was `20261111-181615-ad52b1e8`.

| Measured / independently reviewed | Result |
|---|---:|
| Minimum held sole-corner clearance | **30.024847 mm** |
| Required minimum | 29.500 mm |
| Held fresh empty contact packets | 75 |
| Minimum held supporting-sole COM margin | 59.938295 mm |
| Peak motor utilization | 53.060476% |
| Peak measured joint speed | 0.0340808 rad/s |
| Saturated controller samples | 0 |

All six motion, sensor, damping, lowering/pressure, lift and load-guard audits
passed. Fresh no-contact evidence and landing contact were confirmed. The robot
completed centering, rising and return, restored the original gains, and all
owned processes exited cleanly. Package sources stayed unchanged during recording.
The physical geometry, masses, gain constants, motor configuration and damping
source hashes still match the 2 mm baseline.

Build and **34 focused tests passed**: 31 panel tests, one mirrored 2 mm
live-program regression, and two clearance tests including both-side 10/30 mm
geometry, collision envelopes, virtual work and invalid-height evidence rejection.
The full historical test suite was not rerun.

Independent raw-data review is `measured-review.json` in this preparation folder;
`physical-parameters-review.json` records unchanged baseline parameters.
The original raw recordings and plot remain under the live GUI job folder.

**Verified:** 30 mm vertical lift, no contact, landing and return with left support.
**Calculated only:** a 50 mm forward swing at this clearance.
**Still unrun:** mirrored right-support 30 mm live test and any executed forward
placement/step. The prepared 10 mm modes also remain unverified; the 30 mm pass
does not fabricate an intermediate experiment.
