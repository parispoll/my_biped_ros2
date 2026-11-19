**Latest review: both supporting sides passed all three stages; six measured trials validated.**

**Measured validation completed on 5 November 2026: all three left-support stages passed.**

# Staged lowered unloading: 20%, 10%, 5% remaining load

This separate mode tests progressively lighter positive contact on one foot.
The supporting side is named explicitly. The requested first sequence uses
left support, unloading the right foot; mirrored roadmap entries are separate.
No foot is lifted. Missing/empty contact messages remain invalid, never zero.

## Model and schedule

With 40 mm lowering, solve for fixed flat soles, fore/aft COM centering and
lateral COM position `(0.5 - lighter_fraction) × 0.17 m`. Static sole-center
force balance predicts the requested split. All joint limits and cached path
motor torques are checked. No mass, geometry, gains or motor caps are changed.

| Right-foot target (left support) | Pelvis shift | COM shift | Endpoint peak static utilization | Measured floor |
|---|---:|---:|---:|---:|
| 20% | 79.124 mm | 51.0 mm | 28.47% | 18% |
| 10% | 105.552 mm | 68.0 mm | 37.27% | 8% |
| 5% | 118.782 mm | 76.5 mm | 44.84% | 3% |

Values above are predictions until each live recording is audited. The
legacy motor limits in older support-search documents refer to the former
14.42 kg robot. This mode uses the current 22.28 kg P54 profile.

Use 64 simulated seconds per sideways movement, matching the duration of
the verified 40 mm experiment. Larger distance increases reference speed;
offline sampled checks enforce the existing 0.05 rad/s audit bound. This is
still slow quasi-static motion, not a walking-speed or impact test.

The full schedule is baseline 3 s, lower 40 s, settle 4 s, transfer 64 s,
hold 6 s, return 64 s, settle 4 s, rise 40 s, recovery 8 s: 233 simulated
seconds after readiness. Each run takes approximately 17–18 wall minutes.

## Acceptance and progression

Keep existing sensor freshness/coverage, contact, tracking, sole geometry,
vertical force sum, torque and damping checks. The new mode alone selects
its measured lighter-foot floor, two percentage points below its target.
The previous modes retain their 25% measured and 30% planned limits.

The held measured lighter-foot share must be within one percentage point
of the target. The measured COM ground projection must be at least 10 mm
inside the supporting sole during the audited hold. This is a geometric
condition, not proof of single-support dynamic balance.

The panel refuses a 10% test until a measured 20% run passes on the same
supporting side; similarly 5% requires 10%. A failed or interrupted stage
must not unlock the next. Each trial returns to centered standing before
another starts. First sequence: 20%, 10%, 5%, all with left support.

## Implementation map

- `my_robot_description/lowered_unload.py`: endpoint IK, joint checks, derived target-specific floors and independent held-load/COM-margin audit.
- `my_robot_description/lowered_shift.py`: shares existing trajectory construction via narrow subclass hooks; original distance validation and defaults retained.
- `my_robot_description/partial_unload.py`: explicit validated share-floor argument; existing callers retain their original defaults.
- `scripts/hold_roll_stance.py`, `launch/roll_standing.launch.py`: select the new program and its guard, preserving control law and shutdown handling.
- `scripts/experiments/run_lateral_shift.py`: `--lowered-unload {0.2,0.1,0.05} --direction {left,right}`, saves goals and runs existing plus new audits.
- `tools/force_panel/config.py`, `jobs.py`, `static/index.html`, `static/app.js`: separate request mode, target selector, stage prerequisites, roadmap, history and plot integration.
- `tests/test_lowered_unload.py`, `tests/test_force_panel.py`: geometry/speed/torque/return, mirrored planning, old-envelope rejection, measured-audit rejection and prerequisite checks.

Offline evidence: `evidence/preparation/lowered-unloading/`.
Live job IDs and results are recorded there as each experiment completes.


## Measured results — left support, right foot unloaded

| Target right share | Measured share | Pelvis travel | COM travel | Minimum held COM margin inside left sole | Peak motor utilization |
|---|---:|---:|---:|---:|---:|
| 20% | 19.9992% | 79.131 mm | 51.004 mm | 26.004 mm | 28.474% |
| 10% | 9.9989% | 105.561 mm | 68.006 mm | 43.005 mm | 37.271% |
| 5% | 4.9988% | 118.791 mm | 76.506 mm | 51.505 mm | 44.841% |

All six audits passed for every stage. Zero saturated motor samples were
recorded. All returned to centered standing, with final height offset below
0.003 mm. Controller and COM observer exited cleanly in all three logs.
The package source hashes are identical between the three trials and unchanged
within each trial. The full offline suite passed 300 tests.

GUI jobs:

- 20%: `evidence/gui/20261105-071519-09e4cb38/`, including `summary.json`, `response.png` and raw recordings.

- 10%: `evidence/gui/20261105-073320-0c56723e/`, including `summary.json`, `response.png` and raw recordings.

- 5%: `evidence/gui/20261105-075132-6781cffa/`, including `summary.json`, `response.png` and raw recordings.

The 5% stage held approximately 95% of the weight on the left foot while
the right retained about 10.93 N of positive support. This demonstrates
quasi-static bilateral unloading and return. It does not demonstrate zero
contact, a lifted foot, single-support recovery or walking. The mirrored
right-support stages remain unrun. They should be checked before proceeding
to contact-aware zero-load handling and a small controlled foot lift.


## Mirrored user-run results — right support, 5 November 2026

All three right-support trials passed every measured audit. Held contact
wrenches were independently decoded again during review and reproduce the
reported lighter-foot shares. The stopped left-support repeat is retained
in history and is not counted as a passed experiment.

| Target left share | Measured share | Minimum COM margin inside right sole | Peak motor utilization | GUI job |
|---|---:|---:|---:|---|
| 20% | 19.9992% | 26.004 mm | 28.474% | 20261105-082912-bbdb0a70 |
| 10% | 9.9989% | 43.004 mm | 37.271% | 20261105-110657-594745a8 |
| 5% | 4.9988% | 51.505 mm | 44.841% | 20261105-112514-fd94f7f3 |

All returned to centered standing, with zero saturated motor samples, clean
controller and COM-observer exits, and unchanged package sources within
each recording. Package hashes also match the paired left-support trials.
Both supporting sides have now passed 20%, 10% and 5% lighter-foot targets.
The source hashes and recordings remain preserved in their original folders.

Next development: distinguish fresh genuine zero-contact evidence from
missing/stale data, plan and validate a zero-load transition with suitable
pressure/torque references, then consider a small controlled foot lift.
These runs still have positive bilateral contact; no foot lift or single-support
stability has been demonstrated.
