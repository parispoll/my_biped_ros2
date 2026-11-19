# Next user-run milestone: 20 mm lowered weight transfer

**Validated in both directions on 4 November 2026.** See [the 30 mm next step](NEXT_LOWERED_TRANSFER_30MM.md) for the measured review and new instructions.

Original preparation notes follow.

**Preparation status before those runs:** No Gazebo experiment is
launched as part of preparation. The passing 5/10 mm results remain separate
evidence; the two 20 mm roadmap entries need their own measured runs.

## Run these two experiments

1. Refresh <http://127.0.0.1:8765/>.
2. Choose **40 mm lowered transfer: 20 mm left** in the roadmap.
3. Confirm lowering **40 mm**, transfer distance **20 mm**, direction **Left**.
   Leave Show the Gazebo simulation checked if you want the native view.
4. Click **Run lowered transfer** and allow about **12-13 wall minutes**.
5. Wait for the result and plot. Note the job ID.
6. Repeat with **40 mm lowered transfer: 20 mm right**, confirming **Right**.
7. Send the two job IDs after completion for log and measurement review.

The motion still lowers first, transfers sideways, returns to center while
lowered, then rises. Both feet remain planted and loaded. The GUI will save
the recordings, source hashes, summary, plot and launch log automatically.

## What this tests

The verified 10 mm transfers moved COM about 6.458 mm and produced a roughly
53.80/46.20 load split. The next increment doubles lateral pelvis travel.
The lateral movement duration also doubles from 16 to **32 simulated seconds
each way**, retaining the same slow reference speed scale.

The physical geometry, PID gains, motor caps, damping, contact/force guards
and numerical acceptance limits remain unchanged. This is a larger bilateral
transfer, not a foot-lift experiment.

## Calculated expectations - not measurements

| Quantity | Left test | Right test |
|---|---:|---:|
| Requested pelvis shift | +20 mm | -20 mm |
| Predicted COM shift | +12.899 mm | -12.899 mm |
| Predicted left-foot share | 57.588% | 42.412% |
| Predicted right-foot share | 42.412% | 57.588% |

The maximum static feedforward demand across the planned poses is about
**20.39%** of its configured motor cap. Actual commands also contain feedback
corrections. Static headroom does not replace the measured audits.

## Schedule after full standing readiness

| Phase | Simulated seconds |
|---|---:|
| Baseline | 0-3 |
| Lower 40 mm | 3-43 |
| Settle while lowered | 43-47 |
| Move sideways 20 mm | 47-79 |
| Hold shifted posture | 79-85 |
| Return sideways | 85-117 |
| Settle at center | 117-121 |
| Rise | 121-161 |
| Recovery | 161-169 |

Total motion time is **169 simulated seconds**, plus readiness. With the
0.25 real-time-factor target, the complete run usually needs 12-13 wall minutes.
The force/pose hold analysis uses 83-85 s, and final return uses 166-169 s.

## Code changes

| File | Change |
|---|---|
| `src/my_robot_description/my_robot_description/lowered_shift.py` | Adds ±20 mm to the allowed distances. Existing duration and table-spacing rules generate a 32 s lateral movement and 161 cached lateral poses. |
| `tools/force_panel/config.py` | Accepts 20 mm and converts distance/direction into the signed CLI value. |
| `tools/force_panel/jobs.py` | Adds separate 20 mm left/right roadmap entries. They remain ready to test until matching measured runs pass. |
| `tools/force_panel/static/index.html` | Adds the 20 mm choice; existing preview/timing/history code already handles the selected distance. |
| `scripts/run_lateral_shift.py` | Updates CLI help to document ±0.02 m support. |
| `tests/test_lowered_shift.py` | Extends both-sign geometry, COM, speed, torque, return and audit-rejection checks to 20 mm while retaining 5/10 mm checks. |
| `tests/test_force_panel.py` | Checks the 20 mm command and prevents passing 10 mm evidence from verifying 20 mm. |

Offline calculations and validation logs are in
`evidence/preparation/lowered-shift-20mm/`. The `planner-preview.json` file is
explicitly marked as an unmeasured prediction. The new stage is ready for
your experiments, not marked verified by these calculations.
