# Combining a lowered posture with lateral weight transfer

Update: 10 mm left/right transfers are prepared as the next user-run tests.
See [NEXT_LOWERED_TRANSFER_TESTS.md](NEXT_LOWERED_TRANSFER_TESTS.md) for settings,
timing, expected values and changed files. The 5 mm measurements below remain
the validated baseline; preparation does not validate the 10 mm extension.

This separate P54 experiment combines the measured 40 mm crouch with a
**5 mm pelvis translation**, keeping both soles fixed and flat. No foot is
lifted and no external force is applied. The controller remains joint PID
with static feedforward; measured contact forces supply the existing live
guard and independent audits, not a new load-share servo.

## Run everything from the GUI

Open <http://127.0.0.1:8765/>. In the roadmap choose **40 mm lowered transfer:
5 mm left** or its rightward counterpart. The selected mode is **P54 - 40 mm
lowered lateral transfer**. The depth is fixed at the validated 40 mm value;
choose the lateral direction, leave Gazebo checked, then **Run lowered
transfer**. The GUI locks settings, shows progress, stores history and plots
the results. Verification is recorded separately for the two directions.

| Phase | Program time, simulated seconds | Purpose |
|---|---:|---|
| Baseline | 0-3 | Record centered standing |
| Lower | 3-43 | Repeat the validated slow 40 mm lowering |
| Settle while lowered | 43-47 | Separate vertical and lateral transients |
| Shift sideways | 47-55 | Move the pelvis 5 mm with a quintic time phase |
| Hold shifted posture | 55-61 | Measure COM and actual load redistribution |
| Return sideways | 61-69 | Re-center without rising |
| Settle at center | 69-73 | Check the combined motion returns while lowered |
| Rise | 73-113 | Return to original standing height |
| Recovery | 113-121 | Check final return |

Full measured standing readiness comes before this timeline. At the 0.25
real-time-factor target, allow about 9-10 wall minutes including startup.

## The calculation behind the new motion

We reuse `LoweringProgram` for the vertical part. At the lowered height H,
each lateral table point first chooses hip roll:

```
q_roll = atan2(-pelvis_y, H - foot_thickness)
q_ankle_roll = -q_roll
```

The solver then chooses hip and knee pitch so that both foot frames remain
at ground height and the COM stays aligned with the soles in the fore/aft
direction. Ankle pitch cancels hip plus knee pitch. The small required
fore/aft pelvis translation is derived from forward kinematics, rather than
being assumed zero. Every joint limit and the fixed/flat-foot geometry is
checked before activation.

The whole-body COM displacement is **not** 5 mm just because the pelvis moves
5 mm. It is calculated from all link masses and their world positions. Under
the static sole-center pressure assumption:

```
F_left  = M g (0.5 + COM_y / 0.170)
F_right = M g (0.5 - COM_y / 0.170)
```

Static feedforward is recalculated for those forces. These predicted loads
are planning inputs, not substitutes for contact measurements. Actual loads
come from the separately recorded Gazebo contact wrenches.

## What must pass

The original checks remain: fresh contact at both feet, no motor saturation,
20% motor headroom, at least 25% measured load on each foot, total weight
within 5%, slow joint speed no more than 0.05 rad/s, planted-foot geometry,
tracking, source timing/coverage, IMU and physical damping consistency.

The lateral audit checks the sign and magnitude of measured COM and pelvis
motion and their return. The height audit checks the complete planned
lowering/rising reference, with at most 2 mm baseline-relative tracking error
and 1 mm final height/fore-aft offset. The force audit compares the measured
load split with the planned changing split (within 2 percentage points),
requires an actual held load change exceeding 1 N in the requested direction,
and checks force-weighted CoP against independently observed COM within 3 mm.

This last condition prevents a recording with pose motion but no measured
load transfer from being called successful. A passing result remains a
small bilateral transfer, not single support or walking.

## Files and separation of responsibilities

| File | Responsibility |
|---|---|
| `src/my_robot_description/my_robot_description/lowered_shift.py` | New offline combined-motion planner and phase schedule |
| `src/my_robot_description/scripts/hold_roll_stance.py` | Explicit planner selection; unchanged PID and bilateral force guards |
| `src/my_robot_description/launch/roll_standing.launch.py` | Pass the signed lateral setting |
| `scripts/run_lateral_shift.py` | New mutually exclusive `--lowered-shift` option and audit orchestration |
| `src/my_robot_description/my_robot_description/lowering_audit.py` | Optional planned load-split reference and measured-transfer requirement; centered experiments retain 50/50 reference |
| `tools/force_panel/config.py` | Fixed-depth, no-push request validation and signed runner command |
| `tools/force_panel/jobs.py` | Combined progress, completion, plotting and per-direction roadmap evidence |
| `tools/force_panel/static/index.html`, `app.js` | Mode, direction, results and roadmap controls |
| `scripts/plot_lowering.py` | Additional measured/planned lateral-position graph and planned left-load reference |
| `tests/test_lowered_shift.py`, `tests/test_force_panel.py` | Mirrored geometric/speed/headroom checks and rejection of missing measured transfer or invalid GUI settings |

Build, JavaScript syntax and 289 unit tests passed before the live run. The
physical URDF, motor caps, PID gains, damping model and existing numerical
acceptance limits were not changed. Current live evidence is saved by the GUI
under its job ID; the final result determines the roadmap status.

For diagnostic reproduction outside the GUI (fresh folder required):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/run_lateral_shift.py --gui \
  --actuator-profile robotis_p54 --lowered-shift 0.005 \
  --output evidence/my-lowered-left-transfer-01
```

Use `-0.005` for rightward motion. The normal user workflow is the GUI.

## Verified leftward run - 1 November 2026

GUI job **`20261101-191826-77207717`**, with Gazebo enabled, passed every
motion, independent sensor, physical damping, lowering/force and live guard
audit. Package hashes were unchanged throughout measurement.

| Quantity | Measured result |
|---|---:|
| Pelvis lowering | 40.0003 mm |
| Leftward pelvis displacement | 5.0131 mm |
| COM displacement relative to feet | 3.2333 mm (prediction 3.2249 mm) |
| Left-foot load-share change | +1.8992 percentage points |
| Maximum split error relative to planned split | 0.0204 percentage points |
| Maximum final height offset | 0.00245 mm |
| Maximum force-weighted CoP/COM separation | 0.0461 mm |
| Peak joint speed | 0.02234 rad/s (limit 0.05) |
| Saturated motor samples | 0 |

The load-share change is approximately a transfer from 50/50 to 51.90/48.10,
not a single-leg stance. These tiny errors are simulated numerical results,
not a claim of real robot precision. Both feet remained planted and the
combined posture returned to baseline before the run ended.

The `shift` sub-audit retains its predicted-force label; measured force
validation is reported separately in `lowering` and `load_guard`. No predicted
force was substituted for missing contact data.

The leftward roadmap entry is verified independently of the rightward entry.
The rightward planner passed offline mirrored tests but has not yet received
a live combined-motion validation. That mirrored GUI run is the next step.
