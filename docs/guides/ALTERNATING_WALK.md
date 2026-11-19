# Three alternating steps

This page documents the verified baseline. See [Longer and faster walking profiles](LONGER_FASTER_WALK.md) for the new configurable sequence.

This is a separate finite quasi-static walking experiment. It retains the verified first step, then alternates support and foot placement twice, ending in a stable double-support stop. It does not implement indefinite walking, turning, or recovery from pushes while walking.

## Geometry and progression

Initial feet are abreast. With left initial support, the swing order is **right → left → right**.

| Placement | Supporting foot | Moving foot | Moving foot travel | Landing ahead of support |
|---|---|---|---:|---:|
| 1 | Left | Right | 50 mm | 50 mm |
| 2 | Right | Left | 100 mm | 50 mm |
| 3 | Left | Right | 100 mm | 50 mm |

The final left foot is 100 mm ahead of its initial position and the final right foot is 150 mm ahead. At equal load, the COM projection should progress approximately 125 mm. All three swings use 30 mm sole clearance and the same 40 mm crouch.

The first placement follows the previously verified first-step trajectory. Before each subsequent swing:

1. Retain the two current world-frame foot positions.
2. Move desired COM from the feet's midpoint to the new supporting-foot center over 64 simulated seconds, reducing the other foot's planned load from 50% to zero.
3. Confirm unloading, then lift the trailing foot 30 mm, advance it 100 mm, and lower it 50 mm ahead of the support foot.
4. Confirm fresh landing contact before increasing its load.
5. Reload to 50/50, hold briefly, then exchange support roles for the next placement.

The supporting foot must remain fixed in world coordinates. Changing support changes which ankle receives the established single-support gain schedule. It does not change the gains' values, motor limits, physical damping or contact/geometry tolerances.

## Planning calculations

The existing flat-foot leg inverse kinematics calculates each five-joint leg independently from the requested world feet and pelvis translation. A two-variable root solve chooses pelvis X/Y to put COM over the desired support projection. The Jacobian uses fixed 1 µm differences, avoiding the solver's unreliable relative numerical differences at nearly zero lateral coordinates.

Vertical reactions sum to `mass × 9.81`. Their planned shares follow a quintic interpolation. Small common sole-pressure offsets ensure force-weighted pressure equals COM projection in both X and Y. Joint static torques include those pressure moments. Each sampled pose checks joint limits, flat-foot geometry, collision envelopes, at least 10 mm pressure margin and at least 20% motor reserve.

All phase references use the same quintic timing as the first step. Subsequent horizontal swings take 40 simulated seconds so that 100 mm foot travel stays under the existing 0.05 rad/s slow-reference bound. Every boundary checks continuity of both joint targets and feedforward torques.

## Timing

First step and hold: 0–241 simulated seconds after readiness. Each later placement adds 214 seconds:

| Phase | Duration |
|---|---:|
| Transfer support | 64 s |
| Confirm unloaded | 6 s |
| Lift | 15 s |
| Advance trailing foot | 40 s |
| Lower foot | 15 s |
| Confirm landing | 2 s |
| Reload | 64 s |
| Double-support hold | 8 s |

The canonical complete walking timing is stored in `motion_plan.steps` (three placements with absolute phase boundaries). The original base program schedule describes the first-step prefix only. The new profile wrapper exports the complete time-mapped schedule.

Total: **669 simulated seconds**. The GUI targets real-time factor 1.0 for this mode. This changes wall-clock pacing only: the physics timestep, controller periods and simulated trajectory timings are unchanged. Historical modes retain their default 0.25 factor. Freshness and recording coverage must still pass; faster pacing does not excuse missing evidence.

## Independent measured audit

The audit replays raw physics-stamped contact states together with independently observed sole corners and COM. It verifies each swing separately, rather than treating the successful first step as proof that later steps worked.

Each placement must show fresh no-contact evidence while raised, at least 29.5 mm measured clearance during horizontal motion, at least 10 mm COM margin inside the supporting sole, landing contact before reload, the correct final foot travel within 1 mm, and final 50/50 loading within two percentage points. Live and replayed foot tracking must stay within 0.5 mm vertically and 1 mm horizontally throughout the trial.

Controller equation replay, joint-speed and torque bounds, raw IMU/pose, sensor freshness, COM coverage, physical damping and unchanged-source checks remain required. Unknown or stale contact data never means unloaded.

## Files

- `alternating_walk.py`: new alternating stance planner, world-foot references, support-aware gains and guards.
- `alternating_walk_audit.py`: per-placement measured evidence and whole-trial audit.
- `preview_alternating_walk.py`: offline complete-path preflight.
- `plot_alternating_walk.py`: both feet, clearance, load exchange and forward progress.
- `run_lateral_shift.py`, `hold_roll_stance.py`, `roll_standing.launch.py`: separate walking selection and configurable wall pacing.
- `tools/force_panel/`: walking preview/run modes, prerequisites, progress, results and roadmap.
- `show_center_of_mass.py`: buffers every independent physics observation separately from marker drawing.
- `zero_load.py`: contact-pair age comparison at integer-nanosecond resolution.
- `reaudit_alternating_walk.py`: read-only saved-data replay with strict original-source checks.
- `render_first_step_replay.py`: also reconstructs a walking replay from measured joints and independently observed sole geometry; not Gazebo screen capture.
- `tests/test_alternating_walk.py`: support changes, retained landing positions, phase continuity, mirrored support inside the sequence and rejection of incorrect contact/position evidence.

Package Python modules are under `src/my_robot_description/my_robot_description/`; executable runtime controller is under `src/my_robot_description/scripts/`; standalone tools are under workspace `scripts/`.

## GUI reproduction

Choose **Three alternating steps OFFLINE preflight** and run it. After it passes, choose **Three alternating steps measured**, keep initial support **Left**, enable **Show the Gazebo simulation**, and click **Run three alternating steps**.

The GUI requires a measured first step on the selected initial supporting side, followed by the full walking preflight. Current measured first-step evidence is for left initial support.

## Status

Implementation and offline planner checks are complete. The GUI offline preflight `20261113-175259-0c7ed70d` passed. Focused tests passed: two walking tests, two first-step regressions, 31 panel tests, two exact timestamp-boundary tests, three read-only re-audit tests, and two COM-buffer tests.

The first live trial `20261113-175441-03995179` physically completed all three steps and stopped cleanly. Its initial audit hit a floating-point boundary error: exactly 30 ms was represented as slightly over 30 ms. `zero_load.py` now compares integer nanoseconds, preserving exactly the same inclusive limit; a 30 ms + 1 ns test is rejected.

Read-only GUI re-audit `20261113-181336-946d2b4e` preserved the original recording and confirmed all three placements, controller, sensor and damping checks. It also found a genuine 125 ms gap in the independent COM log, exceeding the existing 100 ms bound. Therefore that recording is **not a fully verified walking trial**.

The observer `show_center_of_mass.py` previously recorded only the latest physics state after drawing markers. It now buffers every independently received physics state and flushes the buffer separately from visualization, retaining original timestamps. This prevents slow drawing from discarding measurement samples. No controller or physics limits were relaxed. The repeat trial below verified the complete recording path.

A separate GUI **Re-audit saved walking run** action preserves the failed original report. Its source check allows only the precise documented post-run timestamp fix, and rejects unrelated motor/control/observer changes. It never starts Gazebo or sends robot commands.

## Verified result — 13 November 2026

Repeat GUI recording **`20261113-181745-9d3ab96d` passed all six audits**: controller, contacts/geometry, sensors, COM, physical damping, and all three alternating placements. Runtime source hashes match, final gains returned to their base values, and Gazebo/controller/observer exited cleanly.

| Placement | Swing foot | Measured foot travel | Minimum airborne clearance | Final swing-foot load share |
|---|---|---:|---:|---:|
| 1 | Right | 50.004197 mm | 29.993590 mm | 50.017368% |
| 2 | Left | 100.007377 mm | 29.999139 mm | 50.018092% |
| 3 | Right | 100.010712 mm | 29.999142 mm | 50.018073% |

Whole-trial measurements:

- COM forward progress: **125.008761 mm**.
- Minimum supporting-sole COM margin during the sampled horizontal swings: **59.407380 mm**.
- Maximum sole-height tracking error: **0.146332 mm**.
- Maximum foot XY tracking error: **0.163530 mm**.
- Peak joint speed: **0.035870 rad/s**, below the unchanged 0.05 rad/s audit bound.
- Peak commanded motor utilization: **53.296758%** of the existing caps; zero saturated recorded samples.
- Final three-second peak joint speed: **0.000005577 rad/s**.
- Controller torque-composition replay error: zero.

Raw evidence and summary are in `evidence/gui/20261113-181745-9d3ab96d/`. `response.png` plots both foot advances, measured clearance, measured load exchange and COM progress. `walking-replay.gif` reconstructs the motion from recorded joints and independent physics-observed sole corners. It is schematic, time-compressed, and not a Gazebo screen recording.

This is one fully verified finite three-placement quasi-static walking trial, starting with left support. It is not a claim of indefinite or disturbance-robust walking. The earlier trial and its failed/re-audited reports remain preserved as evidence of the logging defects and their fixes.

Next development: extend the tested sequence length and then reduce transfer/swing durations with new measured validation. Turning, terrain adaptation and pushes during walking remain separate future work.

Final COM coverage details:

```json
{
  "passed": true,
  "samples": 32403,
  "max_gap_s": 0.02100000000007185,
  "first_sim_time": 101.01,
  "last_sim_time": 769.987,
  "mass_kg": 22.28,
  "mass_variation_kg": 0.0
}
```
