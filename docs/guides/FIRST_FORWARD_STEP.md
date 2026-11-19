> **Progress update, 13 November 2026:** the subsequent three-step alternating gait is now measured and verified. See [ALTERNATING_WALK.md](ALTERNATING_WALK.md). This document preserves the earlier single-step milestone.

# First forward step: 50 mm placement with 30 mm clearance

This stage replaces the vertical lift's return to the original feet positions with a forward landing and a new staggered stance. It is one deliberate foot placement, not continuous walking. The first tested direction uses left support and moves the right foot.

## Geometry and calculations

The selected swing foot advances 50 mm (+world X). Clearance remains the previously measured 30 mm. The original support foot stays at its original position, and pelvis height stays 40 mm below the standing pose during the step.

For each foot, convert its world target to the upright pelvis frame. With lateral hip offset `hip_y`, the roll angle and effective sagittal height are:

```
dy = foot_y - pelvis_y - hip_y
z = foot_z - pelvis_height
hip_roll = atan2(dy, -z)
sagittal_z = -sqrt(dy² + z²)
ankle_roll = -hip_roll
```

Existing two-link inverse kinematics calculates hip pitch, knee pitch and ankle pitch for the requested X and sagittal Z. The pitch angles sum to zero, keeping the sole flat. Every sample checks joint limits, fixed support-foot geometry, conservative collision envelopes and 20% static motor headroom.

Three separate smooth motions make the first trial easier to inspect: lift vertically, translate horizontally at full clearance, then lower vertically at the new X. For a phase fraction `s` in [0,1], use `u = 10s³ − 15s⁴ + 6s⁵`. This gives zero velocity and acceleration at the phase ends.

## Why landing requires new control references

Returning to the old zero-load program would command the forward foot back to its old location while it touches the ground. Instead, the new program retains both new foot locations and solves the pelvis X/Y position for a gradual COM/load transfer.

During reload, the desired landing-foot load rises from 0% to 50%. The desired COM moves from the single-support endpoint to the midpoint of the two foot centers. At the end, the COM projection is 25 mm ahead of the original foot-center line and laterally centered. Pelvis X is approximately 8.34 mm ahead of its initial position; COM and pelvis do not move by the same amount because the leg configuration changes.

Static reactions sum to `mass × gravity`. A small common offset of the two center-of-pressure positions makes their force-weighted center match COM X/Y, satisfying whole-body static moment balance. The offset must remain at least 10 mm inside each sole. These are planning estimates; fresh contact measurements must independently confirm the commanded load transfer.

## Schedule after measured standing readiness

| Simulated time (s) | Phase |
|---|---|
| 0–117 | Existing baseline, 40 mm lowering, lateral transfer and unloading |
| 117–132 | Raise right foot to 30 mm |
| 132–152 | Advance right foot 50 mm while raised |
| 152–167 | Lower at the new X position |
| 167–169 | Confirm fresh landing contact before reload |
| 169–233 | Move pelvis/COM and reload toward 50/50 |
| 233–241 | Hold the new staggered stance |

At 0.25 real-time factor, allow roughly 18–20 wall minutes including startup. This trial closes Gazebo after recording. It does not rise, return to the original feet, or attempt another step.

## Files and responsibilities

- `src/my_robot_description/my_robot_description/motion/first_step.py`: new geometry, static loads, phase references and live position/contact guards.
- `src/my_robot_description/my_robot_description/validation/first_step_audit.py`: replay raw contact states, independently observed sole geometry, sensors, motor equations and final load/position evidence.
- `scripts/planning/preview_first_step.py`: offline preflight; no simulation or motor commands.
- `scripts/plots/plot_first_step.py`: prediction or measured-response plot.
- `scripts/experiments/run_lateral_shift.py`: separate `--first-step` selection, shared recording/cleanup and step-specific audit routing.
- `src/my_robot_description/nodes/control/hold_roll_stance.py`: routes the new selection into the existing bounded effort controller.
- `src/my_robot_description/launch/roll_standing.launch.py`: new `first_step_enabled` switch and side camera for this mode.
- `tools/force_panel/`: separate GUI preflight and measured-step modes, prerequisites, history, results and roadmap.
- `tests/test_first_step.py`: both-side FK, continuity and guard rejection checks; GUI request/prerequisite checks.

The old zero-load and vertical-lift modes retain their original trajectories. The new step uses the existing zero-load gain schedule, effort caps, physical damping, IMU limits and sensor freshness bounds.

## Required evidence

A successful result must show all of the following:

- Planned and observed sole heights agree within 0.5 mm; XY tracking remains within 1 mm.
- The raised swing foot has fresh explicitly empty contact packets; the supporting contact remains loaded.
- Support COM margin remains at least 10 mm throughout swing.
- Landing contact exists before reloading.
- Final forward foot position is within 1 mm of 50 mm and landed-foot load share is within two percentage points of 50%.
- Joint speed stays below 0.05 rad/s, torque stays within the reserved 80% envelope, and recorded motor composition matches the controller equations.
- Independent pose/IMU, COM/contact coverage, physical damping and unchanged-source checks pass.

## Reproduce through the GUI

Open http://127.0.0.1:8765/. Choose **50 mm forward step OFFLINE preflight: left support** and run it. Once it passes, choose **50 mm measured forward step: left support**, enable **Show the Gazebo simulation**, then **Run one 50 mm step**.

The GUI requires a same-side measured 30 mm lift and a same-side step preflight. The right-support 30 mm live lift has not yet been measured, so the mirrored forward step is not yet enabled by the prerequisites.

## Validation status

- Build passed.
- Two new tests passed; the geometry test exercises both supporting sides.
- 31 existing panel tests passed.
- GUI offline step preflight `20261113-170406-dc5944b5` passed, including independent single-support gravity-gradient checks.
- First measured GUI step `20261113-170601-d4d3c645`: **passed all six audits**, completed with unchanged sources and clean Gazebo/controller/observer exits.
- Negative evidence check: the previous successful 30 mm vertical lift was independently replayed through the forward-step audit and correctly rejected (no 50 mm advance).

## Measured result — 13 November 2026

| Measurement | Result |
|---|---:|
| Final right-foot forward advance | 50.004228 mm |
| Minimum sole clearance during forward motion | 29.993588 mm |
| Final right-foot load share | 50.017367% |
| Minimum sampled COM margin during forward swing | 59.614672 mm |
| Maximum sole-height tracking error, whole trial | 0.295631 mm |
| Maximum foot XY tracking error, whole trial | 0.322808 mm |
| Maximum pelvis XYZ tracking error | 0.096377 mm |
| Peak joint speed | 0.035940 rad/s |
| Peak motor utilization | 53.060477% |
| Saturated recorded controller samples | 0 |
| Final load/stance audit samples | 150 |

Fresh empty contact packets confirm the raised foot was airborne. Fresh landing contact was confirmed before reloading. The final stance was held with both feet loaded, and all controller, contact/geometry, sensor, COM, physical-damping and forward-step audits passed. Motor command composition replay error was zero.

Raw evidence: `evidence/gui/20261113-170601-d4d3c645/`. The response plot is `response.png`. `step-replay.gif` is an educational reconstruction from recorded joint states and physics-observed sole corners; it is not a Gazebo screen recording. Time is compressed, and the torso is schematic.

**What remains:** the mirrored 30 mm lift and first step have not been tested live. Repeated walking, moving the trailing foot past the leading foot, unloading from the new staggered stance, and disturbance recovery during a step remain unimplemented. The next development task is a planner/controller that begins from the measured staggered stance, transfers support to the forward foot, and safely advances the trailing foot. Do not restart the original symmetric zero-load sequence from this final stance.
