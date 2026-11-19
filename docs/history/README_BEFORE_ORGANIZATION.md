> **Latest verified milestone — 13 November 2026:** three alternating simulated steps (right → left → right) passed all six audits, with 30 mm clearance, 125.009 mm COM forward progress, stable double-support stop and no motor saturation. The independent COM recorder now buffers physics observations separately from marker drawing. See [the walking guide](docs/ALTERNATING_WALK.md). This is a finite slow gait; longer/faster walking and disturbance recovery remain future work.

> **Latest measured milestone — 13 November 2026:** one 50 mm forward step with 30 mm clearance passed through the GUI (left support/right swing), including fresh airborne contact evidence, forward landing and 50/50 reload in a new staggered stance. Measured advance 50.004 mm, clearance 29.994 mm, peak motor utilization 53.1%, no saturation. See [the first-step guide](docs/FIRST_FORWARD_STEP.md). Continuous alternating walking is next; mirrored live stepping remains untested.

> **Latest tested clearance — 11 November 2026:**
> [Calculated 30 mm foot lift](docs/STEP_CLEARANCE_30MM.md) passed with left
> support, including contact loss, landing and return. The 50 mm forward swing
> is checked offline; executed forward placement remains the next development boundary.

> **Current milestone — 11 November 2026:**
> [Guarded 2 mm foot lift and landing](docs/LIVE_FOOT_LIFT.md) passed on both
> supporting sides using identical package sources. Both GUI roadmap entries
> are verified. Forward placement, alternating steps and walking remain future work.

> **Latest measured milestone — 11 November 2026:**
> [First guarded 2 mm foot lift](docs/LIVE_FOOT_LIFT.md) passed with left support,
> including fresh contact loss, landing and full return. Right support is ready
> to test separately. Forward stepping and walking remain future work.

> **Next-stage preparation — 11 November 2026:**
> [First 2 mm foot-lift offline preview](docs/FIRST_FOOT_LIFT_PREVIEW.md) is available in the GUI.
> It checks geometry and single-support static torques; no live lift has been executed.

> **Current technical handoff — 11 November 2026:**
> [Full architecture, verified results, limitations and next step (Greek)](docs/TECHNICAL_HANDOFF_2026-11-11.md).
> Current development uses the ten-joint P54 model. Both zero-load transfer/return trials passed;
> deliberate foot lift and walking remain unimplemented. The six-joint baseline below is historical.

# Humanoid standing baseline

For the current file structure, startup stages, troubleshooting and development commands,
read [the architecture guide](docs/ARCHITECTURE.md).

The robot description is now composed from `urdf/components/` files. Launch code is
split into helper modules under `src/my_robot_description/my_robot_description/`.
The calculated bent-knee stance is documented in [Step 2: bent-knee standing](docs/BENT_KNEE_STANCE.md).
The pelvis IMU integration is documented in [Step 3a: pelvis IMU](docs/PELVIS_IMU.md).
Crouch startup reliability and joint diagnostics are covered in [the tracking investigation](docs/CROUCH_TRACKING_DIAGNOSIS.md).
The foot sensor integration is documented in [Step 3b: foot contacts](docs/FOOT_CONTACTS.md).
The first disturbance baseline is documented in [Step 4: push experiment](docs/PUSH_EXPERIMENT.md).
Finite-effort joint control begins in a separate [Step 5a: torque bench](docs/TORQUE_BENCH.md).
Next, [Step 5b: standing torque calculations](docs/STANDING_TORQUES.md) explains the full robot's static loads before changing its controller.
The separate [Step 5c: effort-controlled posture](docs/EFFORT_STANDING.md) applies bounded torque with joint-angle feedback and records a checked standing experiment.
The [Step 5d: gentle effort-mode push](docs/EFFORT_PUSH.md) compares a 1 N disturbance with a fresh no-push reference and checks whether the original posture returns.
The [Step 5e: ankle diagnosis](docs/ANKLE_DIAGNOSIS.md) audits your recordings against controller terms and a conditional static gravity/friction calculation.
The [Step 5f: physics torque audit](docs/PHYSICS_TORQUE_AUDIT.md) closes the previously unexplained moment using Gazebo inputs, DART implicit damping and joint constraints.
The [Step 5g: timestep comparison](docs/TIMESTEP_COMPARISON.md) compares 1 ms and 0.5 ms physics steps with fixed controller settings and checks the actual push inputs.
The [Step 5h: repeated resolution study](docs/RESOLUTION_STUDY.md) adds 0.25 ms and compares resolution changes against two-run variation at each step.
The [bounded PID recovery guide](docs/RECOVERY_CONTROL.md) explains the explicit damping model, integral controller, before/after code and reproduction commands.
The [Step 6a: IMU balance observer](docs/BALANCE_OBSERVER.md) calculates and plots hypothetical ankle corrections while the existing joint PID remains in control.
The [Step 6b: active IMU ankle feedback](docs/ACTIVE_BALANCE.md) enables a gated, ramped ankle correction while retaining total motor caps and combined anti-windup.
The [Step 6c: balance repeatability check](docs/BALANCE_REPEATABILITY.md) compares repeated off/on tests with fixed gains and reports individual results and observed variation.
The [Step 7a: pulse-duration comparison](docs/PUSH_DURATION_COMPARISON.md) increases nominal impulse by 50% at fixed force and compares fresh reference/push cases.
The [Step 7b: forward duration and repeat study](docs/DURATION_REPEAT_STUDY.md) checks both directions and reversed duration order without changing the controller.
The notes below describe the original recovery and standing milestone.

## Initial recovery

Recovered from `/home/parispoll/robot_ws/src/my_robot_description` on 2026-10-26.
The originals are preserved in `/home/parispoll/Documents/ChatGPT/humanoid-recovery-2026-10-26/robot_ws`. This workspace contains the six-joint biped,
not the separate iCub model or the unrelated ros2_control demos in humanoid_ws.

## Run

```bash
cd '/home/parispoll/Documents/ChatGPT/robotics carrer'
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
export ROS_DOMAIN_ID=42
export GZ_PARTITION=biped_recovery
ros2 launch my_robot_description standing.launch.py gui:=true
```

Omit `gui:=true` to run headless. Use the same ROS_DOMAIN_ID and GZ_PARTITION
in other terminals. Stop with Ctrl+C.

The initial command is a straight, symmetric stance: all six joint positions zero.
Gazebo's position interfaces hold the joints; the trajectory controller accepts
subsequent commands. Gravity is enabled, the base is free, and the feet contact
the ground. This establishes ideal position-controlled standing, not torque-limited
actuation, disturbance rejection, walking, or feedback balance.

## What was repaired

- Replaced Classic `gazebo_ros2_control` with Harmonic `gz_ros2_control`.
- Replaced Classic spawn/launch and world plugins with Gazebo Sim equivalents.
- Added simulator plugin search paths and a simulation clock bridge.
- Start controller spawners after the robot creation process exits, with service timeout.
- Use a temporary controller YAML path without spaces for Gazebo's argument parsing;
  remove it on launch shutdown.
- Install only directories that exist (the original CMake referenced a missing rviz directory).
- Removed misleading trajectory PID gains: this baseline uses a position command interface.
- Excluded the old repeatedly published asymmetric bent-leg trajectory.

## Verify

With the simulation running and matching environment variables:

```bash
ros2 control list_controllers
python3 scripts/check_standing.py
```

The pose check samples 30 simulated seconds and requires the model origin to stay
above 0.50 m and tilt below 0.15 rad throughout. Use `timeout 95` around the script
if the simulator may be disconnected. Evidence is saved under `evidence/`.

## Next development steps

1. Add IMU and foot contact measurements, then record tilt, contact and joint tracking.
2. Establish a symmetric, slightly bent stance with appropriate initial joint positions.
3. Move to bounded effort control and validate actuator torque requirements.
4. Add feedback for sagittal balance; test small disturbances quantitatively.
5. Add lateral hip/ankle degrees of freedom before lateral balance and walking.

The current model has only pitch joints, so it cannot actively correct lateral tilt.
Reviewed the beginning and final standing discussion of the shared conversation
[Next steps for biped robot](https://chatgpt.com/share/6adf1146-cc58-83eb-910f-3b1969ebcdfb).
It confirms backward tipping and pose tuning were unresolved. In this URDF all leg
axes point along +Y: for an upright pelvis, flat feet require hip + knee + ankle = 0.
The suggested historical reset [0.05, 0.45, -0.10] sums to 0.40 rad (about 23 degrees),
so it cannot produce flat feet with an upright pelvis. A torso pitch joint is not
a prerequisite for the straight static stance demonstrated here.

Validation: colcon build passed; both controllers activated; 1,768 pose samples
over 30.012 simulated seconds passed, with base height 0.564999 m and no measured
tilt. A final 15-second startup/shutdown run also activated both controllers and
exited cleanly when interrupted. The GUI option is provided but has not been
visually tested. See evidence/standing-check.json and evidence/final-launch.log.

Gazebo integration reference: https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html


## Open in VS Code

Open `/home/parispoll/Documents/ChatGPT/robotics carrer` as the workspace folder.
Start with `src/my_robot_description/launch/standing.launch.py`, then inspect
`urdf/my_biped.urdf.xacro`, `config/controllers.yaml`, and `worlds/standing.sdf`
inside that package. Generated build/install/log directories are hidden by workspace
VS Code settings but retained so the tested setup can still run.

Retired robot_ws, humanoid_ws, contol_ws, and ros2_docker were consolidated into
`/home/parispoll/Documents/ChatGPT/humanoid-recovery-2026-10-26`.
The old ros2_ws remains in place because OS permissions prevented moving it.
Home-level build/install/log include other projects and were left alone.


## Local force-experiment panel

Run `./scripts/start_force_panel.sh`, then open http://127.0.0.1:8765.
Choose forward/backward force, magnitude, duration, existing IMU feedback and
Gazebo visibility. The panel runs the existing reference/push experiment and
shows progress, saved plots and history. Initial limits: 0.1–2 N and 0.02–0.30 s.
See [the GUI guide](docs/FORCE_PANEL.md) for usage, architecture and limitations.


## Ten-joint roll prototype: separate standing experiment

The original six-joint model and push GUI remain the baseline. A separate
hip/ankle-roll prototype now has verified two-foot effort standing. See
[the educational standing guide](docs/ROLL_STANDING.md) for the static-load
calculation, controller changes, timing fixes and measured evidence.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
python3 scripts/run_roll_standing.py --gui --output evidence/my-roll-standing-01
```

Use a new output directory. This records 30 simulated seconds after settling,
then closes the simulation; it is not a single-leg or walking experiment.

### Small lateral COM shift (both feet grounded)

The ten-joint model also has a verified ±10 mm pelvis-shift experiment, producing
approximately ±7.12 mm COM movement and a return to center. See
[the lateral COM shift guide](docs/LATERAL_COM_SHIFT.md) for the geometry,
shift-specific gain tuning, measured results and force-estimate limitations.

```bash
python3 scripts/run_lateral_shift.py --gui --displacement 0.01 \
  --output evidence/my-left-com-shift-01
```

Source ROS/workspace as above; use a fresh output folder. Negative displacement
runs the other direction. This is separate from the six-joint push website.

### Validate measured foot loads

The existing contact sensors provide usable per-point ground reaction forces.
The [foot-load guide](docs/FOOT_LOAD_VALIDATION.md) explains summation, frames,
source timing, weight/pressure checks and measured left/right load transfer.
Controller gains and geometry remain unchanged in this sensing milestone.

```bash
python3 scripts/run_lateral_shift.py --gui --foot-loads --displacement 0.01 \
  --output evidence/my-measured-left-loads-01
python3 scripts/analyze_foot_loads.py evidence/my-measured-left-loads-01
```

### Force-guarded partial unloading

See [the partial-unloading guide](docs/PARTIAL_UNLOADING.md) for the 60/40 and
65/35 derivation, measured force guards, joint targets and reproduction commands.
Both feet remain grounded; existing motor caps and posture gains are retained.

See [the unloading-limit analysis](docs/UNLOADING_LIMITS.md) for why the current
fixed-height motion reaches hip-roll torque and leg-extension limits before zero load.

The [posture and foot-pressure search](docs/SUPPORT_POSTURE_SEARCH.md) explores
lowered pelvis poses and optimized pressure locations before any new foot-lift trial.

The follow-up [lean and asymmetric-leg analysis](docs/LEAN_SUPPORT_SEARCH.md)
checks body rotation, independent leg IK and pressure-limited static transfers.

### Documented actuator upgrade

The [actuator assumptions and P54-class upgrade](docs/ACTUATOR_UPGRADE.md) explains
continuous versus stall torque, new motor masses/inertias, speed guards and validation.
Use `--actuator-profile robotis_p54` with the ten-joint standing or lateral-shift
runner. The default `legacy` profile preserves earlier experiments.

See [P54 weight transfer](docs/P54_WEIGHT_TRANSFER.md) for verified 70/30 transfers
in both directions and the model-mass audit correction with preserved raw evidence.

### Measured load-share correction

The [load-feedback guide](docs/LOAD_SHARE_FEEDBACK.md) explains the new bounded
force-to-posture outer loop, its failed initial tuning and the passing matched
reference-offset experiment. Enable it explicitly with `--load-feedback` on P54
partial unloading; `--load-reference-bias` is an optional validation input.

### Current P54 physical-pulse panel

Open http://127.0.0.1:8765/ after `bash scripts/start_force_panel_background.sh`.
The [physical lateral push and panel guide](docs/P54_LATERAL_PUSH_PANEL.md) covers
the current P54 mode, level front camera, verified 1 N / 0.2 s pulse and revised
P54-only roll velocity damping. Legacy modes remain separately selectable.
