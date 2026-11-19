> Historical overview: its original six-joint model and subsequent updates are not the current development snapshot.
> Read [the technical handoff dated 11 November 2026](TECHNICAL_HANDOFF_2026-11-11.md) for the ten-joint P54 architecture, measured zero-load results and proposed foot-lift stage.

# Humanoid project handoff: achievements, roadmap and learning goals

Workspace: `/home/parispoll/Documents/ChatGPT/robotics carrer`.
This file is a handoff for the separate educational chat, based on recorded
project results as of 2026-10-28. Read the implementation and linked guides when
building detailed explanations; distinguish measured results from future goals.

## What we are building and why

The original aim was to create a humanoid in ROS 2 and get it standing in Gazebo.
That standing milestone is achieved. The current model is a simplified biped:
a free pelvis/base, two thighs, two shins and two feet, with hip/knee/ankle pitch
joints on each leg. It has six actuated pitch joints, not a complete humanoid
with arms and lateral leg articulation. Total mass is approximately 14.1 kg.

The broader development direction is a modular, understood, reproducible
simulation that can stand, balance, recover from disturbances and eventually
progress toward controlled walking. Walking is a future aspiration, not an
achieved result. Physical hardware is not part of the currently validated work.
The user wants to learn the mathematics, ROS concepts and code through this
project, with small steps, sketches, before/after code and clear explanations.

## Achieved milestones

1. **Recovered and organized the project.** Migrated old Gazebo Classic launch,
   spawning and control integration to ROS 2 Jazzy / Gazebo Harmonic. Consolidated
   the working workspace and separated robot components, launch helpers,
   calculations, controller code and experiment tooling.
2. **Reliable position-controlled standing and a calculated crouch.** Established
   the initial standing baseline and symmetric flat-foot crouch. For an upright
   pelvis, hip + knee + ankle = 0. The usual crouch is approximately
   [-0.224080, 0.400000, -0.175920] rad on each leg.
3. **Measurements.** Added a pelvis IMU, joint feedback, body pose and separate
   foot-contact streams. Verified IMU at 100 Hz and fresh ground-contact
   evidence for both feet. Contact messages are not calibrated support forces.
4. **Repeatable push experiments.** Added fresh zero-force references, bounded
   disturbance requests, recording, quantitative recovery bands and plots.
   Staying upright and returning to the original posture are separate criteria.
5. **Finite-effort standing.** Computed static joint loads, tested effort
   interfaces and moved to bounded torque commands. Current hip/knee/ankle
   motor caps are ±6/10/6 N m per leg. Knee static feedforward is approximately
   -2.522 N m per leg in the usual crouch.
6. **Diagnosed residual lean.** Traced motor commands through Gazebo/DART and
   constraints. Identified strong timestep sensitivity associated with the
   backend implicit damping realization; did not assume the torque disappeared.
   Added optional native physics/DART observers and a separate explicit viscous
   damping plugin using the constrained joint velocity, retaining the original
   physical coefficients. Added bounded joint integral correction and
   anti-windup. Small-push recovery then passed.
7. **IMU balance observation.** Created pure pitch/rate and ankle-proposal math
   plus a read-only observer with no motor publishers. Verified signs,
   quaternion/gyro conversion, clipping and unchanged motor composition before
   enabling the correction.
8. **Optional active sagittal balance.** Added gated, ramped IMU ankle correction
   on top of the joint PID. Kept the total motor caps, added the correction to
   anti-windup prediction, and compared feedback off/on with fresh references.
   Moved high-rate contact sampling out of the controller and removed normal
   recorder startup metadata RPCs from its live feedback path.

## Current controller, in plain equations

Joint error: `e = q_target - q`.

`motor_request = static_feedforward + Kp_joint*e - Kd_joint*q_dot + joint_I + balance`

`motor_command = clip(motor_request, ±motor_cap)`

Joint Kp: 80/100/60; Kd: 0.10/0.15/0.05; Ki: 80/100/60, in hip/knee/ankle
order. Each joint integral contribution is bounded to ±1 N m. It freezes
increments that would worsen saturation of the combined request.

For the optional body correction, with pitch in radians and pitch rate in rad/s:

`raw_per_ankle = (20*pitch + 4*pitch_rate)/2`

`balance_per_ankle = ramp * clip(raw_per_ankle, ±0.5 N m)`

Only the ankles receive this additional term. It requires fresh IMU data and
fresh ground-contact evidence for both feet. The ramp takes one simulated
second after posture readiness; recording waits until the ramp/settle period
has finished. Positive forward pitch needs positive ankle motor correction
because its reaction on the upper robot is opposite. The equal ankle split
assumes double support; it does not measure equal load sharing.

Passive physical damping remains separate from motor control:
`tau_passive = -b*q_dot`, with hip/knee/ankle b = 2.5/3.0/2.5 N m s/rad.

## Latest verified results and their limits

The authoritative matched study is
`evidence/active-balance/paired-04/validation.json`. It contains four fresh push
trials (forward/backward, feedback off/on), each with its own fresh no-push
reference. Source/configuration hashes stayed fixed and settling times matched.

| Requested push | Peak pitch off → on | Reduction | Last outside any band off → on |
| --- | --- | --- | --- |
| +2 N for 0.2 s | 0.335° → 0.286° | 14.6% | 0.712 s → 0.988 s |
| -2 N for 0.2 s | 0.486° → 0.390° | 19.7% | 4.198 s → 0.706 s |

All passed recovery, fresh foot-contact, command/gate and saturation checks.
Forward return was slightly slower despite the lower peak; backward return was
faster. These are one off/on pair per direction, not a general stability proof.
Requested pulses and published windows matched, not independently measured
actuation impulses. Torque traces are commands, not calibrated motor torque.

`evidence/active-balance/standing-30s-01/summary.json` passed a separate 30 s
active standing check, with no saturation and stable height. Maximum IMU tilt
was approximately 0.00484°. Build and 119 unit tests passed. The combined
summary is `evidence/active-balance/validation.json`.

The earlier joint-PID recovery was also checked at 1, 0.5 and 0.25 ms physics
steps. Do not attribute that three-step study to the later active-IMU controller;
the latest active comparison used the normal 1 ms physics step.

Known limits: pitch-only legs; no lateral balance, walking, stepping, terrain
adaptation or physical hardware validation. Fresh collision evidence does not
prove support-force distribution. One shutdown log includes ROS subscription
teardown exceptions after requested SIGINT, after recording validation; raw
logs are retained. Earlier failed development runs are excluded from the final
matched comparison and explained in the active-balance guide.

## Recommended next development stages (proposals, not completed work)

1. **Educational consolidation now.** Explain what exists, the equations, units,
   coordinate signs, ROS topics, startup and closed-loop data flow. Draw diagrams
   from the actual source. Keep educational work separate from changing control.
2. **Repeat and characterize the current controller.** Repeat paired trials;
   inspect gain tradeoffs, measured/observed force timing, saturation headroom,
   longer runs, timestep sensitivity of active balance and realistic sensor
   delay/noise. Fix reproducible timing/teardown problems without hiding errors.
3. **Improve support-state estimation.** Validate foot forces/contact moments,
   center of mass and support geometry before claiming force balance, center of
   pressure or ZMP control.
4. **Controlled weight transfer.** Start with double-support sagittal motions,
   planned joint references and bounded effort; then assess unloading one foot.
5. **Extend the physical model for lateral control.** Add appropriate hip/ankle
   roll and other needed degrees of freedom, with realistic inertias, actuator
   limits and new validation. The present six pitch joints cannot actively
   stabilize lateral lean.
6. **Single support and stepping.** Develop foot placement, swing trajectories,
   contact transitions and recovery criteria; progress toward slow flat-ground
   walking only after the prerequisites pass.

Long-term success means a simulated biped that behaves under finite actuator
limits, has quantitative evidence for its balance/stepping behavior, is easy to
diagnose and can be explained by its owner—not just a model held by ideal
position interfaces. Revisit the final scope with the user before any hardware
or substantially larger humanoid build.

## Reading order and code map for the educational flowchart

Start with `README.md`, `docs/ARCHITECTURE.md`, `docs/STANDING_TORQUES.md`,
`docs/RECOVERY_CONTROL.md`, `docs/BALANCE_OBSERVER.md`, `docs/ACTIVE_BALANCE.md`.

Package root: `src/my_robot_description/`.

- `urdf/effort_standing.urdf.xacro` plus `urdf/components/`: physical model,
  inertias, joints, sensors, effort interfaces and damping plugin.
- `launch/effort_standing.launch.py`: configuration, simulator, robot spawning,
  bridges, controller startup, optional observers/contact sampler and shutdown.
- `my_robot_description/description.py`, `simulation.py`, `sensors.py`,
  `controllers.py`, `startup.py`: separated launch responsibilities.
- `my_robot_description/stance.py`, `statics.py`: geometry, desired crouch and
  static feedforward calculations.
- `scripts/hold_effort_stance.py`: the actual torque-command publisher, startup,
  IMU/joint/contact callbacks, composition, guards and logging.
- `my_robot_description/posture_effort.py`, `integral.py`, `torque.py`: joint
  error, P/D terms, bounded integration/anti-windup and final clamp.
- `my_robot_description/balance.py`, `balance_control.py`: pitch/rate conversion,
  candidate law, fresh-support gate and ramp.
- `scripts/sample_contacts.py`: latest original contact messages forwarded at
  50 simulated Hz, keeping their original timestamps.
- `scripts/observe_balance.py`: diagnostic-only IMU proposal recording. It does
  not command motors even when a separate active controller is enabled.
- `my_robot_description/feedback_timing.py`, `posture_setup.py`: timestamp policy
  and controller-written startup targets/caps for the recorder.
- `src/viscous_damping.cpp`: separate per-physics-step passive damping.
- `config/effort_standing.yaml`, `balance_control.yaml`, `balance_observer.yaml`:
  joint gains, applied body gains and independent observer settings.
- Workspace `scripts/run_effort_push.py`, `effort_push_experiment.py`,
  `push_experiment.py`: isolated simulation lifecycle, metadata, disturbance
  publication, measurements, recordings and quantitative recovery checks.
- Workspace `scripts/run_active_balance_comparison.py`,
  `compare_active_balance.py`: matched experiment orchestration and plots.
- `my_robot_description/balance_audit.py`, `effort_push.py`, and workspace
  `tests/`: verification, command accounting and regression tests.

Verify actual topic names and message types in source. Useful diagram layers:
(1) launch/startup sequence; (2) live closed-loop signal flow, including Gazebo,
bridges, IMU/joint feedback, contact sampler, controller and effort interface;
(3) controller math showing feedforward, joint PID, IMU correction, anti-windup,
final motor clamp and separate passive damping; (4) experiment/logging/audit
path. Distinguish ROS topic arrows from launch dependencies, local function
calls and file reads/writes. Avoid one unreadable diagram of every function.


## Update: Step 6c repeat consistency

The engineering chat subsequently completed a reversed-order repeat with the controller, model, gains and limits unchanged. There are now two off/on pairs per direction: forward peak reduction 14.53–14.65%, backward 19.38–19.71%. All eight push trials and their eight fresh references passed the recorded checks. The slower-forward/faster-backward return tradeoff persisted. Build and 123 unit tests passed. This is descriptive repeat evidence, not a general stability proof. See `docs/BALANCE_REPEATABILITY.md` and `evidence/active-balance/repeat-study-01/repeat-study.json`. Runtime control flow is unchanged; only experiment/statistics tooling was added.


## Update: separate force-experiment GUI

A local browser panel now wraps the existing paired effort runner. Launch with
`./scripts/start_force_panel.sh`; see `docs/FORCE_PANEL.md`. The form selects
world-frame +X/-X, 0.1–2 N, 0.02–0.30 s, existing IMU feedback on/off and Gazebo
visibility. The backend owns one experiment, handles cancellation, saves history
and displays recorded results. Control/model/gain sources are unchanged.
The runner gained an optional ROS-domain argument (default 57); the GUI uses 58.
For educational diagrams, add a browser → API → job manager → existing runner
layer above the existing experiment flow, without connecting the GUI directly
to joint-command topics. Startup clock-guard failures remain visible and saved;
the GUI does not solve or conceal that intermittent controller startup issue.


## Update: startup clock synchronization

`hold_effort_stance.py` now gates effort activation on initialized ROS time and
fresh joint timestamps via `startup_timing_ready()` in `feedback_timing.py`.
This addresses the intermittent zero-clock/future-feedback startup failure.
The active guard, gains, balance law and model are unchanged. The runner now
surfaces child exceptions with `experiment_failure.launch_failure()`. The full
suite has 141 passing tests. See `docs/STARTUP_CLOCK_FIX.md` and
`evidence/startup-clock/validation.json`; old source hashes are historical and
a new matched study must capture the new startup-source hashes.


## Update: visual force arrow

The shared `scripts/push_experiment.py` recorder now drives the optional
`my_robot_description/force_visual.py` GUI marker worker from push/clear events.
It draws a world-X box/cone arrow beside the pelvis, follows model position,
and clears only its own marker namespace. It never publishes forces or motor
commands and never waits for a renderer on the force loop. Controller/model
sources and force parameters are unchanged. See `docs/FORCE_ARROW.md`.


## Step 7a: pulse-duration characterization

A new comparison runner tests backward 2 N at 0.20 s and 0.30 s, a 50%
increase in nominal requested impulse inside existing force/duration limits.
Each case gets fresh reference/push worlds; source snapshots are enforced.
Recovery failure stops escalation; infrastructure failure remains an invalid
experiment, with no automatic retry. Gains, caps, model and timing guards are
unchanged. See `docs/PUSH_DURATION_COMPARISON.md` and
`evidence/duration-comparison/backward-01/comparison.json` for current outcomes.


## Step 7b: forward characterization and reversed-duration repeats

The duration runner now accepts `--direction forward|backward` and
`--reverse-order`, retaining backward/.20-then-.30 defaults. The separate
`run_duration_repeat_study.py` verifies the completed direction seeds, then
runs fresh reversed-order comparisons in each direction.
`duration_repeat_analysis.py` checks raw audits/source snapshots, prevents
double-counting and describes two observed repeats per direction;
`plot_duration_repeats.py` plots the observed values/ranges. The full suite
passed 156 tests. All engineering changes are workspace analysis/orchestration
code; the robot package is unchanged. See `docs/DURATION_REPEAT_STUDY.md`,
`evidence/duration-comparison/forward-valid-01/comparison.json` and
`evidence/duration-comparison/repeat-study-01/repeat-study.json` for outcomes.
One earlier invalid forward reference (`forward-01`) is preserved and excluded.


Step 7b completed: eight valid pushes and eight references passed. The longer
pulse increased peak lean in both repeats in both directions: forward paired
increase 45.74–46.94%, backward 46.41–47.67%. No saturation or measurement-gap
freezes occurred; contact evidence and source snapshots remained valid. One
additional forward reference with a timing error was preserved and excluded.
See `docs/DURATION_REPEAT_STUDY.md` and the repeat-study verification JSON.


## Update: observer first-IMU discovery

A GUI startup failure with an empty observer recording exposed an overly short
first-message deadline. `observe_balance.py` now uses the separate pure
`imu_watchdog.check_imu_watchdog()` policy: ten wall seconds for first valid
IMU discovery, then the unchanged two-second stream-loss deadline. The active
controller/gains/physics/measurement freshness checks are unchanged. Build and
162 tests passed; a real ROS delayed-discovery/stream-loss probe passed. See
`docs/IMU_DISCOVERY_FIX.md` and `evidence/imu-discovery/validation.json`. The
observer source snapshot intentionally changes; old studies remain historical.


The follow-up joint startup correction now discards out-of-age samples only
before readiness, with no torque/integral/timestamp/watchdog update. Fresh data
restarts the full settling window; backwards/nonfinite time and ready-state
timing failures remain fatal, and prolonged loss still trips the wall watchdog.
The timing helper's new startup option defaults off. Full suite: 169 tests.
This intentionally changes startup-source hashes; gains, limits and physics
remain unchanged. See the joint-startup section of `docs/IMU_DISCOVERY_FIX.md`.


The full startup correction also moves the hardware-interface query and
recorder into warmup. The shared recorder supports an optional ready-file gate;
the effort runner opens it only after existing readiness/settling checks, then
the normal fresh-baseline window runs. Connection/arm/start times are saved.
Ready-state age limits and measurement-gap audits remain strict. Diagnostics
prefer the triggering exception over secondary cleanup errors. Full suite:
173 tests. See the latest sections in `docs/IMU_DISCOVERY_FIX.md` and its final
GUI validation artifact for actual success/failure evidence.


The final GUI check `20261028-170156-30c2ef7d` passed both trials at backward
2 N / 0.20 s with Gazebo enabled. It exercised the delayed first IMU (2.899 s)
and startup feedback rejections (one in reference, three in push), then completed
full settling and all audits. Connection/arm/recording timestamps were ordered
correctly. No measurement-gap freezes or saturation. Full suite: 173 tests.
Source snapshots and preserved failed attempts: `evidence/imu-discovery/validation.json`.
