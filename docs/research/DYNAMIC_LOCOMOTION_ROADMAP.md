# Faster walking, squats and jumping: research and proposed development

Prepared 17 November 2026. This is a research/design proposal, not an implemented dynamic controller or a demonstration of new skills.

## User priorities

1. Faster walking first, then squats, then jumping.
2. Simulation first, using physically plausible actuator models.
3. Reinforcement learning may be developed alongside an explainable controller.

## What we have actually achieved

The saved run `20261114-105944-c4c3904f` verified ten finite quasi-static placements: 75 mm landing ahead of support, 30 mm sole clearance, 712.776 mm measured COM progression in 1,095 simulated seconds. Average progression was approximately 0.000651 m/s. This is a successful baseline, but not yet dynamic walking.

Interactive mode adds command-driven start, landing-boundary stop, hold and resume. Its later two-placement smoke tests are separate from the original full ten-step audit. Sensor-delivery and numerical unloaded-contact corrections are documented in `../guides/INTERACTIVE_SIMULATION.md`.

## Research references and what to learn

### Atlas: optimized motion plus online correction

Boston Dynamics describes parkour motion templates generated using offline trajectory optimization and adapted online using model predictive control. We should adopt the separation between feasible motion planning and feedback correction, rather than copying Atlas geometry or assuming current commercial Atlas uses exactly the same stack as that historical demonstration.

Source: [Boston Dynamics, Flipping the Script with Atlas](https://bostondynamics.com/blog/flipping-the-script-with-atlas/).

### Cassie: two useful controller paths

The University of Michigan controller combines an angular-momentum inverted-pendulum MPC foot-placement planner with a gait controller. This is a useful explainable reference for choosing future footholds from motion state. Cassie's different feet, compliance and actuation prevent direct transfer of its gains or equations without adaptation.

Source: [UMich BipedLab cassie_alip_mpc](https://github.com/UMich-BipedLab/cassie_alip_mpc).

A separate Cassie study demonstrated real-world walking, running and jumping using reinforcement learning with short/long input-output history and task randomization. This supports an RL research track, not a claim that a policy trained on Cassie will control our model.

Source: [Li et al., Reinforcement Learning for Versatile, Dynamic, and Robust Bipedal Locomotion Control](https://arxiv.org/abs/2401.16889).

### iCub: motor power and momentum matter

The iCub jumping study compared optimized velocity and torque control. Both jumped in simulation, while only velocity control achieved jumping on the real robot. Its findings emphasize centroidal angular momentum, actuator power and low-level performance. This is an important warning against judging jump feasibility from static torque alone.

Source: [Bergonti et al., Torque and velocity controllers to perform jumps with a humanoid robot](https://arxiv.org/abs/2207.01026).

### Dynamic actuator architecture

The ODRI architecture paper describes a torque-controlled brushless motor with low-ratio transmission and demonstrates force/impedance control on a quadruped. The associated hardware repository also contains the Bolt biped. We should study this actuator architecture; that paper is not evidence that its quadruped or an unchanged Bolt is equivalent to our flat-foot biped.

Sources: [ODRI architecture paper](https://arxiv.org/abs/1910.00093), [official hardware repository](https://github.com/open-dynamic-robot-initiative/open_robot_actuator_hardware).

### Squat planning

A 2025 study uses three-particle MPC and whole-body control for squat tracking. Its reported squat comparison is in simulation, so it should be treated as an algorithm reference, not a verified physical-robot squat benchmark.

Source: [Squat Motion of a Humanoid Robot Using Three-Particle MPC and Whole-Body Control](https://www.mdpi.com/1424-8220/25/2/435).

## Current model audit

From the local URDF and actuator configuration:

- Ten leg joints: hip roll/pitch, knee pitch, ankle pitch/roll on each side; no hip yaw or actuated waist.
- Thigh and shin: 0.26 m each; feet: 0.20 × 0.12 m.
- The existing 7.5 kg base is already a lumped upper-body/pelvis mass, not a massless placeholder.
- P54 profile total modeled mass: approximately 22.28 kg.
- Per-leg torque caps: 44.7, 25.3, 44.7, 25.3, 25.3 N m; modeled output-speed guard: 2.4 rad/s.
- The manufacturer lists the PH54-100 continuous operating speed as 29.2 rpm and torque as 25.3 N m. Those operating specifications are not a complete torque-speed, impact or thermal model.

Source configuration: `src/my_robot_description/config/roll_actuators.yaml`, `urdf/components/properties.xacro`, `urdf/components/base.xacro`, `urdf/components/leg_roll.xacro`.

Manufacturer reference: [ROBOTIS PH54-100-S500-R](https://emanual.robotis.com/docs/en/dxl/p/ph54-100-s500-r/).

Our conclusion is a hypothesis to test: existing geared motors may constrain dynamic speed and impact behavior, but the present evidence does not prove they cannot walk faster or make a small hop. A rectangular independent torque/speed limit cannot establish dynamic feasibility. The next audit must include torque-speed coupling, mechanical power, rotor/reflected inertia, damping, current/voltage constraints and consistent motor masses.

## Geometry recommendation

Keep the verified geometry for the first dynamic-control investigation. Preserve the baseline as an explicit model/profile if a second design is introduced.

1. Keep all ten existing joint axes and the current foot dimensions initially. Faster straight walking does not automatically require hip yaw or arms.
2. If a clearer torso is needed, split the existing base mass into pelvis and fixed torso links. Recompute combined COM and inertia; do not add another full torso mass on top. A fixed split adds no active control authority.
3. Evaluate one actuated waist-pitch joint only if the motion study shows a useful improvement in trunk posture or angular momentum. It adds mass and a new control task, so it is not an automatic upgrade.
4. Consider alternative motor/transmission profiles before changing leg lengths. Include the masses and inertias implied by any faster actuator. Moving actuator mass proximally requires an actual modeled transmission, not deletion of distal mass.
5. Add hip yaw when steering is a planned task. Add arms after the lower-body dynamic controller is understood.

## Proposed controller architecture

Command interface → motion/velocity planner → dynamic state and contact estimator → contact-aware phase supervisor → task/whole-body controller → actuator model → Gazebo.

Retain current joint feedback and effort interface. Add a separate dynamic controller mode so the quasi-static baseline stays reproducible. State should include floating-base pose/velocity, joint position/velocity, COM position/velocity, contact modes and, where needed, angular momentum. Use independent simulator state to validate the estimator rather than silently treating ground truth as an IMU-only estimator.

For explanation and initial planning, a constant-height linear inverted-pendulum approximation can introduce capture point:

`capture_x = com_x + com_velocity_x / sqrt(g / com_height)`.

This approximation is appropriate only under its height/angular-momentum assumptions. It cannot model squat/jump flight and impacts by itself. A task/whole-body controller must reconcile desired COM/body motion and foot motion with full dynamics, contact forces, friction, joint limits and actuator limits.

Phase changes should depend on confirmed unloading and touchdown, with timeout handling. They should not assume all clock-scheduled contact changes physically occurred.

## Why jumping requires a distinct mode

The current zero-load guard requires total vertical reaction within ±5% of body weight and a loaded supporting foot. It would intentionally reject meaningful takeoff acceleration and flight.

A future jump supervisor needs explicit phases: preparation/squat, push-off, flight, touchdown, landing absorption, recovery. Each phase needs physically justified force/contact conditions. Freshness, finite measurements and motor constraints remain essential; do not obtain flight by disabling all contact guards.

Illustrative calculation, not a feasibility result:

For mass 22.28 kg and COM apex rise 0.03 m above takeoff:

- Takeoff COM vertical speed: `sqrt(2 g h) = 0.767 m/s`.
- Added gravitational energy to the apex: `m g h = 6.56 J`.
- If vertical speed increases from zero to that value over 0.20 s, average vertical ground force is `m(g + v/T) ≈ 304 N`, or 1.39 body weights.

This simple point-mass estimate ignores joint geometry, initial countermovement, motor losses, leg motion and landing impacts. It is enough to show why the current ±5% weight band is unsuitable, but it does not prove the motors can deliver the required impulse. Foot clearance is not necessarily equal to COM apex rise.

## Development stages and acceptance evidence

### A. Dynamic feasibility study — immediate next task

Keep the URDF unchanged. Calculate joint velocity, acceleration, torque and power requirements for a family of shorter step cycles, then compare candidate motor envelopes. Establish a contact/estimation latency budget. Choose the first faster gait from these results; do not pick an impressive speed arbitrarily.

Deliver: reproducible calculations, plots, limiting joints and a justified first test target. The tool should distinguish rigid-body modeling assumptions from measured motor specifications.

### B. Explainable faster walking

Implement floating-base state estimation, online COM/footstep correction and a contact-aware step supervisor. Progress from isolated support transitions to repeated walking. Replace long static pauses with feasible, measured transitions. Validate commanded/measured speed, foot clearance, slip, contact transitions, tracking, actuator utilization and final stop across repeat runs.

### C. Parallel RL comparison

Define the same robot, actuator limits, sensor observations, action semantics and task commands for training. A faster training simulator may be useful; Gazebo remains an independent deployment check. Verify model parity before interpreting transfer results. Start by learning walking only, with randomized masses/friction/delays and explicit training/evaluation separation. Report falls and failed episodes as well as successful clips. The local machine reports an RTX 4050 Mobile GPU and about 16 GB RAM; training throughput and memory fit still require measurement.

Keep the explainable controller as the baseline. RL is not a replacement for identifying unrealistic actuator assumptions or mismatched simulation physics.

### D. Squats

Sweep feasible depth with planted feet, COM/body posture tracking, joint limits and torque/power checks. Start with a symmetric shallow squat and return, then increase depth and speed separately. A deeper URDF joint range does not establish collision-free or motor-feasible motion. The target depth is selected by calculations, not copied from another robot's video.

### E. Small two-foot hop

Only after takeoff and landing feasibility: select a small calculated COM rise, verify actual flight from independent contacts and geometry, then verify landing absorption and stable recovery. Measure force impulse, takeoff velocity, apex, contact timing and actuator peaks. A visually bouncing robot is not sufficient evidence of a controlled jump.

## Implementation status

This document contains research and proposals. No torso, new actuator, dynamic walking controller, RL policy, deep squat or jump was implemented or simulated for this research turn. Existing walking code and geometry were left unchanged.
