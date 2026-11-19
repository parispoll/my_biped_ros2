# Faster walking: phase-specific timing tests

The verified 75 mm ten-placement baseline uses timing scale 0.5 for all movement phases and lasts 1,095 simulated seconds after readiness. Faster Gazebo playback is separate from faster robot motion.

The new fourth `WalkingProfile` setting is `transfer_time_scale`. Zero means inherit the movement scale, preserving all earlier profiles. An explicit value controls `move_out`, `transfer` and `reload`; swing/lowering phases use the main movement scale. Protected sensing and landing holds retain their minimum durations.

The first candidate uses 0.25 movement timing and 0.5 transfer timing. Three placements take 278.5 simulated seconds instead of 339 s; ten would take 905 s instead of 1,095 s. These durations are predictions until measured audits pass.

The 5 Hz sampled reference screen estimated peak joint speed 0.1492 rad/s and peak joint acceleration 0.1344 rad/s². This is a kinematic screen, not a complete inverse-dynamics or actuator-power feasibility result. The existing static geometry/gravity checks still run. Actual forces, measured tracking, motor utilization and body motion are verified by the experiment recording/audits.

Motor torque/speed caps, the explicit 0.25 rad/s fast-profile audit envelope, tracking tolerances, force bands, contact freshness and geometry thresholds remain unchanged for this candidate. There is no new torso or changed mass distribution.

GUI and CLI expose separate movement and weight-transfer timing. A matching offline preview must agree on count, distance and both effective timing scales before a GUI live run is allowed. Historical previews without a transfer field inherit their main movement timing.

## Run record

- `20261117-195443-e47b5f8d`: three-placement offline preflight passed, movement 0.25 / transfer 0.5.
- `20261117-195608-a9a91c94`: failed before walking because Gazebo could not load the installed ROS controller's diagnostic dependency. No gait failure or success can be inferred. See [dependency repair](ROS_DIAGNOSTIC_COMPATIBILITY.md).
- `20261117-200157-e96abbe2`: repeat with isolated diagnostic compatibility library; results pending.

The repeat `20261117-200157-e96abbe2` stopped during first-foot lift because sole-height tracking exceeded the retained 0.5 mm bound. The last recorded sample showed 0.481 mm reference/observed sole error, 14 ms geometry lag and approximately 0.00179 rad knee tracking error. This identifies tracking latency/error, rather than proving insufficient motor torque.

The optional `velocity_feedforward` setting adds `(physical viscous coefficient + scheduled joint derivative gain) × reference joint velocity` to the existing static support feedforward. Joint PID still subtracts `Kd × measured velocity`; the added term supplies desired-motion damping torque. Reference velocity is computed from the known smooth plan with a 1 ms finite difference, not from future measurements. The physical damping coefficients come from the same joint URDF used by the damping plugin. The existing command caps and reserve checks remain active, and the independent torque-composition audit recomputes the modified feedforward from the plan.

This is model-based damping compensation, not full floating-base inverse dynamics. It does not claim to include inertial/Coriolis torque or identify an accurate hardware torque-speed curve. Existing profiles default to feedforward disabled.

- `20261117-200750-88728437`: three-placement preflight passed with movement 0.25 / transfer 0.5 and velocity feedforward.
- `20261117-200914-733f1f51`: live candidate with velocity feedforward; results pending.

The first feedforward live attempt `20261117-200914-733f1f51` also stopped at the sole-height guard. Its recorded physical damping array was all zeros: the runtime-prepared URDF stores physical damping in the explicit Gazebo plugin and clears native joint damping, whereas the planning URDF stores it on joints. The initial implementation read only the native tags.

`model/physical_damping.py` now reads the explicit plugin when present, validates all ten names and rejects duplicated native/plugin damping; otherwise it reads geometry joint tags. An actual prepared-versus-geometry model check produced identical `[0.2, 2.5, 3.0, 2.5, 0.2]` coefficients per leg, with two additional regressions covering representation parity and duplication rejection. This fixes runtime feedforward and preserves offline/live motor-composition agreement.

- `20261117-201351-50e7eebf`: corrected physical damping lookup repeat; results pending.

The corrected-damping repeat `20261117-201351-50e7eebf` completed the first placement and tracked the second swing but stopped because touchdown was not yet confirmed before reload. The last geometry had the swing sole approximately 0.1 micrometre above the plane while its fresh contact message was empty. Tracking tolerance was not relaxed.

The feedforward variant now retains a four-second recontact hold and adds a 0.1 N touchdown force seed. Its joint torque increment is computed once per landing posture as the difference between two static reaction distributions: full weight on support versus 0.1 N on the landing foot and weight minus 0.1 N on support. This preserves the planned total force. The increment ramps in during the latter half of lowering, remains during recontact and fades out during the first second of reload. All actual contact-force and geometry guards remain active; the seed is below the existing 0.5 N unloaded-foot bound and does not substitute for measured contact.

The preload torque, velocity compensation and static feedforward are logged separately. Tests cover ramp/removal and their exact torque sum. Faster preflights also identify planner/model/URDF/config source content through a fingerprint; a changed feedforward planner requires a fresh preview.

- `20261117-202503-feb24944`: updated landing/preload offline preflight passed.
- `20261117-202634-b6504aa9`: updated live repeat; results pending.

The landing/preload repeat `20261117-202634-b6504aa9` stopped on a 1.386 N landing transient during final set-down. The existing blanket 0.5 N swing-foot rule was still being applied during actual touchdown, despite fresh ground contact, a loaded support foot, total force within the 5% weight band and swing share below the existing two-percentage-point bound.

The faster feedforward variant now has an explicit phase policy in `motion/touchdown.py`: the 0.5 N limit remains during unloading, lift, advance and all raised-foot motion; only `lower_foot`/`recontact` with planned sole clearance below 0.5 mm may use a landing limit of 2% body weight (4.371 N for this model). Existing total-force, load-share, support-margin, freshness, no-contact-while-raised and landing-before-reload rules still apply. The base programs retain their original policy by default. This is a deliberate phase-specific landing-policy change, not an assertion that every force threshold is unchanged.

The 0.1 N preload now ramps in over one second **after** nominal set-down and fades during reload, avoiding an extra downward seed during the airborne descent. Unit tests ensure touchdown permission cannot apply during unloading or flight, and that the preload does not persist into final hold.

- `20261117-203250-8a6c3d71`: phase-specific touchdown preflight; results pending.
