# Zero-load transition: sensing before lifting

This stage adds a separate contact-state source and zero-load trajectory.
It commands a fixed flat-sole reference, not a raised foot or a walking step.
Live validation status and job IDs are recorded below when trials complete.

## Why the first sensor bench failed

The original Gazebo Contact publisher emits a message only when its contact
list is nonempty. The source code confirms the behavior:
https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/contact/Contact.cc

In GUI job `20261105-160828-c4cd7fc6`, moving a 1 kg fixture above the floor
produced no fresh empty contact packets. The existing sampler retained the
original timestamp, so its repeated packet correctly became stale. The
failed diagnostic is preserved; silence is not reclassified as zero load.

The physics backend independently clears each collision's ContactSensorData
when that collision has no contact in the current physics step:
https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/physics/Physics.cc

A new read-only PostUpdate plugin reads that component, including explicit
empty lists, and publishes at 100 simulated Hz on separate `/biped/state/`
topics. It sends no motor commands and does not replace the original topics.
It waits for initial contact evidence and stops emitting when paused or when
the component is absent. A clock reset requires contact evidence again.

GUI job `20261105-161143-a18d60f9` passed three contact-air-contact cycles
through the new source, ROS bridge and existing sampler: 1,329 sampled
fresh no-contact observations, while the original raw topic had zero.
Counts include repeated snapshots of source-stamped packets, not 1,329
independent sensor messages. This fixture result validates sensor semantics,
not humanoid balance.

## Explicit contact states

| State | Meaning | A reliable zero force? |
|---|---|---|
| loaded_contact | Valid named foot/ground wrench above 1 N | No |
| touching_unloaded | Valid ground manifold at or below 1 N | Use its measured force; it can be small and positive |
| no_contact | Fresh source-stamped, explicitly empty physics contact list | Yes; pressure point undefined |
| unknown | No message has arrived | No |
| stale | Original source stamp is too old | No |
| invalid | Bad frame, stamp, collision ordering or wrench | No |

At exactly zero force, center of pressure is `None`, never an invented sole
center. Earlier bilateral decoders still reject empty and zero-total contact
messages by default.

## New trajectory and guards

Solve the lowered fixed-foot IK with COM at the supporting foot center:
`COM_y = ±0.085 m`. Static sole-center reactions become `(mg, 0)` or `(0, mg)`.
The current P54 model predicts about **±132.022 mm pelvis travel**, and
**53.058% peak static motor utilization**. All ten joint limits are checked.
Only floating-point roundoff smaller than 1e-8 N at the zero endpoint may be
clipped and rebalanced; a genuinely negative reaction is rejected.

The timing remains the verified 233 simulated-second program, with 64 s for
each lateral movement. Offline checks validate fixed soles, COM, joint speed,
torque bounds and exact return. Geometry and motor caps are retained; zero-mode gain schedules are described below.

The new mode requires passing same-side 5% unloading and a passing physics
contact-state diagnostic. During execution:

- Supporting-foot contact must remain loaded and fresh at all times.
- Other-foot absence is permitted only below 1% planned remaining share.
- Both source ages must stay within 50 ms, with the existing small future-stamp allowance.
- Total measured vertical support must stay within 5% of weight.
- Measured remaining share must follow the planned share within two percentage points.
- Existing tilt, tracking, motor, foot-geometry and sensor checks remain active.

Independent held-window acceptance requires **mean unloaded force ≤0.5 N**,
**COM projection at least 10 mm inside the supporting sole**, contact-state
coverage and matching live guards. This is an operational zero-load tolerance,
not a claim of mathematically exact zero or robust single-support balance.

## File map

- `src/my_robot_description/plugins/contact_state_probe.cpp`: physics-side contact heartbeat.
- `CMakeLists.txt`: builds/installs its shared library.
- `my_robot_description/contact_state.py`: explicit source/freshness/force classifier.
- `my_robot_description/foot_loads.py`: opt-in exact-zero manifold decoding; earlier defaults retained.
- `my_robot_description/zero_load.py`: separate zero-target planner, live guard, paired state decoder and independent goal audit.
- `lowered_unload.py`, `lowered_shift.py`: narrow reuse hooks and zero-endpoint roundoff handling.
- `hold_roll_stance.py`, `roll_standing.launch.py`: new-mode selection, new stream/bridge/sampler remapping and live guard.
- `lateral_audit.py`, `lowering_audit.py`: explicit optional contact-policy/pairing hooks; existing defaults remain strict.
- `scripts/experiments/run_contact_transition_bench.py`: isolated contact/air/contact fixture, evidence and plot.
- `scripts/experiments/run_lateral_shift.py`, `record_lateral_sensors.py`, `plot_lowering.py`: zero-load recording, audits and plot.
- `tools/force_panel/`: diagnostic/zero-load modes, prerequisites, results and roadmap.
- `tests/test_contact_state.py`, `tests/test_zero_load.py`, `tests/test_force_panel.py`: classification, guarded absence, independent rejection, mirrored geometry and GUI gates.

The panel also now loads all saved job metadata rather than only the latest
20 records, so new experiments do not erase older verified roadmap stages.

## Reproduce from the GUI

1. Refresh http://127.0.0.1:8765/.
2. The contact sensor diagnostic should show verified.
3. Select **Zero-load transition: left support** or **right support**.
4. Confirm supporting side. Click Run lowered transfer.
5. Allow about 18 wall minutes. Inspect the unloaded-force result and saved plot.

No foot-lift command is part of this stage. Contact absence can occur at a
fixed-sole reference with negligible load; subsequent lifting needs separate
clearance, touchdown and single-support control validation.


## First humanoid trial and unloaded-ankle correction

GUI job `20261105-163140-e3a5716c` is preserved as a failed trial. It
reached about 0.071 N right-foot force, then observed fresh no-contact
evidence. The right ankle-roll velocity began alternating and growing;
the existing 2.4 rad/s operating guard stopped the simulation before the
hold or return. This is not counted as a successful zero-load experiment.

The right-foot URDF inertia about its ankle-roll axis is approximately
0.0025866 kg m², including the inertial-origin parallel-axis term. A
parent-fixed, gravity-compensated linear approximation uses 4 ms motor
updates, four 1 ms physics steps per motor update, KP=40, KI=5 and physical
viscous coefficient 0.2. Its sampled transition matrix has spectral radius
2.09964 at motor KD=2 (unstable) and 0.999499 at KD=0.3 (locally stable).
This approximation diagnoses the observed alternating response; it is not
a full-body or impact stability proof.

In the first correction, only zero-load mode scheduled the unloading ankle-roll derivative gain
from 2 to 0.3 as planned remaining load decreases from 5% to 1%, using a
quintic blend. It restores the gain during return. All other joint gains,
integral gains, physical damping, motor caps and speed guards remain as
before. The command audit independently reconstructs and checks the exact
recorded derivative schedule. The previous bilateral modes retain their
original derivative gains.

Added `scripts/analysis/analyze_unloaded_ankle.py` and
`tests/test_unloaded_ankle.py`; expanded zero-load planning tests to check
the scheduled joint, range and restoration. Numerical output is saved in
`evidence/preparation/zero-load-transition/unloaded-ankle-analysis.json`.


## Second trial and full supporting-posture stiffness

GUI job `20261105-165132-1c0bd7a2` reached the hold after the free-ankle
correction, but was stopped by the measured-load tracking guard. The other
foot reloaded above two percentage points as support-leg motion developed.
This trial is retained and is not a successful zero-load validation.

The next diagnostic fixes the supporting sole in world coordinates and
expresses all ten joint angles relative to it. For each perturbed pose it
computes the world COM and gravitational potential `U(q) = m g z_COM(q)`.
The finite-difference gradient agrees with the nominal feedforward torques.
The Hessian `H = d²U/dq²` is the local gravitational stiffness. At equilibrium,
posture PID adds `diag(KP)`, so a negative eigenvalue of `H + diag(KP)` reveals
an unstable static mode. With the original gains the minimum is about
**−33.29 N·m/rad**. Increasing only ankle roll leaves a separate ankle-pitch
mode unstable, so both supporting-ankle axes must be considered.

Finite-difference body linear/angular Jacobians and URDF inertias also give
`M = Σ(m Jvᵀ Jv + Jwᵀ I_world Jw)`. A local sampled model includes this mass
matrix, gravity Hessian, existing physical damping, four 1 ms physics steps,
4 ms motor updates and the existing integral gains. It assumes a fixed
support sole and ignores impacts, delays and saturation. It is diagnostic,
not a nonlinear stability guarantee.

At the candidate supporting-ankle KP=120 and KD=8 (pitch and roll), the
minimum stiffness eigenvalue becomes **+30.58 N·m/rad** and the sampled
spectral radius is about **0.999783**, below one. The unloaded ankle-roll
KD remains 0.3. All these zero-mode gains use the same smooth 5%-to-1%
planned-load blend and restore their original values on return. Supporting
hip and all other gains, motor caps, speed limits and physical damping are
unchanged. Proportional and derivative gains are recorded each sample and
independently checked by the command reconstruction audit.

Reproduce the read-only diagnostic from the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/analysis/analyze_zero_support.py \
  --trial evidence/gui/20261105-165132-1c0bd7a2 \
  --output evidence/my-zero-support-analysis.json
```

Numerical matrices and candidate comparisons are saved in
`evidence/preparation/zero-load-transition/full-posture-linearization.json`.


## Verified left-support result — 5 November 2026

GUI job `20261105-171009-b3cfd7e7` completed the full 233 simulated seconds:
lower 40 mm, shift the pelvis 132.022 mm left, hold, return and rise.
All posture, independent sensor, force, contact-policy, motor, damping and
return audits passed. Package sources stayed unchanged during recording.

| Measurement | Result |
|---|---:|
| Right-foot mean vertical force in audited hold | 0.003478 N (limit 0.5 N) |
| Minimum held COM projection margin inside left sole | 59.981 mm |
| Measured lateral COM shift | 85.017 mm |
| Peak requested motor utilization | 53.057% (guard 80%) |
| Peak joint speed | 0.022327 rad/s |
| Saturated controller samples | 0 |
| Final maximum pelvis lateral offset | 0.000093 mm |
| Final maximum height offset | 0.002381 mm |

All 100 audited held force samples classified the right foot as
`touching_unloaded`; none were `no_contact`. This is operational zero load
with a fixed-sole reference, not a commanded foot lift or demonstrated
lifted-foot balance. The controller restored its original gains on return.
Controller, COM recorder and Gazebo exited cleanly.

The original raw contact sensor failed the separate absence diagnostic;
the new physics-state stream passed three contact/loss/restoration cycles.
Two failed robot trials remain preserved and explained above. Offline
validation included 316 full-suite tests before the final supporting-ankle
schedule, then 65 focused tests after that schedule.

Full measured evidence is in
`evidence/gui/20261105-171009-b3cfd7e7/summary.json`; an independent motor
utilization and shutdown review is in
`evidence/preparation/zero-load-transition/measured-review.json`.
The GUI marks only left-support zero load verified. Next, mirror this test
with right support before introducing a small deliberate foot lift.


## Verified mirrored right-support result — 5 November 2026

User-run GUI job `20261105-194755-063f4cdd` passed the complete 233-second
zero-load transition and return. The supporting foot was right; the left
foot was unloaded. Its audited mean vertical force was **0.003480 N**,
with a minimum right-sole COM margin of **59.981 mm**. All 100 held samples
were touching_unloaded, with no commanded lift and no held no_contact samples.
Measured COM travel was −85.017 mm and pelvis travel −132.048 mm.

Peak requested motor utilization was **53.057%**, with zero saturated
samples and a peak joint speed of 0.022347 rad/s. Final maximum height offset
was 0.002457 mm. All sensor, contact-policy, force, posture, motor, damping
and return audits passed; the original gains were restored and owned
processes exited cleanly. Source hashes match the verified left-support
trial: **True**. No source changed during this recording.

Both supporting sides now have measured zero-load validation. Next is a
small intentional foot lift, with separate clearance, contact-loss and
support-balance checks. Neither present trial demonstrates that lift.
Independent review: `evidence/preparation/zero-load-transition/mirrored-right-review.json`.
