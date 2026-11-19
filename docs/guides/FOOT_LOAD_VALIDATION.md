# Measuring foot loads before trying to unload a foot

The previous shift experiment measured COM movement and collision evidence, but
its foot-force curves were static estimates. This step reads the physics
engine's actual contact-wrench messages, validates their interpretation, and
compares the measured loads at center, during the held shift and after return.
The controller, motion, joint limits, motor caps, masses and geometry are unchanged.

## No extra joint, sensor mass or controller was needed

Each existing foot contact sensor already publishes contact points, normals,
penetration depths and a wrench for each point. Those fields were present in
`ros_gz_interfaces/Contacts` but our earlier recorder kept only a ground-contact
boolean. The new recorder optionally saves their complete payload.

This is **simulated ground-reaction-force feedback**, not a hardware load cell
or a calibrated real-world sensor. It is read-only and is not used by the motor
controller in this step. The existing joint PID and static feedforward still
produce the motion.

## What a foot load is

A flat foot has four ground contact points in these recordings. One point's
force is not the whole foot's force. We sum every point exactly once:

```text
F_foot_world = Σ F_contact_point_world
Fz_foot = sum of the upward Z components
Fz_total = Fz_left + Fz_right
```

The current adapter accepts the recorded ordering **foot collision first,
ground collision second**. It uses the first body's force, checks the second
is equal and opposite, and rejects negative upward support. It never uses
absolute values to conceal a sign mistake, averages the two feet into a single
force, or substitutes `mass × gravity` for an unavailable reading.

The messages' frame IDs and individual wrench timestamps are empty/zero in
this backend. Source time comes from the enclosing Contacts header. World-frame
force interpretation is established from Gazebo Physics's contact API and the
Harmonic 8.10 implementation, then checked against weight and moment balance.
A different collision ordering, non-world frame, missing wrench data, duplicate
manifold or incomplete point arrays is rejected. Changing backend or contact
schema requires revalidation, not guessing a rotation or force sign.

Primary references:

- [Gazebo Physics 7 contact API](https://gazebosim.org/api/physics/7/GetContacts_8hh_source.html)
- [Gazebo Sim 8.10 contact-message construction](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.10.0/src/systems/physics/Physics.cc)

The action/reaction check verifies the message interpretation and transport;
the producer constructs opposite vectors, so that check alone is not independent
proof of force accuracy. Weight balance and pressure/COM consistency add the
physical checks below.

## Validation at rest

The model mass is 14.42 kg and world gravity is 9.81 m/s²:

```text
W = m × g = 14.42 × 9.81 = 141.4602 N
Expected symmetric support = W / 2 = 70.7301 N per foot
```

These expected values are comparisons only. They are not inputs to the force
summation. A synthetic test doubles all recorded forces while preserving
opposite point-force pairs: it must fail the weight check.

Mean total force must be within 1% of weight in the late baseline, hold and
recovery phases. Peak total deviation during the slow motion must be within
5%. Baseline left/right difference must be within 1% of weight. The instantaneous
total need not equal weight exactly while accelerating: vertical acceleration
and asynchronous sampling can change it.

## Pressure center provides a separate moment check

For contacts on a horizontal plane, the force-weighted pressure center is:

```text
CoP_x = Σ(point_x × point_Fz) / Σ point_Fz
CoP_y = Σ(point_y × point_Fz) / Σ point_Fz
```

This is computed from measured point forces and positions. We do not read the
zero-filled wrench torque fields as a measured net foot moment. The adapter
can also derive contact moments using `Σ(position × force)`.

During quiet support, the global CoP should be near the COM's ground projection.
During motion they can differ because of acceleration/angular momentum. For
this slow small test, their horizontal separation must remain below 3 mm.
The COM comes from the separate eleven-link Gazebo COM probe, not the force
sum. This is a consistency check, not a general dynamic-stability criterion.

## Source time and pairing

The optional raw capture records each foot at approximately 50 simulated Hz,
preserving source stamps. Left-foot frames are paired only with the latest
right-foot frame at or before the left stamp, no older than 0.03 s. Future
samples are never used. COM matching also uses only a prior sample, at most
0.1 s old. Force coverage must span the full 25-second program with gaps no
larger than 0.05 s. Missing or stale data fails rather than becoming zero load.

This first validator requires positive, bilateral ground contact. It does not
yet interpret a newly empty contact message as a validated zero-load event for
a lifted foot. That needs a separate sensor-health/unloading state transition.

## What is checked during the shift

The motion is the same ±10 mm pelvis request, producing approximately ±7.12 mm
COM movement. The baseline, late hold and recovery windows are unchanged:
1–2.5 s, 11–13 s and 22–25 s after readiness.

We separately report:

- **Measurement validated:** timestamps, force schema, signs, weight balance,
  baseline symmetry and pressure/COM consistency passed.
- **Load transfer observed:** the expected foot gains at least 1 N while the
  other loses at least 1 N relative to baseline.
- **Returned near baseline:** each final mean foot load is within 1 N of its
  baseline value.

A valid force sensor need not show the load redistribution predicted by a
particular motion. Finite-area feet can redistribute pressure as well as total
load. The three flags are distinct so a failed motion objective cannot be hidden
by relabeling a measurement as invalid or replacing it with a static estimate.

## Files changed

- `my_robot_description/foot_loads.py`: pure decoding, point summation, source-time
  pairing, phase statistics, and physical consistency checks. No ROS or commands.
- `scripts/experiments/record_lateral_sensors.py`: optional `--capture-contacts`; complete
  raw contact payloads go to `contact-wrenches.jsonl`. Existing recording defaults
  remain unchanged. Capture starts before warm-up with the other measurements.
- `scripts/experiments/run_lateral_shift.py`: `--foot-loads` enables capture and closed-file
  force validation. `foot_forces_measured/validated` becomes true only when the
  force measurement audit passes. The existing motion/sensor/damping checks
  must still pass independently.
- `scripts/analysis/analyze_foot_loads.py`: repeatable post-run audit and plots from raw
  measurements; refuses to synthesize missing force data.
- `tests/test_foot_loads.py`: point summation, frames/order, missing data,
  sign/action-reaction, doubled-force rejection and stale pairing.

The earlier nested `predicted_foot_forces_n` stays explicitly a model estimate.
Measured phase means are under `foot_loads.phase_means`. No force feedback has
been added to the controller, and no mass, geometry, control gain, motor cap,
world physics setting or Gazebo plugin has changed for this measurement step.

## Reproduce

Use a fresh output directory:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
python3 scripts/experiments/run_lateral_shift.py --gui --foot-loads --displacement 0.01 \
  --output evidence/my-measured-left-loads-01
python3 scripts/analysis/analyze_foot_loads.py evidence/my-measured-left-loads-01
```

Use `--displacement -0.01` with another output folder for the other side.
The first raw probe is `evidence/foot-load-contact-probe-01`; its original
motion summary remains unchanged and the separate `foot-load-audit.json`
contains the post-run force validation. Older runs without raw contact-wrench
capture cannot be turned into measured-force recordings from their COM estimates.

`foot-loads.png` plots measured foot forces, total support against weight,
measured load shares and pressure center against independent COM. These replace
estimates only in the new plot; earlier estimate plots keep their original labels.

## Verified results

Both directions passed force, motion, sensor and physical-damping validation.
The +Y headless probe was validated after recording; its original summary stays
unchanged. The fresh -Y GUI run used `--foot-loads` and passed the integrated
runner with `foot_forces_measured: true` and `foot_forces_validated: true`.

| State | Measured left Fz | Measured right Fz | Measured total |
| --- | --- | --- | --- |
| Center baseline | 70.7301 N | 70.7301 N | 141.4602 N |
| Held +Y shift | 76.6549 N | 64.8053 N | 141.4602 N |
| Held -Y shift | 64.8053 N | 76.6549 N | 141.4602 N |
| Returned center | approximately 70.7301 N | approximately 70.7301 N | 141.4602 N |

Each shift transferred approximately **5.925 N** from one foot to the other.
The supported-side share rose from 50% to about **54.19%**; the other share
fell to about 45.81%. The held total remained close to the independent 141.4602 N
weight prediction. Maximum instantaneous total error was 0.0633 N for +Y and
0.00380 N for -Y. Unequal source timestamps can contribute to instantaneous
sum differences; samples are not claimed to be simultaneous.

Maximum horizontal CoP/COM separation was 0.302 mm and 0.325 mm respectively,
below the 3 mm small-motion check. Final mean foot loads returned within
0.001 N of their baseline values. The lowest foot load was still approximately
64.78 N: neither foot was close to unloading in these experiments.

The final suite passes **249 tests**, including a check that freezes the force
values at their center readings while leaving fresh timestamps: it fails the
independent pressure/COM consistency check when COM moves. Fresh packet stamps
alone therefore do not validate an unchanged force signal.

Artifacts:

- `evidence/foot-load-contact-probe-01/`: +Y motion, raw contact payloads,
  separate `foot-load-audit.json` and `foot-loads.png`.
- `evidence/foot-load-right-01/`: -Y GUI run, integrated `summary.json`, raw
  payloads, repeated offline audit and plots.
- The combined `verification.json` in the latter folder records source hashes,
  unchanged control/geometry settings, software versions and both outcomes.

There is one measured run per direction in this stage, not a repeatability
or force-envelope study. These are ideal simulation forces, not hardware
calibration. The next step is to plan gradual unloading against measured foot
loads, kinematic reach and motor-torque limits before attempting a lift.
