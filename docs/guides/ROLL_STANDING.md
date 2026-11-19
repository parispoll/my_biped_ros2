# First two-foot standing with ten joints

This milestone connects the separate roll-joint model to bounded effort control.
It establishes a double-support baseline before lateral weight shifts, foot
unloading or single-leg balance. No walking or active body-roll stabilization
is included in this experiment.

## The calculated stance

The new model has eleven links with total mass 14.42 kg. Roll angles start at
zero and both knees bend by 0.4 rad. For an upright pelvis and flat soles:

```text
hip_pitch + knee_pitch + ankle_pitch = 0
COM_x - foot_center_x = 0
```

`roll_stance.py` finds the hip angle numerically using the full URDF tree and
all link masses. It does not reuse the old seven-link mass approximation.
At the default knee bend, the planned per-leg joint targets are:

| Joint | Target (rad) | Motor command cap (N·m) |
| --- | --- | --- |
| Hip roll | 0 | ±6 |
| Hip pitch | -0.2237043353 | ±6 |
| Knee pitch | 0.4000000000 | ±10 |
| Ankle pitch | -0.1762956647 | ±6 |
| Ankle roll | 0 | ±6 |

The expected pelvis height is 0.5544914465 m. Under ideal equal static support,
each foot carries `14.42 × 9.81 / 2 = 70.7301 N` vertically. This is an equilibrium
assumption, not a measured foot force.

For each joint, the planner sums distal body weight moments and the assumed
ground-reaction moment about its actual joint axis. The motor cancels their sum:

```text
motor torque = -axis · [Σ(r_mass × weight) + r_sole × ground_force]
```

The resulting feedforward is approximately -2.563224 N·m at each knee and zero
at the other joints. Zero nominal roll torque follows the symmetric pose; it
does not mean roll motors will never need torque during disturbances or a shift.
The ground-reaction assumptions apply only to this static two-foot stance.

## Controller and physical damping

The separate ten-joint controller uses named ordering. For every joint:

```text
error = target - measured_position
requested = gravity_feedforward + Kp × error - Kd × measured_speed + integral
motor_command = clamp(requested, -motor_cap, +motor_cap)
```

The integral contribution is bounded and freezes increments that would drive
an already saturated request farther into saturation. Delayed but fresh feedback
freezes integration for that update; stale source timestamps fail the guard.
The default per-leg gains are explicit in `roll_posture.py`:

| Joint order | Kp | Kd | Ki | Integral cap (N·m) |
| --- | --- | --- | --- | --- |
| Hip roll | 40 | 0.10 | 20 | 0.5 |
| Hip pitch | 80 | 0.10 | 20 | 1.0 |
| Knee pitch | 100 | 0.15 | 25 | 1.0 |
| Ankle pitch | 60 | 0.05 | 15 | 1.0 |
| Ankle roll | 40 | 0.10 | 20 | 0.5 |

These are prototype gains and motor budgets, not manufacturer specifications.
IMU orientation is used for a pelvis-tilt shutdown check, not an additional
balance torque in this baseline. The roll joints hold neutral angles through
joint PID. A later body-roll feedback experiment must be measured separately.

Physical viscous damping remains separate from motor feedback:
`tau_damping = -b × joint_velocity`, applied at every 1 ms physics step.
Per-leg coefficients are `[0.2, 2.5, 3.0, 2.5, 0.2]` N·m·s/rad. Implicit backend
damping and joint friction are zeroed in this effort-test description, so damping
is not applied twice. The geometry-only model retains its original declared
parameters; the controlled description makes this conversion explicit.

The existing damping plugin previously required exactly six configured joints.
It now accepts an explicit `expected_joints=10`; omitting that parameter still
requires six. Existing six-joint configuration and coefficients are unchanged.

## Startup and measurement

The simulator first holds the configured initial positions while the effort
controller is inactive. After fresh clock/joint/IMU feedback and command-limit
enforcement are confirmed, a checked controller switch activates bounded effort
commands. Readiness requires at least 15 simulated seconds of effort control
and at least two seconds within 0.002 rad error and 0.002 rad/s speed.

After readiness, independent tools measure 30 simulated seconds of:

- Free robot pelvis pose, height and tilt.
- Fresh collision evidence for both feet touching the ground.
- Pelvis IMU orientation, angular speed and acceleration.
- All ten joint errors, speeds, motor terms and saturation.
- Whole-body COM, including the four added carrier masses.
- Explicit physical damping and sampled motor inputs at physics updates.

The COM probe already iterates over all inertial robot links. It did not need
a new eleven-link C++ implementation; the new run instead verifies its measured
mass against 14.42 kg and audits telemetry coverage. Collision contact evidence
is not a calibrated foot-force measurement, and support geometry is not ZMP.

The original 0.15 rad pelvis tilt shutdown and 0.2 rad / 3 rad/s posture guards
remain. Freshness failures stop the launch. Passing no-push standing does not
establish push recovery, single-foot balance or readiness to walk.

## Files and responsibilities

- `roll_geometry.py`: package-level FK and mass geometry, moved from the offline
  script so launch and controller can share it. `scripts/planning/roll_model_geometry.py`
  remains a compatibility import for the existing geometry tools.
- `roll_stance.py`: solves the centered crouch and estimates static loads.
- `roll_posture.py`: ten-joint PID math, integral bounds and motor clipping.
- `roll_description.py`: initial joint states, command-limit checks, temporary
  controller YAML and explicit damping configuration.
- `urdf/roll_effort.urdf.xacro`: separate actuated model with IMU/contact sensors.
- `config/roll_standing.yaml`: ten named effort interfaces and broadcaster.
- `scripts/hold_roll_stance.py` inside the package: ROS subscriptions, activation,
  timing guards, readiness and recording. It calls the pure control math.
- `launch/roll_standing.launch.py`: startup sequence, bridges, COM and cleanup.
- `roll_audit.py`: independent checks on saved motor and damping evidence.
- Workspace `scripts/experiments/run_roll_standing.py`: isolated experiment, independent
  measurement processes and summary. Existing six-joint runners remain separate.
- `CMakeLists.txt`: installs the new ROS executable; the existing plugin build
  picks up its explicit ten-joint count support.
- `tests/test_roll_standing.py`: static geometry, controlled initial states,
  motor limits, damping configuration, antiwindup and evidence rejection tests.

## Reproduce

From the workspace root, use a new output folder for each run:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
python3 scripts/experiments/run_roll_standing.py --gui \
  --output evidence/my-ten-joint-standing-01
```

The runner isolates ROS domain 76 and a unique Gazebo partition. It runs at
0.25× pacing, collects 30 simulated seconds after readiness and closes its owned
simulation when finished. Allow a few minutes of wall time. Omit `--gui` for a
headless measurement. This experiment is not yet an option on the push website.

For an interactive simulation that stays open, source the workspace, choose
an unused ROS domain and Gazebo partition, and run:

```bash
export ROS_DOMAIN_ID=76
export GZ_PARTITION=roll_standing_manual
ros2 launch my_robot_description roll_standing.launch.py gui:=true
```

Ctrl+C closes it. That manual launch alone is not an independently verified
measurement; use the runner to generate the full evidence summary.

## Why the first controller transfer was changed

The first launch attempt (`roll-standing-01`) stopped because the new executable
was missing executable permission. That packaging issue was fixed; the failed
attempt remains saved and is not counted as a standing trial.

The next run (`roll-standing-02`) activated successfully but did not settle before
the startup deadline. Its pitch oscillations grew, while roll remained near
neutral. The transferred pitch integral gains were `[80, 100, 60]`, the same
as the six-joint profile. Copying gains is a starting experiment, not proof they
will work after changing the articulated model and mass distribution.

For the next candidate, only pitch integral gains were reduced by a factor of
four: `[20, 25, 15]`. For example:

```python
# Transferred profile, per leg: hip roll, hip pitch, knee pitch, ankle pitch, ankle roll
KI = [20., 80., 100., 60., 20.]
# Candidate: roll gains unchanged, pitch integral gains reduced
KI = [20., 20., 25., 15., 20.]
```

At an ankle error of 0.01 rad, the previous integral contribution changes at
`60 × 0.01 = 0.6 N·m/s`; the candidate changes at `15 × 0.01 = 0.15 N·m/s`.
That slows the accumulation of corrective torque and reduces the tendency to
keep pushing after the error changes sign. This is a measured tuning candidate,
not a universal rule that lower integral gain is always better.

No motor cap, geometry, physical damping coefficient, proportional gain,
velocity feedback gain, roll integral gain, readiness threshold or tilt guard
was increased to make the candidate pass. The failed startup trace is retained
for comparison. `scripts/plots/plot_roll_standing.py` plots the two attempts.

## Sensor-time correction found in the GUI repeat

The first GUI-enabled repeat (`roll-standing-04`) reached joint readiness, then
stopped on the new IMU freshness check. Its independent pose checker subsequently
observed a height change after the posture controller exited. This is retained
as a timing failure, not counted as a successful standing repeat or an
independent physical recovery failure.

The initial check subtracted the newest IMU stamp from the current joint
message stamp. These are separate asynchronous streams. For example:

```text
Joint message stamp: 10.01 s
Latest IMU stamp:    10.04 s
Simulation clock:   10.05 s

IMU relative to joint: -0.03 s  (different measurement instants)
IMU age from clock:     0.01 s  (fresh)
```

`roll_timing.py` now checks latest IMU age against the shared simulation clock.
The bounds remain -0.02 to +0.05 s; genuinely stale or excessively future IMU
stamps still stop the run. The old recording does not include the rejected
sample's timestamps, so it cannot prove which timestamp direction triggered
that failure. The correction addresses the reference-time defect; the fresh
GUI-enabled rerun is the verification of the corrected path.

A separate recorded `imu_minus_joint_stamp_s` exposes the cross-stream offset
without treating it as source freshness. Joint feedback keeps its independent
clock-age and integration-gap checks. Unit tests cover asynchronous stamps,
genuine staleness and future/invalid timestamps.

The next two startup attempts are also retained: `roll-standing-05` found a
missing installed timing module and required a package rebuild;
`roll-standing-06` caught a genuine 0.051 s IMU age during initial startup.
The startup policy now skips an unsynchronized IMU update before readiness,
without integrating or publishing a new command from that update, and restarts
the full settling interval. The one-wall-second sensor-loss watchdog still
applies. After readiness, the same 0.051 s age remains a failure. The independent
verifier processes also start 0.5 wall second apart to avoid one import/discovery
burst. Their individual measurement durations and thresholds are unchanged.

## Closing writers before auditing their files

The corrected timing path in GUI run `roll-standing-07` completed all independent
measurement windows. Its final file audit failed because the runner read the
C++ damping CSV while that writer was still appending a row. Rechecking the
closed file passed every evidence audit; this is saved separately as
`postrun-verification.json`. The original error summary is preserved.

The runner now closes its owned simulation after all live measurement tools
finish, then reads the closed controller/COM/damping files. This separates
acquisition from analysis and avoids interpreting a half-written final row as
a physical model failure. The change does not discard malformed interior rows,
relax damping checks or change any motor control parameter.

## Verified results

Two fresh 30-simulated-second standing runs passed: `roll-standing-03`
(headless) and `roll-standing-08` (GUI enabled, corrected acquisition/audit
pipeline). The additional GUI recording `roll-standing-07` also passed all
closed-file evidence checks, but retains its original runner error and separate
reassessment. It is not presented as an error-free runner completion.

| Measurement window | Headless 03 | GUI 08 |
| --- | --- | --- |
| Pose/contact/IMU checks | Pass | Pass |
| Maximum joint error | 5.34e-7 rad | 5.04e-7 rad |
| Maximum joint speed | 1.60e-6 rad/s | 1.61e-6 rad/s |
| Maximum measured pelvis tilt | 2.84e-7 rad | 2.87e-7 rad |
| Pelvis height | 0.554491368 m | 0.554491365 m |
| Observed COM mass | 14.42 kg | 14.42 kg |
| Saturated motor samples | 0 | 0 |
| COM coverage/motor composition/passive damping | Pass | Pass |

These near-zero values describe an ideal, symmetric, stationary simulation.
They do not establish comparable accuracy on hardware, response to disturbances,
or single-leg stability. The new model has no planar guide, static pelvis or
external support fixture; both feet provide support on the ground. Contact
checks remain collision evidence, not force validation.

The key controller change was pitch integral gain reduction. Model geometry,
motor caps and passive damping stayed fixed between the unsuccessful controller
transfer and successful candidate. Timing/packaging/file-lifecycle corrections
were needed to make the experiment tool run reliably and are documented above.
All failed attempts remain saved; nothing was silently retried into the same
output directory or removed from the evidence history.

The next milestone is a small, measured **sideways weight shift with both feet
remaining on the ground**. We must derive compatible hip/ankle roll targets and
measure body/COM response before unloading a foot. Standing success alone does
not justify jumping directly to a leg lift.

The final suite passes **231 tests**. A separate 5-simulated-second original
six-joint PD/explicit-damping regression also passed pose, contact and IMU
checks. Its closed damping trace passed the original six-joint damping audit,
confirming default six-joint behavior remains supported. This short regression
is not a rerun of the earlier force-recovery studies.

`evidence/roll-standing-08/verification.json` records the source/binary checks,
comparison runs and compatibility test. The converted controlled model has
ten revolute joints, eleven links and `static=false`, with no world-support
joint. The saved controlled URDF is an inspection snapshot containing temporary
configuration paths from the run; use the launch command to regenerate a
runnable description rather than directly spawning that snapshot.

The small bilateral lateral COM-shift milestone is now implemented separately;
see [LATERAL_COM_SHIFT.md](../history/LATERAL_COM_SHIFT.md). It uses an optional lower roll
integral gain while the ordinary-standing defaults documented here remain
unchanged. Foot-load validation still precedes any unsupported foot lift.
