# First measured lowering experiment

Update: the GUI now offers 10 mm and 20 mm lowering milestones. See
[GUI_ROADMAP.md](GUI_ROADMAP.md) for current controls, timing and development
stages. The 10 mm measurements below remain the original baseline.

This is the first implementation step after the geometry calculation PDF. It
lowers the pelvis by **10 mm**, holds, and returns, with both feet planted.
It does not implement the PDF's 60 mm lowered single-support candidate yet.

## Why we separate this motion

The existing lateral experiment assumes almost constant body height. A deeper
posture needs a distinct planned height reference and measured height audit.
Starting with a small centered lowering lets us isolate the kinematics and
feedforward before combining lowering with lateral load transfer.

We retain the P54 masses, geometry, torque caps and PID gains. The 25% minimum
measured load share, 5% total-weight force band, contact freshness, motor
headroom, IMU limits, planted-foot checks and 0.05 rad/s slow-motion audit
remain in effect. Existing experiments keep their constant-height audit.

## Geometry and forces

At each table point the planner solves two equations for hip and knee pitch:

```
COM_x(q) - ankle_x(q) = 0
-ankle_z(q) + foot_thickness = requested_pelvis_height
```

Ankle pitch is `-hip_pitch-knee_pitch`; roll targets remain zero. The planner
then computes the small fore/aft pelvis translation needed to keep the ankle
world positions fixed:

```
pelvis_x = initial_ankle_x - current_ankle_x
```

Simply increasing both knee angles without re-centering COM would give an
incorrect fore/aft static moment. For this symmetric centered family the
predicted vertical reactions remain `M*g/2` per foot. Those are feedforward
assumptions: actual forces are independently decoded from contact wrenches.

The 41 cached poses are interpolated with a quintic time phase. Schedule:
3 s baseline, 10 s lowering, 6 s hold, 10 s rising, 8 s recovery. All are
simulated seconds, after full standing readiness. The simulator runs with a
0.25 real-time-factor target; allow about four wall minutes including warm-up.

## Separate responsibilities

| File | Responsibility |
|---|---|
| `src/my_robot_description/my_robot_description/motion/bilateral_lowering.py` | Offline centered-COM IK, feedforward table and timed references |
| `src/my_robot_description/nodes/control/hold_roll_stance.py` | Explicit opt-in program selection; existing effort controller and live bilateral force guards |
| `src/my_robot_description/launch/roll_standing.launch.py` | Pass the two new lowering parameters and enable contact sampling |
| `src/my_robot_description/my_robot_description/validation/lateral_audit.py` | Replay existing motor/contact/foot checks; accept a planned height only when explicitly supplied |
| `src/my_robot_description/my_robot_description/validation/lowering_audit.py` | Independently measure lowering/return, fore/aft motion, force symmetry and CoP/COM agreement |
| `scripts/experiments/run_lateral_shift.py` | New mutually exclusive `--lowering` mode; start recorders before warm-up, close writers before audits, save source hashes |
| `scripts/plots/plot_lowering.py` | Compare planned/measured height and plot measured loads and joint speeds |
| `tests/test_bilateral_lowering.py` | Check planted feet, centered COM, limits, return, and rejection of missing motion or wrong force split |

## Acceptance criteria

Height follows the planned trajectory within the existing 5 mm absolute pose
envelope; the new lowering audit additionally requires at most **2 mm**
baseline-relative height/fore-aft tracking error. The measured hold drop must
match the requested drop within 2 mm. Final height and fore/aft displacement
must return within **1 mm** of baseline.

Measured left load share must stay within 2 percentage points of 50%, total
vertical reaction within 5% of weight, and force-weighted CoP within 3 mm of
the measured COM projection. Force samples require complete source-timestamp
coverage with no gap greater than 0.05 simulated seconds. These are additional
checks, not replacement estimates for missing contact evidence.

## Reproduce

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash

python3 scripts/experiments/run_lateral_shift.py --gui \
  --actuator-profile robotis_p54 --lowering 0.01 \
  --output evidence/my-bilateral-lowering-01

python3 scripts/plots/plot_lowering.py evidence/my-bilateral-lowering-01
```

Use a new output folder each time. `--gui` opens Gazebo with the front camera;
omit it for a headless measured run. There is no push in this experiment.
The web panel continues to offer its existing P54 lateral-transfer/pulse mode;
this new lowering experiment is currently run through the command above.

Do not increase the 10 mm limit to jump directly to single support. A passing
run validates this small bilateral lowering only. Later steps should check
larger lowering, combine it with a measured load transfer, and implement
explicit contact-transition states and pressure/balance control before lift.

## Verified result - 1 November 2026

`evidence/p54-bilateral-lowering-01` passed all motion, independent sensor,
physical damping, measured lowering/pressure and live force-guard audits.
The package source hashes were unchanged throughout measurement. This was a
headless run; `--gui` is available for reproduction but was not newly tested
as a native visual observation in this milestone.

| Measurement | Result |
|---|---:|
| Requested / measured lowering | 10.000 / 10.008 mm |
| Planned / measured fore-aft pelvis motion | -5.745 / -5.747 mm |
| Maximum baseline-relative height tracking error | 0.0364 mm |
| Maximum final height offset | 0.0059 mm |
| Maximum final fore-aft offset | 0.0070 mm |
| Maximum load-share deviation from 50% | 0.0094 percentage points |
| Maximum vertical-force error from weight | 0.0404 N |
| Maximum CoP/COM horizontal separation | 0.0214 mm |
| Maximum force-sample gap | 0.020 s |

The build and 284 unit tests passed, including geometry and audit-rejection
checks. The first full-suite invocation omitted the workspace environment
and failed package discovery; the corrected invocation sourced both ROS and
`install/setup.bash` and passed. This was a test-environment issue, not a
change to acceptance criteria. Logs are retained with the experiment.

`response.png` shows planned/measured height, actual foot shares and joint
speed. The robot's URDF, physical damping, actuator limits and PID gains were
not changed. The next milestone should extend lowering gradually toward the
calculated posture while retaining bilateral support, before combining it
with a lateral transfer.
