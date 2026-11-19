# First lateral COM shift with both feet grounded

The aim is a small, slow sideways center-of-mass shift and return. Both feet
must remain in ground contact. This is preparation for unloading a foot; it is
not single-leg balance, walking or a foot-force measurement.

## What is commanded

Each fresh run starts from verified ten-joint standing. The program requests
10 mm of upright pelvis translation in world Y, holds, and returns. +Y is the
robot's left; -Y is its right. It uses the existing effort caps, pitch gains,
physical damping, 1 ms physics step and 0.25× pacing, with a separately selected
roll integral gain for the shift experiment (explained below). No support guide, fixed
pelvis, force pulse or additional actuator is introduced.

| Phase after readiness | Simulated time | Purpose |
| --- | --- | --- |
| Center baseline | 0–3 s | Measure the initial COM and feet |
| Move outward | 3–7 s | Smoothly change the joint targets |
| Shifted hold | 7–13 s | Measure the settled lateral response |
| Return | 13–17 s | Smoothly return to the original pose |
| Center recovery | 17–25 s | Measure residual displacement and settling |

The hold's final two seconds are used for the mean shifted COM, and recovery's
final three seconds for the return check. Each direction uses a fresh world.
The first headless run tests +Y; the opposite direction is a separate run.

## Why hip roll alone is insufficient

Let `d` be the desired pelvis Y translation, and `z0 < 0` the original ankle
height relative to its hip. Each planted ankle must have relative lateral
coordinate `-d` while its world position stays unchanged. For an upright pelvis:

```text
hip_roll = atan2(-d, -z0)
local_sagittal_z = -sqrt(z0² + d²)
ankle_roll = -hip_roll
```

Opposite ankle roll cancels the hip-roll rotation after the pitch chain is
made flat. The increased sagittal reach also requires small changes to hip,
knee and ankle pitch. Their angles come from the existing inverse-kinematics
solver, using the actual new model's segment lengths and limits. Merely adding
hip roll with the old pitch targets would change the feet's height/placement.

For +10 mm, the endpoint per leg is approximately:

| Joint | Target (rad) |
| --- | --- |
| Hip roll | -0.01962490 |
| Hip pitch | -0.22274795 |
| Knee pitch | 0.39809635 |
| Ankle pitch | -0.17534841 |
| Ankle roll | +0.01962490 |

Both hip rolls have the same sign for a translated upright pelvis. This is
different from spreading the legs outward, where mirrored hip rolls have
opposite signs. The geometry preview from the earlier model milestone showed
leg spreading, not a planted-foot weight shift.

The endpoint transforms match each original foot position/orientation to
floating-point precision in the planned model. The control loop interpolates
41 precomputed poses; tests bound its intermediate foot-position discrepancy
below one micrometre. This kinematic check does not replace measured foot motion.

## Smooth time progression

For each four-second transition, normalized phase `s` runs from 0 to 1:

```text
u = 10s³ - 15s⁴ + 6s⁵
outward displacement = d × u
return displacement = d × (1 - u)
```

This gives zero endpoint velocity and acceleration for the scalar time law.
The precomputed angle/load table is interpolated along it. Interpolation is a
numerical approximation; it does not guarantee exact dynamic tracking or
continuous acceleration at every internal table boundary.

## Predicted COM and loads

All eleven link COMs are transformed and mass-weighted at each planned pose.
For the +10 mm pelvis request, the model predicts a **+7.1191 mm COM shift**.
The COM travels less than the pelvis because the feet remain planted and the
leg segment masses move by different amounts.

A static estimate assumes vertical reactions at each sole center, zero
horizontal force, and no foot contact moment. With feet at `yL = +0.085 m` and
`yR = -0.085 m`, lateral equilibrium gives:

```text
FL + FR = total_mass × gravity
FL × yL + FR × yR = total_mass × gravity × COM_y

FL = total_weight × (COM_y - yR) / (yL - yR)
FR = total_weight - FL
```

At the planned endpoint this gives approximately 76.654 N left and 64.806 N
right, versus 70.730 N each at center. These are **predicted forces, not measured
loads**. Finite-area feet can shift their pressure centers, so COM position
alone does not uniquely identify the actual left/right normal forces.

The existing static moment calculation is extracted into
`double_support_loads` and accepts specified vertical foot reactions. It sums
all distal weight moments and the assumed foot-reaction moment about each joint
axis. The resulting feedforward is bounded by the unchanged motor budget. It
is not an inverse-dynamics calculation and omits acceleration and horizontal
contact forces. All static assumptions remain explicit.

## Runtime safeguards and measurements

Motion is optional and disabled in the ordinary standing launch. In shift mode,
the controller subscribes to the existing 50 Hz contact sampler. Readiness
requires fresh contacts for both feet in addition to the standing checks.
After readiness, missing, stale or non-ground contact at either foot stops the
shift. IMU/joint timing, posture limits, torque clipping and antiwindup remain.

COM and sole geometry are measured independently by the Gazebo COM probe.
The analysis uses COM Y relative to the midpoint between the measured soles,
so translating the entire robot through foot sliding cannot mimic a COM shift.
Measured sole movement is checked separately. A separate read-only recorder starts before simulation warm-up and captures
Gazebo model pose, raw ROS contacts, and every received IMU sample. Closed-file
audits select the exact movement/return window. No new measurement processes
are launched at readiness.

Predeclared acceptance checks include:

- Correct shift direction and mean held COM within 2 mm of the static prediction.
- Independently measured held pelvis displacement within 2 mm of its request,
  and every final-three-second pelvis Y sample within 1 mm of baseline.
- Every recorded COM sample in the final three seconds within 1 mm of the
  pre-movement baseline, with the mean also reported.
- Foot-center drift no more than 1 mm, sole height range no more than 3 mm,
  and sole-corner ground heights within 3 mm.
- Both contacts fresh/grounded throughout the recorded controller program.
- Joint error no more than 0.01 rad during motion and 0.002 rad during the
  analyzed late hold/recovery intervals; speed no more than 0.05 rad/s.
- Full controller and COM coverage, constant expected mass, reconstructed
  named motor commands and bounded integral contributions.
- Existing passive damping and observed motor-input bounds.

The static prediction is a target/comparison, not an external force sensor.
Passing this experiment establishes a small bilateral COM shift, not validated
foot unloading or unsupported single-leg balance.

## Code organization

- `lateral_shift.py`: pure fixed-foot geometry, static load table and timed
  move/hold/return program. XML parsing/FK/static calculations happen before
  activation; the control loop performs only table interpolation.
- `roll_stance.py`: retains the original standing plan and extracts reusable
  `double_support_loads`. Zero-shift targets/feedforward are regression-tested.
- `roll_posture.py`: explicit optional integral-gain profile, with ordinary
  standing defaults preserved.
- `hold_roll_stance.py`: optional program/contact integration in the existing
  ten-joint ROS adapter. The normal standing mode remains the default.
- `roll_standing.launch.py`: optional shift parameters and contact sampler;
  original sensor/model/controller setup stays the same.
- `lateral_audit.py`: independently reconstructs timed targets and motor
  composition, checks measured COM/foot geometry and labels force estimates.
- Workspace `scripts/record_lateral_sensors.py`: read-only ROS/Gazebo acquisition,
  started before warm-up. It never publishes motor commands.
- Workspace `scripts/run_lateral_shift.py`: isolated ROS domain 77 and unique
  Gazebo partition, recorder lifecycle, fail-fast supervision and closed-file
  auditing. It stops its owned processes before analyzing their files.
- `tests/test_lateral_shift.py`: fixed-foot geometry, direction symmetry,
  timing/return, intermediate interpolation, budgets, invalid inputs and
  acceptance/rejection of synthetic evidence.

## Reproduce after building

From the workspace root, use a new output directory:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
python3 scripts/run_lateral_shift.py --gui --displacement 0.01 \
  --output evidence/my-left-com-shift-01
```

Use `--displacement -0.01` with a new output directory for the other direction.
The run starts the shift after standing readiness and its three-second baseline,
then stops its own simulation after measurements. Allow a few minutes at 0.25×.
The default standing runner and six-joint push website do not enable this motion.

## What the first physical test exposed

`evidence/lateral-shift-left-01` reached a held COM change of approximately
7.125 mm, close to the 7.119 mm prediction, with both feet still planted. It
nevertheless failed: joint errors and velocities grew during recovery and the
conservative IMU acceleration check also failed. The full program's recorded
maximum joint error was 0.0164 rad and speed was 0.1121 rad/s, above acceptance
limits. COM mean alone would have hidden the oscillation.

The candidate changes only roll integral gain in the optional shift mode:

```text
Hip roll / ankle roll Ki: 20 → 5
Pitch Ki stays: hip 20, knee 25, ankle 15
```

This slows accumulation of corrective roll torque. The ordinary standing
controller still defaults to the original roll Ki=20. The adapter explicitly
selects `LATERAL_KI` only when the shift program is enabled, and records the
actual gains. Caps, proportional/velocity gains, trajectory, physical damping
and model are unchanged. A test verifies default-standing and shift gains
remain distinct.

Recovery checks also require every sampled COM value in the final three seconds
to lie within 1 mm of baseline, in addition to the existing joint criteria.
A small average could otherwise hide a large alternating sway. This strengthens
settling verification rather than accepting the first run's near-zero mean.
The initial failure and its independent post-run diagnosis remain saved.

The IMU checker was originally designed for stationary standing. We retain its
strict near-upright/rate/acceleration bounds as an additional conservative check
for this very slow small motion; it is not a general IMU acceptance test for
faster future movements. The first run's separate joint errors/speeds already
showed a real settling problem, independent of that stationary-check assumption.


## Starting measurement before the strict phase

`lateral-shift-left-02` stopped before movement because IMU age reached 0.091 s
just after readiness, coinciding with launching external verification processes.
This is an infrastructure/timing failure, not evidence about the lower integral
gain. The original motion and stationary-sensor checks were not relaxed.

The runner now launches a single separate read-only recorder before simulation
warm-up. There is no post-readiness hardware-query or process-import burst.
It records Gazebo model pose, source-stamped raw contacts, and full-rate received
IMU samples. The controller and COM observer keep their separate recordings.
After program completion, all writers close and pure audits select the same
program interval from the source timestamps. They retain the same pose, contact,
IMU acceleration/rate and freshness bounds. Missing or stale samples still fail.
The runner also notices a stopped simulator/recorder immediately rather than
waiting for a long checker timeout.

Sensor tests reject stale cached IMU observations and a single raw acceleration
spike; those cannot be hidden by phase averages or recorder downsampling.


## First verified result

The revised +Y experiment (`evidence/lateral-shift-left-03`) passed after the
recording pipeline was moved before warm-up and shift-mode roll Ki was reduced
to 5. Ordinary standing still uses roll Ki=20.

| Metric | +Y measured result |
| --- | --- |
| Requested pelvis translation | +10.000 mm |
| Mean held pelvis translation | +10.0023 mm |
| Predicted held COM translation | +7.1191 mm |
| Mean held COM translation relative to feet | +7.1207 mm |
| Largest final-three-second COM offset | 0.0182 mm |
| Largest joint tracking error | 0.000308 rad |
| Largest late-hold/recovery tracking error | 0.0000593 rad |
| Peak joint speed | 0.01013 rad/s |
| Saturated motor samples | 0 |
| Fresh bilateral contacts / COM coverage / independent sensors / damping | Pass |

The final three-second mean COM offset was approximately -0.00136 mm. The peak
is reported as well because a small mean cannot establish settling. Measured
sole geometry stayed within the predefined drift/height limits. These small
errors describe an ideal rigid-body simulation, not hardware accuracy.

The nominal shift is modest: under the sole-center static assumptions, the
load split changes from 50/50 to approximately 54.2/45.8 percent. Both feet
remain well loaded in that model. Actual normal forces have not been measured.

## Mirrored result and final verification

The fresh -Y GUI-enabled run (`evidence/lateral-shift-right-01`) also passed.

| Metric | +Y headless | -Y GUI enabled |
| --- | --- | --- |
| Measured held pelvis shift | +10.0023 mm | -10.0022 mm |
| Measured held COM shift relative to feet | +7.1207 mm | -7.1207 mm |
| Largest final-three-second COM offset | 0.0182 mm | 0.0178 mm |
| Largest joint tracking error | 0.000308 rad | 0.000308 rad |
| Peak speed in final three seconds | 0.000299 rad/s | 0.000298 rad/s |
| Saturated motor samples | 0 | 0 |
| Independent pose/IMU/contact, COM, motor and damping checks | Pass | Pass |

The final suite passes **242 tests**. `lateral-shift-left-03/verification.json`
records the two cases, actual gain files, source snapshots, script hashes and
failed-attempt classifications. The recorder's first complete sample was at
simulation time 0.02 s in both successful runs, before effort activation.
Geometry, world, controller YAML and C++ sources match the prior standing
baseline. Ordinary-standing gains match its saved profile, and the standing
launch still defaults to `shift_enabled=false`.

There is one successful fresh run per direction. This verifies the selected
small shift and return, not repeatability across many trials, larger lateral
motion, external pushes or single-leg balance. The software gain change is
specific to shift mode; the acceptance limits and physical model were not
relaxed to turn the initial failure into a pass.

## View the recordings

```bash
python3 scripts/plot_lateral_shift.py evidence/lateral-shift-left-03 \
  --compare evidence/lateral-shift-left-01
python3 scripts/plot_lateral_shift.py evidence/lateral-shift-right-01
```

Each writes `response.png` in the successful run folder. The left comparison
shows the initial Ki=20 oscillation beside the successful Ki=5 response. The
force panel in the plot is explicitly a static estimate, not a measured signal.
The six-joint force-experiment website is unchanged; launch this ten-joint
motion with `scripts/run_lateral_shift.py`.

The next lesson should validate foot-load feedback before progressively
unloading one foot. COM movement and fresh collision messages alone cannot
establish how much vertical force each foot actually carries.

## Subsequent foot-force measurement

A separate sensing milestone now validates actual simulated contact forces;
see [FOOT_LOAD_VALIDATION.md](FOOT_LOAD_VALIDATION.md). Run with `--foot-loads`
to capture and validate them. The old plots/recordings above retain their
estimate labels; older runs without raw wrench payloads cannot be retroactively
converted into measured foot loads.
