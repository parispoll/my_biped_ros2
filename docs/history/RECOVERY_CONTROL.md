# Recovery control: from PD to bounded PID

This step fixes the small-disturbance posture experiment. It does not yet add
walking, foot placement, or IMU-based whole-body balance. The pelvis IMU still
provides measurements and a tilt shutdown; the controller uses joint feedback.

## What was wrong with the earlier experiment?

The earlier force-path audit accounted for the torque: the command reached DART,
constraint reactions were present, and DART's implicit damping used an intermediate
unconstrained velocity. That intermediate velocity can be nonzero even when the
final constrained joint velocity is zero. The apparent stationary damping torque
and remaining lean changed considerably with timestep. We should not tune a
controller around that effect or call mere survival successful recovery.

There is a second issue: a proportional controller needs a position error to
produce a correcting torque. Joint friction can therefore leave a small position
offset after the push, even after we change the damping integration.

## Change 1: make the damping input explicit

Before, the joint's URDF `dynamics` supplied damping directly to the physics
backend. In the new optional mode, that backend damping is zero and a Gazebo model
plugin applies this law at **each physics step**:

```cpp
const double v = velocity->Data()[0];
const double tau = -j.coefficient * v;
force->Data()[0] += tau;
```

Here `v` is the last constrained joint velocity, not the intermediate prediction.
The coefficients remain hip 2.5, knee 3.0, ankle 2.5 N m s/rad, on both legs.
Masses, geometry, joint friction, foot friction and motor limits are preserved.
This changes the numerical realization of damping; it is not evidence that every
DART damping model is incorrect. Explicit stepping has its own stability limits,
so we check this robot at several timesteps.

The plugin runs after motor commands and before physics. It adds passive torque
without resetting position or velocity. During startup it leaves joints without
an effort command alone. Its trace checks `tau = -b*v` and `tau*v <= 0` at every
sampled input and over every ten-step block. This check is not a proof of total
discrete energy convergence.

## Change 2: add bounded integral contributions

Before:

```python
requested = feedforward + kp * error - kd * velocity
```

After:

```python
integral = bounded_integral.update(error, simulated_dt, pd_request, motor_limits)
requested = pd_request + integral
command = clamp(requested, -motor_limit, motor_limit)
```

The full equation for joint i is:

`tau_i = clip(tau_ff_i + Kp_i*(q_target_i-q_i) - Kd_i*v_i + I_i, ±motor_limit_i)`

At each new joint-state timestamp:

`I_candidate_i = clip(I_i + Ki_i*error_i*dt, ±integral_limit_i)`

All six joints have an enabled integral term: hip/knee/ankle Ki = 80/100/60
N m/(rad s), with each integral contribution limited to ±1 N m. Existing Kp,
Kd and total motor caps (±6/10/6 N m) are unchanged. The mode is called **PID**
because it retains the original proportional and derivative terms.

Example: a persistent ankle error of +0.001 rad over 0.1 simulated second adds
`60 * 0.001 * 0.1 = +0.006 N m` to the integral contribution. A negative error
reduces it. Once the error disappears, the contribution can remain and supply
torque without requiring the proportional error to remain.

Conditional anti-windup freezes an increment that would push a saturated motor
request farther outside its cap. Increments that unwind the stored contribution
remain allowed. The PID readiness check waits at least 15 simulated seconds and requires all
joint errors below 0.0001 rad and speeds below 0.001 rad/s continuously for two
seconds before starting a baseline. This avoids comparing recovery with startup
drift. Integration starts only after effort activation and uses the
message's simulated timestamp, so changing wall-clock pacing does not change Ki.
A timestamp reversal or a gap greater than 0.05 s stops the PID experiment.

## Where each responsibility lives

| File | Responsibility |
| --- | --- |
| `src/my_robot_description/src/viscous_damping.cpp` | Physics-step passive damping and its trace |
| `src/my_robot_description/urdf/components/explicit_damping.xacro` | Transfer the original coefficients to the plugin; disable backend damping |
| `src/my_robot_description/my_robot_description/integral.py` | Bounded integration and anti-windup, independent of ROS |
| `src/my_robot_description/my_robot_description/damping.py` | Check damping trace law and input power |
| `src/my_robot_description/scripts/hold_effort_stance.py` | Combine feedforward, P, D and optional I; publish bounded commands |
| `src/my_robot_description/config/effort_standing.yaml` | Gains and integral contribution limits |
| `src/my_robot_description/launch/effort_standing.launch.py` | Select damping/controller modes and optional recordings |
| `scripts/run_effort_push.py` | Fresh reference and push, evidence, shutdown and validation |
| `tests/test_integral.py`, `tests/test_damping.py` | Numerical rules, physical-model isolation and launch-argument regression |

The original effort mode defaults remain `damping_model:=implicit` and
`controller_mode:=pd`. Position mode is unchanged. Compare modes explicitly;
do not combine results from different modes as repeats of one experiment.

## Reproduce

From the workspace terminal:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
python3 scripts/run_effort_push.py \
  --damping-model explicit --controller-mode pid \
  --real-time-factor .5 --force-x 2 --gui \
  --output evidence/my-pid-recovery-01
python3 scripts/plot_effort_push.py evidence/my-pid-recovery-01
```

The runner creates its own isolated simulation. Do not separately launch another
one for this command. It first runs a fresh zero-force reference, closes it, then
runs a fresh push simulation. Choose a new output folder each time so prior
measurements are preserved. Half-speed pacing helps message scheduling; pulse
and observation times are still simulated seconds.

The existing criteria require the entire last simulated second to be within
±0.1 degree pitch change, 0.01 rad/s pitch speed, 2 mm height change and
0.002 rad joint change from the recorded baseline, with fresh ground-contact
evidence for both feet. These thresholds and the 10 s observation are unchanged.

## Why the first two fixes were not enough

The retained evidence includes both unsuccessful attempts:

1. `evidence/recovery-fix/explicit-pd-2N`: explicit damping with the old PD
   controller stayed upright, but retained about 0.395 degree of pitch change
   and 0.00475 rad of largest joint change. It failed recovery.
2. `evidence/recovery-fix/explicit-pid-2N-01`: an ankle-only integral improved
   pitch, but the knee retained 0.00222 rad of joint change, above 0.002 rad.
   It also recorded before the integral had finished its startup adjustment.
3. `evidence/recovery-fix/explicit-pid-all-2N-01`: bounded integral on every
   joint plus the stricter PID readiness check passed. Peak pitch change was
   0.333 degree; final change was -0.00126 degree; largest final joint change
   was 0.000216 rad. The last sample outside any recovery band was 0.687 s
   after force-clear publication. That is a sampled return-band result for this
   run, not a general settling-time guarantee.

No return bands, torque caps, force magnitude or 10 s recovery duration were
relaxed to obtain the passing result. The comparison PD case ran at real-time
factor 1 and the PID case at 0.5; both use simulated times. Force application
has transport timing variability, so the curves are illustrative paired
experiments, not exactly identical measured impulses.

![PD and PID comparison](../evidence/recovery-fix/recovery-control.png)

The two early `explicit-pi-*` folders retain aborted development recordings:
one stopped for a stale command stream; the other stopped after the mode was
renamed to PID between reference and push. Neither counts as validation.

## What the physics audit checks now

An integral controller can continue changing torque while friction holds a
joint nearly stationary. Comparing average motor command with average total
physics input would then omit the explicit passive contribution and its timing.
The plugin therefore logs the motor input, passive damping and their sum at
matching simulation timestamps, plus maximum motor magnitude across each
10-step block. The physics observer must see exactly that sum. The DART observer
separately checks force mode, command/force agreement, zero backend damping and
transmitted-moment closure including the constraint impulse.

The static gravity equation is checked only when the final window is stationary
(all joint speeds below 1e-6 rad/s and position ranges below 1e-6 rad). Dynamic
windows retain the exact force-path checks and report the static gravity result
as not applicable. In the first audited 1 ms push, every joint's force-path check
passed but the ankle was still moving at up to about 0.0018 rad/s in the final
window. Applying a static-only equation to that window was an analysis error.
The original report is retained as `physics-audit-static-assumption.json`.

The timestep runner now accepts explicit mode, PID mode, force and wall-clock
pacing arguments. It can reuse individually completed, audited recordings from
an interrupted study after checking the frozen model/controller hashes and
protocol; it does not count aborted recordings as completed trials.

## Read the plots as a control-system story

1. The shaded vertical strip marks the requested push interval. The force is an
   external input, not a joint command.
2. The orange PD pitch trace stays displaced. The blue PID trace leaves the green
   pitch band, then returns. Returning is a stronger result than merely staying
   upright.
3. Each joint must also return. A pelvis-only check would have missed the failed
   ankle-only experiment's knee offset.
4. The P term responds immediately to angle error. The D term opposes motion.
   The I term changes more slowly and can supply torque after the error becomes
   small. It is bounded separately, then the **total** motor command is bounded
   again. Passive physical damping is an additional simulated joint force, not
   part of that motor command.

To inspect the code in VS Code, open this workspace root, then read
`integral.py` first, `hold_effort_stance.py` second, and
`viscous_damping.cpp` third. The first file has no ROS dependency and is the
smallest place to understand the new controller behavior.

A longer no-push check is available with the same modes:

```bash
python3 scripts/run_effort_standing.py \
  --damping-model explicit --controller-mode pid --real-time-factor .5 \
  --duration 30 --output evidence/my-pid-standing-01
```

For the instrumented timestep comparison:

```bash
python3 scripts/run_timestep_comparison.py \
  --damping-model explicit --controller-mode pid --force-x 2 \
  --real-time-factor .5 --steps .001 .0005 .00025 \
  --output evidence/my-pid-resolution-01
```

This runs one push and one fresh reference at each step by default. A three-step
check with one push per step is a sensitivity check, not the earlier two-repeat
resolution screen and not a formal convergence proof. `--repeats 2` requests the
six-push version. Keep every model/controller setting fixed throughout a study.

Additional wiring and verification changes:

- `CMakeLists.txt` builds and installs the new Gazebo damping library.
- `urdf/effort_standing.urdf.xacro` and `my_robot_description/description.py`
  expand the selected damping variant; their default remains the legacy variant.
- `scripts/run_effort_standing.py` accepts the same mode/pacing arguments for a
  longer no-push verification, with wall-time deadlines scaled to the pacing.
- `scripts/analyze_physics_probe.py` and `my_robot_description/physics_audit.py`
  distinguish the explicit input sum from a stationary legacy audit and gate
  static gravity comparison on a stationary window.
- `scripts/run_timestep_comparison.py` freezes the new damping/integral source
  files as well as the existing model/controller files. Old comparisons using
  different source hashes or modes cannot be reused as matching repeats.
- `scripts/plot_recovery_control.py` creates the four-panel teaching plot.
  `plot_effort_push.py` now labels an ankle change neutrally instead of assuming
  it retains an offset. `plot_timestep_comparison.py` reads force/duration from
  the protocol instead of hardcoding a 1 N request.
- The experiment runners now give a clear argument error for an existing output
  directory and instruct you to choose a new name, before launching a simulator.
- `tests/test_physics_audit.py` covers the explicit force sum, hidden backend
  damping, motor bounds and the static/dynamic distinction. The README links to
  this guide.

## Verified push results

| Push / physics step | Peak pitch change | Final pitch change | Recovery / physics audit |
| --- | ---: | ---: | --- |
| +2 N / 1 ms | 0.3269° | -0.001751° | passed / passed |
| +2 N / 0.5 ms | 0.3249° | -0.001585° | passed / passed |
| +2 N / 0.25 ms | 0.3240° | -0.001803° | passed / passed |
| -2 N / 1 ms, GUI enabled | 0.5063° | -0.025449° | passed / native observers off |

The observed positive force-input impulses were 0.402, 0.403 and 0.401 N s (nominal 0.4 N s); their relative spread was 0.498%. All these pushes had fresh contact evidence for both feet, zero saturated controller samples, and a passing full-trace damping-law/motor-bound check.

The final-pitch difference between 0.5 and 0.25 ms was 0.000218 degree. There is one audited positive push per timestep; this is a practical sensitivity check, not the earlier two-repeat screen.

The forward and backward peaks differ. Foot/stance geometry, friction and initial controller state can produce directional differences; this test does not establish their relative contributions. Both directions met the same return bands.

Evidence: `evidence/recovery-fix/pid-resolution-final/comparison.json` references the three raw cases. The backward run is in `evidence/recovery-fix/pid-backward-2N-01`. The first additional successful forward run is retained in `explicit-pid-all-2N-01`.

![Three timestep responses](../evidence/recovery-fix/pid-resolution-final/comparison.png)


## Final standing and software checks

The explicit/PID no-push experiment passed 30 simulated seconds after readiness. Height remained 0.554486231 m; maximum IMU tilt was 0.004835 degree. Both feet had 30,000 fresh ground-contact samples and no motor command saturated. The posture summary also conservatively includes warmup samples from two seconds after activation; the pose/contact/IMU windows start after PID readiness.

The final package build and all **100 unit tests** passed. Model/controller hashes still match the validated timestep study. `evidence/recovery-fix/validation.json` links the five passing push experiments and the standing result. All runner-owned simulations stopped cleanly.

This gives us a checked small-disturbance posture baseline for the next development step: deliberately adding sagittal body-balance feedback. It does not establish larger-push recovery, lateral balance, stepping or walking.
