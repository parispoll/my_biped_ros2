# Step 5g: change only the physics timestep

The follow-up [repeated resolution study](RESOLUTION_STUDY.md) now adds 0.25 ms
and compares two pushes per resolution. This page records the original one-pair comparison.

The smaller timestep produced a substantially smaller remaining lean in this
comparison. It did **not** make the robot pass the recovery criteria. The normal
simulation default remains 1 ms while we investigate numerical sensitivity.

## 1. What does a timestep mean?

A physics timestep is the amount of simulated time advanced by one physics solve:

```
1 ms   = 0.0010 s -> 1,000 physics steps per simulated second
0.5 ms = 0.0005 s -> 2,000 physics steps per simulated second
```

Both cases keep the controller-manager period at 4 ms (250 Hz). There are therefore
four physics steps per manager period at 1 ms, and eight at 0.5 ms. The controller's
angle targets, gains and command caps are not changed.

The real-time factor is a different setting. Both cases use 0.25, so a simulated
second takes approximately four wall-clock seconds. That slower pacing helps the
native logging and ROS subscriptions keep up. It does not change the simulated
push duration, observation duration, gravitational acceleration or damping values.

## 2. The controlled protocol

Each timestep gets a fresh no-push reference and a fresh 1 N push simulation:

| Setting | 1 ms case | 0.5 ms case |
| --- | ---: | ---: |
| Physics timestep | 0.001 s | 0.0005 s |
| Controller-manager rate | 250 Hz | 250 Hz |
| Requested push | +1 N for 0.2 s | +1 N for 0.2 s |
| Pre-push baseline | 2 simulated seconds | 2 simulated seconds |
| Post-clear observation | 10 simulated seconds | 10 simulated seconds |
| Wall-clock pacing factor | 0.25 | 0.25 |
| Initial knee target | 0.4 rad | 0.4 rad |
| Joint friction | 0.2 | 0.2 |
| Hip/knee/ankle passive damping | 2.5 / 3.0 / 2.5 | 2.5 / 3.0 / 2.5 |
| Hip/knee/ankle command caps | 6 / 10 / 6 N m | 6 / 10 / 6 N m |

Masses, inertias, collision geometry, foot friction, sensors, PD gains and startup
checks are also held fixed. The runner stores hashes of the model/controller files
and checks that they have not changed during the comparison. The requested timestep
is checked against the timestep read directly from DART.

The initialization targets are identical, but the actual settled baseline pose can
vary with timestep. Each response is therefore measured relative to its own actual
pre-push pose, as in the earlier disturbance tests. The raw summaries preserve
those baseline measurements.

## 3. Results of the first paired comparison

Evidence: `evidence/timestep-comparison/paired-01`.

| Measurement | 1 ms | 0.5 ms |
| --- | ---: | ---: |
| Remaining pitch change after observation | **0.480791 deg** | **0.212176 deg** |
| Largest remaining joint-position change | 0.008391 rad | 0.003703 rad |
| Final per-ankle motor input | +0.625237 N m | +0.292778 N m |
| Final per-ankle implicit damping contribution | -0.515144 N m | -0.241706 N m |
| Final per-ankle joint-constraint contribution | +0.108433 N m | +0.045720 N m |
| Final per-ankle transmitted moment | +0.218525 N m | +0.096792 N m |
| Observed physics-input impulse | 0.2010 N s | 0.2000 N s |
| Continuous ground-contact evidence | Yes | Yes |
| Saturated controller samples | 0 | 0 |
| Physics force-path/accounting audit | Passed | Passed |
| Recovery criteria | **Failed** | **Failed** |

The remaining pitch change is about **55.9% smaller** at 0.5 ms in these runs.
The magnitude of the implicit damping contribution is about **53.1% smaller**.

This is evidence that the recorded response is sensitive to physics resolution
under this protocol. It is not a demonstration that the smaller timestep is
physically correct, and it is not a completed recovery-controller repair.

### Check the input before attributing the outcome

The nominal requested impulse is:

```
1 N × 0.2 s = 0.2 N s
```

The physics-input observations are 0.201 N s and 0.200 N s. Their relative spread
is approximately 0.5%. We report this transport/timing variation instead of claiming
that the actual input histories were identical. The larger posture difference is
consistent with timestep sensitivity, but one trial per timestep and nonlinear
friction/contact effects do not let us assign an exact causal percentage solely
to timestep.

The no-push references both passed and showed negligible pitch change during their
recordings. That supports interpreting the observed movement as a disturbance
response rather than baseline drift.

![Timestep response, torque terms and input check](../evidence/timestep-comparison/paired-01/comparison.png)

## 4. The torque balance still works at both resolutions

For the final left-ankle mean values:

```
1 ms:
  +0.625237 - 0.515144 + 0.108433 ≈ +0.218525 N m

0.5 ms:
  +0.292778 - 0.241706 + 0.045720 ≈ +0.096792 N m
```

These are motor input + implicit passive damping + joint-constraint impulse divided
by timestep. The existing native audit still checks the terms against the recorded
transmitted joint moment, all six joints' command delivery and FORCE mode, travel
margins, nominal Coulomb bounds, and zero external force in the final interval.

The balance explains the recorded numerical result. It does not turn the
transmitted wrench into a calibrated motor-torque measurement, nor establish that
this simulator's equilibrium should be expected on hardware.

## 5. Why both recovery tests still fail

The pitch return band remains ±0.1 degree from the measured baseline. Even the
0.5 ms case ends at about +0.212 degree, outside that band. Its largest remaining
joint change, 0.003703 rad, is also outside the ±0.002 rad joint-change band.

The smaller offset is an observation worth following up. We did not enlarge the
bands or change the gains to turn a failure into a pass. Both cases still have
continuous foot-contact evidence and no command saturation.

## 6. What changed in the code?

| File | Before | Now |
| --- | --- | --- |
| Package `launch/effort_standing.launch.py` | The copied world retained its fixed 1 ms step. | Optional `physics_step` sets 1 ms or 0.5 ms in a temporary world copy. Default stays 1 ms; the saved world is untouched. Pacing can be 0.25–1. |
| Workspace `scripts/run_effort_push.py` | No timestep argument; shorter diagnostic pacing options. | Exposes `--physics-step`, records requested settings, and permits common 0.25 pacing for this comparison. |
| Workspace `scripts/effort_push_experiment.py` | Recorder wall deadline used the shared default. | Accepts a wall timeout sized for a slow diagnostic run. |
| Workspace `scripts/push_experiment.py` | Fixed 35 s wall deadline. | Optional finite positive wall timeout, default still 35 s. Sensor freshness and recording/recovery rules are unchanged. |
| Package `my_robot_description/physics_audit.py` | No reusable duration selector for native samples. | `final_getter_window` selects approximately one simulated second using DART's measured timestep and sequence counters. |
| Workspace `scripts/analyze_physics_probe.py` | The native window was tied to a fixed sample count. | Uses the duration selector and records the measured DART timestep. |
| Workspace `scripts/run_timestep_comparison.py` (new) | — | Runs fresh reference/push pairs, checks timestep/model consistency and saves a compact comparison. |
| Workspace `scripts/plot_timestep_comparison.py` (new) | — | Shows baseline-relative tilt, remaining lean, physics torque terms and observed input impulse. |
| Workspace `scripts/plot_physics_audit.py` | Figure caption assumed 1 ms. | Caption uses the measured timestep when plotting either case. |
| `tests/test_physics_audit.py` | Force-path and torque-accounting checks. | Four additional tests cover window duration at both steps, short records, varying timestep and gaps. |

### A subtle measurement error this avoids

The native logger writes every tenth physics step:

```
At 1 ms:   100 logged samples represent about 1 second.
At 0.5 ms: 100 logged samples represent about 0.5 second.
```

Keeping a fixed sample count would quietly halve the analysis window. The updated
selector uses sequence difference × measured timestep to preserve the duration.
Four new tests check this behavior. All **79 Python tests pass**.

The longer wall deadline is also necessary: the 12.2 simulated-second experiment
alone needs about 48.8 wall seconds at factor 0.25, before discovery overhead.
This does not relax the simulated-time freshness thresholds or any recovery band.

## 7. Inspect the saved result first

No simulation is needed to redraw the comparison:

```bash
cd '/home/parispoll/Documents/ChatGPT/robotics carrer'
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 scripts/plot_timestep_comparison.py evidence/timestep-comparison/paired-01
```

Open `comparison.png` in that folder. `comparison.json` contains the compact
measurements, actual-input comparison and unchanged-file hashes. Each timestep
subfolder contains its own normal experiment summary, full native recordings and
`physics-audit.json`.

To reproduce the whole paired experiment, use a new folder:

```bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash

python3 scripts/run_timestep_comparison.py \
  --output evidence/my-timestep-comparison

python3 scripts/plot_timestep_comparison.py evidence/my-timestep-comparison
```

It runs four simulations sequentially and closes each one. Allow several minutes
at the diagnostic pacing. The ordinary effort-standing launch still uses its
original default step and loads neither observer unless explicitly requested.

For a repeated paired study, the runner also supports `--repeats 2`. Its second
repeat reverses the timestep order. That option has not been executed as part of
this first comparison; the reported results are one fresh push per timestep.

## 8. What we can conclude

There is a substantial difference between these two recorded resolutions with the
same controller and physical coefficients. This supports validating simulation
resolution before tuning the controller around a particular residual lean.

Two resolutions do not establish numerical convergence, and one push per resolution
does not characterize variability. Repeats and a further timestep refinement would
be the next checks before selecting a new default or treating a smaller offset as
a physical-model improvement. The current task therefore leaves the 1 ms default
in place and preserves both failed-recovery results.
