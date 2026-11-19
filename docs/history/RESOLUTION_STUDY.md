# Step 5h: repeat the experiment and add 0.25 ms

**The runs are repeatable enough to reveal a timestep effect, but the tested
resolutions do not meet our practical resolution-agreement screen.** The mean
remaining lean falls as the timestep becomes smaller, while the change between
0.5 and 0.25 ms is still much larger than observed repeat variation.

No controller gains, target angles, torque caps, masses, inertias, damping or
friction coefficients were changed. The normal timestep remains 1 ms.

## 1. What we ran

There are two independent pushes at each of three resolutions:

- 1 ms: the earlier verified trial plus one new trial;
- 0.5 ms: the earlier verified trial plus one new trial;
- 0.25 ms: two new trials.

Each push has its own fresh no-push reference. That gives six pushes and six
reference simulations in the combined dataset. Eight simulations were newly run;
four were reused from `evidence/timestep-comparison/paired-01` after checking its
completed status, tracked model/controller hashes and protocol. Installed Gazebo
and ROS vendor-package versions were also checked against the prior audit and
matched. The reused recordings were referenced in place, not copied or overwritten.

The fresh case order was 0.25 ms, 0.25 ms, 0.5 ms, 1 ms. With the earlier pair,
this corresponds to the protocol's forward order followed by reverse order.
This reduces a simple monotonic run-order confound; it is not a randomized study.

Every case kept:

| Setting | Value |
| --- | --- |
| Push request | +1 N in world X for 0.2 simulated second |
| Baseline / recovery observation | 2 s / 10 s of simulated time |
| Controller manager | 250 Hz |
| Knee target | 0.4 rad |
| Joint friction | 0.2 |
| Passive hip/knee/ankle damping | 2.5 / 3.0 / 2.5 |
| Hip/knee/ankle torque caps | ±6 / ±10 / ±6 N m |
| Wall-clock pacing | 0.25 |

The controller settings and physical-source files tracked by the runner stayed
byte-for-byte unchanged throughout collection. DART's recorded timestep was checked
against the requested 1, 0.5 or 0.25 ms in every case. Each response is measured from
its own actual pre-push baseline.

## 2. The criteria were chosen before the new fine-step data

This is a **practical screen for the recorded posture metrics**, not a formal
numerical error bound or a proof of convergence.

We required both of these changes between the two finest resolutions to be small:

- difference in mean remaining pitch change: at most **0.01 degree**;
- difference in mean largest-joint-offset metric: at most **0.0002 rad**.

These are one tenth of the existing recovery bands. The repeat range at each of
the two finest steps also had to stay within those thresholds. All force-path
audits had to pass and the observed push-input impulse spread had to remain within
5%. The thresholds are stored in the study checkpoint and in `resolution_study.py`.
They were not widened after seeing the results.

The joint metric is the largest absolute joint change from baseline in each run.
For this symmetric model the ankle dominates. Pitch and ankle offset are coupled
by the flat-foot geometry, so these are not independent statistical tests.

## 3. The results

Evidence: `evidence/timestep-comparison/resolution-study-01`.

| Physics step | Push 1: remaining pitch | Push 2: remaining pitch | Mean | Observed range |
| --- | ---: | ---: | ---: | ---: |
| 1 ms | 0.480790710 deg | 0.480790835 deg | **0.480791 deg** | <0.000001 deg |
| 0.5 ms | 0.212175888 deg | 0.215284085 deg | **0.213730 deg** | 0.003108 deg |
| 0.25 ms | 0.108164458 deg | 0.107745210 deg | **0.107955 deg** | 0.000419 deg |

A range is `maximum - minimum` of two trials. It is not a confidence interval and
does not characterize every possible run. The extremely small observed 1 ms range
should not be interpreted as a general guarantee of simulator repeatability.

### Compare repeat variation with the resolution change

```
Mean pitch change from 1 ms to 0.5 ms:
  0.480791 - 0.213730 = 0.267061 degree

Mean pitch change from 0.5 ms to 0.25 ms:
  0.213730 - 0.107955 = 0.105775 degree

Largest observed repeat range at the two finest steps:
  0.003108 degree

Predeclared refinement threshold:
  0.010000 degree
```

The last refinement change is about **34 times** the larger fine-step repeat
range, and about **10.6 times** the pitch threshold. It is therefore much larger
than the repeat variation observed in this dataset.

The largest-joint-offset metric tells the same geometric story:

| Step | Mean largest remaining joint change |
| --- | ---: |
| 1 ms | 0.008391363 rad |
| 0.5 ms | 0.003730274 rad |
| 0.25 ms | 0.001884160 rad |

The last difference is **0.001846114 rad**, exceeding the **0.0002 rad** screen
threshold. Repeat variation is below the thresholds, but resolution agreement is
not. The saved result is `resolution_screen_passed: false`.

![Repeat variation compared with refinement changes](../evidence/timestep-comparison/resolution-study-01/resolution-screen.png)

## 4. Was the push comparable?

| Step | Observed physics-input impulses | Mean |
| --- | --- | ---: |
| 1 ms | 0.2010, 0.2010 N s | **0.2010 N s** |
| 0.5 ms | 0.2000, 0.2020 N s | **0.2010 N s** |
| 0.25 ms | 0.20125, 0.20075 N s | **0.2010 N s** |

All three mean impulses match. The total individual impulse spread is about 1%
of the overall mean, inside the predeclared 5% quality check. The requested nominal
impulse remains 1 × 0.2 = 0.2 N s.

These are observations of the force input to simulation. They are not independent
hardware force measurements. Matching mean impulses also does not mean every
input time history was identical; transport and publication timing vary. With
nonlinear friction/contact and only two pushes per step, we do not assign an exact
causal percentage solely to timestep or claim formal statistical significance.

Nevertheless, the large, consistent resolution change with comparable inputs and
much smaller observed repeat ranges supports investigating numerical sensitivity
before interpreting the offset as a controller defect.

## 5. What happened to the physics terms?

Mean final left-ankle terms across the two pushes:

| Step | Motor input | Implicit passive damping | Joint constraint | Transmitted moment |
| --- | ---: | ---: | ---: | ---: |
| 1 ms | +0.625236 N m | -0.515144 N m | +0.108433 N m | +0.218525 N m |
| 0.5 ms | +0.294405 N m | -0.242220 N m | +0.045311 N m | +0.097497 N m |
| 0.25 ms | +0.166327 N m | -0.123719 N m | +0.006532 N m | +0.049140 N m |

The earlier force-accounting checks passed at every resolution, including all six
joints' command agreement, FORCE mode, constraint bounds and stationary-window
checks. Native logging confirmed the actual timestep and zero external push input
in the final interval.

Both feet retained fresh ground-contact evidence in every recorded push. No
controller samples saturated. The PD gains and passive coefficients stayed fixed;
changing the numerical resolution changed the resulting posture and the internal
split-step contributions.

![All recorded responses and input checks](../evidence/timestep-comparison/resolution-study-01/comparison.png)

## 6. Recovery is a different question

None of the six pushes passed the existing recovery criteria. At 0.25 ms, the
largest remaining joint change is inside the ±0.002 rad joint band, but both
remaining pitch changes, about 0.108 degree, are still outside ±0.1 degree.

Three distinctions matter:

- **Recording completed:** we obtained a valid run and its measurements.
- **Physics audit passed:** the logged commands, engine terms and force accounting
  agree under the audit's assumptions.
- **Recovery / resolution screen passed:** separate conditions about returning
  to the baseline or agreement between resolutions. Both are false here.

A smaller or even passing recovery offset at one timestep would not itself prove
that the numerical solution had converged or that the model matched hardware.

## 7. What changed in the files?

| File | Change |
| --- | --- |
| Package `launch/effort_standing.launch.py` | Adds 0.00025 s to the allowed diagnostic steps. The default remains 0.001 s. |
| Workspace `scripts/run_effort_push.py` | Accepts `--physics-step 0.00025`. Other default settings remain unchanged. |
| Workspace `scripts/run_timestep_comparison.py` | Adds selectable steps, matching prior-data reuse, provenance paths and the resolution-screen result. The original two-step/single-repeat default remains available. |
| Package `my_robot_description/resolution_study.py` (new) | Pure calculation of repeat ranges, means, refinement differences, input comparability and the predeclared screen. No ROS/Gazebo runtime needed. |
| Workspace `scripts/plot_timestep_comparison.py` | Supports a third color, referenced prior recordings, replicate dots/range whiskers, and the additional `resolution-screen.png`. |
| `tests/test_resolution_study.py` (new) | Checks genuine metric agreement versus merely decreasing values, excessive variation, unequal/zero inputs, failed audits, missing/duplicate repeats and non-finite measurements. |
| `tests/test_physics_audit.py` | Extends the duration-selector test to 0.25 ms: one simulated second needs about 400 logged native samples. |

There is no controller or physical-model repair in this step. The C++ observers
are unchanged. All **88 Python tests pass**.

## 8. Inspect the evidence without rerunning simulation

```bash
cd '/home/parispoll/Documents/ChatGPT/robotics carrer'
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 scripts/plot_timestep_comparison.py \
  evidence/timestep-comparison/resolution-study-01
```

Open `comparison.png` and `resolution-screen.png` in that folder. The full numerical
assessment is under `resolution_screen` in `comparison.json`. Each trial records its
origin; reused trial data is at its absolute `recording_path`, while fresh trials
are local subfolders. `versions.txt` records the installed vendor packages.

For the code, start with `summarize_resolution_study` in `resolution_study.py`.
It groups rows by timestep, requires two distinct repeat identities, calculates
means/ranges, compares the finest pair and applies the fixed quality thresholds.
The recorder/physics audit remain separate from this interpretation step.

## 9. Reproduce it

After sourcing ROS, rebuild and source the workspace:

```bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
```

To repeat the same collection strategy, reusing the verified earlier pair:

```bash
python3 scripts/run_timestep_comparison.py \
  --steps 0.001 0.0005 0.00025 \
  --repeats 2 \
  --seed-comparison evidence/timestep-comparison/paired-01 \
  --output evidence/my-resolution-study

python3 scripts/plot_timestep_comparison.py evidence/my-resolution-study
```

Choose a new output folder each time. The seed must be completed and have matching
tracked source hashes and protocol; its force-path audits must pass. For a wholly
fresh study, omit `--seed-comparison`. That runs twelve simulations instead of eight
new ones. Allow several minutes at the diagnostic pacing and leave Gazebo unpaused.

The sequence uses headless owned simulations and stops each one before the next.
It preserves sensor freshness, startup checks, force bounds, the observation window
and recovery thresholds. The default launch still uses 1 ms and loads neither
observer without explicit diagnostic options.

## 10. Interpretation and practical decision

The offsets are repeatable in the observed trials, but agreement between 0.5 ms
and 0.25 ms is not yet within the declared tolerances. This study therefore does
not establish a resolution-insensitive result or justify selecting 0.25 ms as a
validated new default.

Further refinement could test whether the changes continue to shrink. Another
focused investigation would validate the passive-damping/contact discretization,
whose intermediate terms we already observed, rather than tune gains to cancel
one timestep's artifact. Either should be a separate controlled step. The current
study leaves normal settings unchanged and preserves the failed results.
