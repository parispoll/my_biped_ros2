# Gradual partial unloading, with measured force guards

The next walking prerequisite is to transfer more weight to one leg and return smoothly. This experiment keeps both feet on the floor. A 35% target means that the lighter foot still supports 35% of the total vertical load; it does not mean lifting that foot.

## 1. Calculate the target

The ten-joint robot weighs `W = m g = 14.42 × 9.81 = 141.4602 N`.
At center, each foot supports approximately `W/2 = 70.7301 N`.

| Target split | Lighter foot | Heavier foot | Predicted pelvis shift | Predicted COM shift |
|---|---:|---:|---:|---:|
| 60/40 | 56.5841 N | 84.8761 N | 23.8798 mm | 17.0 mm |
| 65/35 | 49.5111 N | 91.9491 N | 35.8204 mm | 25.5 mm |

For slow motion with sole-center reactions at lateral positions ±0.085 m, vertical force balance gives `F_L + F_R = W`. Moment balance gives `0.085(F_L − F_R) = W y_COM`. Thus a leftward COM shift of 0.0255 m predicts `F_L/W = (1 + 0.0255/0.085)/2 = 0.65`.

This is a planning approximation. Contact-wrench measurements independently verify what the simulation actually does.

## 2. Convert the COM target into joint angles

The planner moves the pelvis sideways while keeping each sole fixed and level. Hip roll accommodates the lateral displacement; ankle roll cancels it. The pitch-chain inverse kinematics adjusts hip, knee and ankle pitch for the changed leg geometry. The robot's individual link masses determine the COM, so pelvis displacement and COM displacement differ.

`partial_unload.py` searches candidate pelvis shifts until it brackets the requested load fraction, then bisects that interval. Every pose must satisfy the existing joint limits and static torque limits. It caches 41 poses and checks the whole cached path, rather than only the endpoint.

At the leftward 65/35 endpoint, each leg has approximately these joint targets:

| Joint | Target (rad) |
|---|---:|
| Hip roll | −0.070191 |
| Hip pitch | −0.211086 |
| Knee pitch | 0.374881 |
| Ankle pitch | −0.163794 |
| Ankle roll | +0.070191 |

The largest predicted torque utilization is about 42.7%. Existing motor caps remain 6 Nm for hip/ankle joints and 10 Nm for knees. These are model command limits, not hardware ratings established by this experiment.

## 3. Move slowly, and check measured support

After ordinary standing readiness, the sequence is:

| Phase | Simulated duration |
|---|---:|
| Centered baseline | 3 s |
| Smooth outward movement | 8 s |
| Hold the target | 8 s |
| Smooth return | 8 s |
| Centered recovery | 10 s |

The quintic time interpolation starts and ends each movement smoothly. At real-time factor 0.25, this 37-second sequence takes roughly 148 wall seconds, plus startup and settling.

During this opt-in experiment, the controller requires fresh measured forces from both feet. Each must retain at least 25% of total vertical support. The summed force must remain within 5% of weight, source age must remain between −0.02 and +0.05 simulated seconds, and each requested motor torque must stay below 80% of its cap. Invalid measurements stop the experiment; missing contacts are not replaced with invented zero forces.

The measured forces provide a safety guard. They do not continuously correct the joint target to regulate a force setpoint. The same lateral posture gains are retained.

## 4. What changed in the code

| File | Before | Now |
|---|---|---|
| `my_robot_description/partial_unload.py` | No unloading planner | Separate target derivation, 37 s program, live force eligibility and recorded guard audit |
| `my_robot_description/lateral_shift.py` | Small ±10 mm motion only | Reusable fixed-foot geometry; ordinary small-shift interface still limits requests to ±10 mm |
| `scripts/hold_roll_stance.py` | Contacts qualified geometry | Optional decoded foot-force guards and 80% torque guard during unloading |
| `launch/roll_standing.launch.py` | Standing/small-shift options | Explicit unloading enable, fraction and direction options |
| `scripts/run_lateral_shift.py` | Fixed small-shift trial | Opt-in unloading trial with mandatory raw-force capture and post-run validation |
| `lateral_audit.py`, `foot_loads.py` | Fixed analysis windows | Program-specific windows, preserving existing acceptance thresholds |
| `scripts/plan_partial_unload.py` | No preview | Offline front-view geometry and predicted torque utilization |
| `scripts/analyze_foot_loads.py` | 25 s plot | Also handles the unloading schedule and shows its load floor |
| `tests/test_partial_unload.py` | No unloading tests | Target derivation, mirror symmetry, scheduling, force guards and torque audit tests |

Paths in this table are relative to `src/my_robot_description/` except runner/analysis/preview scripts, which are under the workspace `scripts/` folder. The controller adapter is also under the package's `scripts/` folder.

## 5. Reproduce it

From the workspace, source and build:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
python3 scripts/run_lateral_shift.py --gui --unload-fraction 0.4 --direction left \
  --output evidence/my-unload-40-left-01
python3 scripts/analyze_foot_loads.py evidence/my-unload-40-left-01
```

Use a fresh output directory each time. After inspecting a passing 60/40 run, use `--unload-fraction 0.35`; `--direction right` mirrors the transfer. The output contains raw contacts, joint commands, independent COM/pose/IMU data, source hashes and separate audits.

## 6. Interpretation and next question

Passing this experiment establishes slow, bilateral partial unloading and return for these specific trajectories. It does not establish zero-load sensing, single-leg balance, resistance to disturbances while unloaded, or walking. Before lifting a foot, derive whether a suitable pose can bring its load near zero within the existing joint and torque limits, and design measurement handling for genuine loss of contact.

## Recorded results

254 unit tests and the package build passed. All three live trials passed motion, sensor, COM, measured-force, physical-damping and live-guard audits.

| Trial | Measured lighter-foot share | COM shift (mm) | Peak requested torque / cap |
|---|---:|---:|---:|
| `partial-unload-40-left-01` | 39.9995% | 17.0018 | 31.28% |
| `partial-unload-35-left-01` | 34.9993% | 25.5028 | 42.75% |
| `partial-unload-35-right-01` | 34.9994% | -25.5026 | 42.75% |

Plots and raw evidence are in the corresponding `evidence/` folders. The mirrored trial also contains `verification.json`, `unit-tests.txt` and `build.txt`. Physical model, posture gain module, motor-cap configuration and C++ plugin hashes match the previous measured-load milestone.

The follow-up [unloading-limit analysis](UNLOADING_LIMITS.md) calculates the limits of this path before attempting zero load.
