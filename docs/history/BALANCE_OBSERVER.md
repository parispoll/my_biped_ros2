# Step 6a: observe a body-balance calculation

We are now calculating a possible response to pelvis lean. This step records the
answer without sending it to the motors. The existing bounded joint PID posture
controller still drives the robot. Its gains, integral state, targets, torque
caps, readiness checks and damping model are unchanged.

## 1. What the IMU tells us

The IMU orientation is a quaternion relative to the world. `balance.py` converts
it to pitch, the forward/backward angle of the pelvis. Positive pitch means the
pelvis leans toward world +X. The target is world upright, **0 rad**, rather than
the mean of the experiment's baseline.

The IMU gyroscope reports angular velocity in its own body axes. Those axes match
`base_link` in this model. At zero roll, gyro Y is pitch rate. With nonzero roll,
the correct ZYX Euler conversion is:

```python
pitch_rate = cos(roll) * gyro_y - sin(roll) * gyro_z
```

This matters because a body-frame angular velocity component is not generally
identical to an Euler-angle derivative. The pure calculation checks quaternion
normalization and rejects nonfinite readings. The ROS observer also checks the
frame, measurement availability and increasing IMU timestamps.

## 2. Calculate a restoring body moment

Our candidate body-level law is:

`desired_body_moment = -Kp * (pitch - target_pitch) - Kd * pitch_rate`

The angle term opposes leaning. The rate term opposes body rotation. The initial
candidate gains are Kp = 20 N m/rad and Kd = 4 N m s/rad. They are deliberately
separate from the **joint** PID gains and are not yet validated for application.
They are inspection values, not the result of a closed-loop stability analysis.

## 3. Why the ankle proposal has a positive sign

The joint axes point along +Y. For flat grounded feet, the sagittal orientation
relationship is:

`foot_pitch = body_pitch + hip_angle + knee_angle + ankle_angle`

With a flat foot and fixed hip/knee angles, increasing ankle angle decreases body
pitch. Also, a positive ankle motor moment on the foot has an equal and opposite
reaction on the upper robot. Under an equal double-support approximation:

`body_moment = -(left_ankle_motor_moment + right_ankle_motor_moment)`

Therefore the candidate **motor** correction at each ankle is:

```python
error = pitch - target_pitch
p_total = Kp * error
d_total = Kd * pitch_rate
raw_per_ankle = 0.5 * (p_total + d_total)
proposal_per_ankle = clamp(raw_per_ankle, -0.5, +0.5)
```

Positive forward pitch produces a positive ankle motor proposal. Do not add a
second minus sign: the motor reaction already supplies it. This sign argument
uses flat feet and equal support; it does not prove the complete multibody
closed-loop response. Foot rocking, load transfer, joint posture feedback and
contact loss must be considered before application.

If pitch is positive but pitch rate is negative, the robot is already returning.
The D contribution is then negative and can reduce or reverse the proposal to
oppose overshoot. The proposal need not always have the sign of pitch alone.

## 4. A worked calculation

Suppose the robot leans forward by 1 degree and is still rotating forward at
5 degrees/s:

| Quantity | Calculation | Result |
| --- | --- | --- |
| Pitch | 1 × pi/180 | 0.017453 rad |
| Pitch rate | 5 × pi/180 | 0.087266 rad/s |
| Total P contribution | 20 × 0.017453 | 0.349066 N m |
| Total D contribution | 4 × 0.087266 | 0.349066 N m |
| Raw contribution per ankle | (0.349066 + 0.349066)/2 | 0.349066 N m |
| Bounded proposal per ankle | clip to ±0.5 N m | 0.349066 N m |
| Applied balance contribution | observation-only mode | **0 N m** |

The ±0.5 N m bound limits the hypothetical extra contribution. The existing
±6/10/6 N m motor caps remain the total hip/knee/ankle command limits. We have
not implemented combining the candidate with those motor commands.

## 5. Separation of responsibilities and changed files

| File | Change and responsibility |
| --- | --- |
| `my_robot_description/balance.py` | New pure quaternion/gyro conversion and bounded proposal calculation; no ROS |
| `my_robot_description/balance_audit.py` | New recording audit: coverage, candidate formula and unchanged motor composition |
| `scripts/observe_balance.py` inside the ROS package | New IMU subscriber and JSONL recorder; creates no publishers or service clients |
| `config/balance_observer.yaml` | New candidate gains, target and per-ankle proposal bound |
| `launch/effort_standing.launch.py` | Optional observer when `balance_recording` is supplied; default off |
| Package `CMakeLists.txt` | Installs the new observer executable |
| Workspace `scripts/run_effort_push.py` | New `--balance-observer` option; saves and audits reference/push recordings |
| Workspace `scripts/plot_balance_observer.py` | New teaching plot separating measurements, proposals and actual commands |
| `tests/test_balance.py` | Signs, units/splitting, clipping, gyro frame conversion, bad inputs, coverage and non-application |

The first six paths are inside `src/my_robot_description/`. The README links to
this guide; `tests/test_damping.py` also checks the optional observer launch
argument and its omission when observation is disabled.

Before, the IMU was used by the posture node for its tilt shutdown and by the
experiment recorder for measurements. After this change, an additional node
subscribes to the same IMU topic and performs:

```python
pitch, rate = sagittal_imu(quaternion, body_gyro)  # abbreviated argument list
terms = balance_proposal(pitch, rate, settings)
record(terms, applied=False, applied_balance_nm=0.0)
```

The real `hold_effort_stance.py` motor calculation has **not changed**:

`motor_request = static_feedforward + joint_P + joint_D + bounded_joint_I`

The observer has no motor publisher or controller service client. A source test
checks that separation. The recorded motor requests are independently checked
against the four existing terms, so an extra balance torque would fail the audit.
That audit checks requested torque composition, not a calibrated motor measurement.

## 6. Reproduce the observation experiment

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash

python3 scripts/run_effort_push.py \
  --balance-observer \
  --damping-model explicit --controller-mode pid \
  --real-time-factor .25 --force-x 2 --gui \
  --output evidence/my-balance-observer-01

python3 scripts/plot_balance_observer.py evidence/my-balance-observer-01
```

Choose a new output name each time. The runner owns the simulation, first runs a
fresh zero-force reference, then a fresh push. It stops both simulations itself.
Use `--force-x -2` and another output name to observe the opposite direction.
Omit `--gui` to run headless. Quarter-speed wall-clock pacing gives the extra
observer more scheduling margin. The physics step is still 1 ms, and the push
and recovery durations are still simulated seconds. Each 15 s PID warmup takes
about 60 wall-clock seconds at this pacing. The reference has no visible push;
the second fresh simulation contains the push. Allow roughly 4–5 minutes for
the full reference-and-push experiment.

Each trial saves `balance.jsonl` beside `controller.jsonl`. The balance rows
contain IMU source time, receipt time, pitch, pitch rate, candidate settings,
P and D terms, raw and bounded per-ankle proposals, a six-joint proposal vector,
and explicit non-application fields. Summary JSON includes the observer audit.

The audit requires observer coverage within 0.05 s of the experiment boundaries,
no IMU source-timestamp gap greater than 0.05 s, a correct candidate formula and
unchanged motor-request composition. A failing observer stops this instrumented
experiment; observer failure is not counted as a valid trial.

## 7. How to read the four-panel plot

1. **Pitch:** forward or backward lean, compared with the no-push reference.
2. **Pitch rate:** whether the lean is increasing or decreasing.
3. **Candidate contributions:** P, D and their bounded per-ankle proposal.
4. **Actual versus hypothetical:** the existing ankle motor command, the
   unapplied candidate, and a zero line for the applied balance contribution.

A nonzero proposal in the reference can be valid: the actual settled pelvis can
have a small nonzero pitch even though the target is precisely world upright.

Do not interpret this plot as improved balance. The proposal is not applied, so
any recovery belongs to the existing joint PID controller. The next step will
be a separately enabled, bounded application with comparison experiments.


## Recorded forward-push result

The +2 N, 0.2 s observation run (`evidence/balance-observer/forward-02`) passed the existing recovery checks. The observer recorded 1,220 samples over 12.19 simulated seconds in both reference and push, with a maximum IMU timestamp gap of 0.01 s. The largest proposed correction was **0.104826 N m per ankle**, with no proposal clipping.

At 0.199 s after push publication, pitch was 0.0042997 rad and pitch rate was 0.0309145 rad/s. The calculation was:

`0.5 * (20 * 0.0042997 + 4 * 0.0309145) = 0.104826 N m per ankle`

The observer formula error and the motor-request composition error were both exactly zero in the recorded audit. **Applied balance torque remained zero.** The existing joint PID supplied the observed recovery.

![Forward balance observation](../evidence/balance-observer/forward-02/balance-observer.png)

The earlier `forward-01` run at real-time factor 0.5 stopped on the existing joint-feedback timestep guard during its no-push reference. It is retained as an aborted run and is not validation evidence. The accepted run uses factor 0.25; the guard was not relaxed.


## Final validation

| Push | Peak signed proposal per ankle | Motor-composition error | Observer / recovery |
| --- | ---: | ---: | --- |
| +2 N | +0.104826 N m | 0.0 N m | passed / passed |
| -2 N | -0.158925 N m | 0.0 N m | passed / passed |

Both tests used a 0.2 s pulse, a 10 s recovery window, the original 1 ms physics step and real-time factor 0.25. Fresh reference and push recordings all passed the observer audit. Both feet retained contact evidence, no existing motor command saturated, and no proposal reached its 0.5 N m bound.

The final audit checks both the unclamped motor request and the bounded motor command. All **108 tests** passed. The physical-model and joint-controller source hashes still match the earlier recovery validation. Results are linked in `evidence/balance-observer/validation.json`.

The sign reversal verifies the implemented candidate arithmetic against the observed forward/backward motion and the documented model convention. Because the correction was not applied, it does not experimentally prove that the candidate improves recovery.
