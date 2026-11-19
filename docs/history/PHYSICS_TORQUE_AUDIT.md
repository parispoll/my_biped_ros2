# Step 5f: follow the torque into the physics engine

**The missing torque is now accounted for.** The controller's command reaches the
Gazebo input and DART force calculation. DART's implicit passive damping and joint
constraint terms then explain the difference between that input and the transmitted
ankle moment.

This is a diagnosis, not a recovery-controller change. Gains, target angles, masses,
friction, passive damping and torque caps retain their normal settings. The robot
still fails the previously declared return-to-posture criteria after the 1 N push.

## 1. What was missing from our earlier calculation?

We estimated passive damping as:

```
passive damping torque = -2.5 × final measured ankle speed ≈ 0
```

That estimate did not reproduce DART's actual split-step calculation. DART computes
an unconstrained motion prediction, uses implicit damping in that calculation, and
then applies contact/joint constraint corrections. The intermediate predicted
velocity need not be zero when the final measured velocity is nearly zero.

The native audit observes the original forward-dynamics function, calls it without
changing its inputs, and reads its results. This matches the calculation order in
[DART 6.13.2 World::step](https://github.com/dartsim/dart/blob/v6.13.2/dart/simulation/World.cpp).
The implicit damping expression is documented in
[DART GenericJoint](https://github.com/dartsim/dart/blob/v6.13.2/dart/dynamics/detail/GenericJoint.hpp).

## 2. A concrete ankle example from the audited push

The successful native audit is saved in
`evidence/physics-audit/dart-split-step-1N`. In its final stationary interval:

| Quantity, left ankle | Value |
| --- | ---: |
| ROS command | +0.628640208 N m |
| Gazebo physics input | +0.628640208 N m |
| DART command before constraints | +0.628640208 N m |
| DART force before constraints | +0.628640208 N m |
| Implicit passive damping contribution | -0.516764816 N m |
| Joint constraint impulse / timestep | +0.108123497 N m |
| Transmitted joint moment | +0.219998889 N m |
| Gravity estimate from Gazebo COM positions | +0.219962658 N m |

The balance is:

```
transmitted moment = motor input + implicit damping + joint constraint contribution
                   = +0.628640208 - 0.516764816 + 0.108123497
                   = +0.219998889 N m
```

The measured internal terms close this equation to floating-point precision in
the final window. The independent gravity estimate differs from the transmitted
moment by approximately 0.000036 N m. This small difference is retained in the
report rather than labelled exact equality.

The audited push's command and lean differ slightly from your runs because the
transport command window varies by a few milliseconds. The same accounting issue
appears in the baseline and all six joints; we are not claiming identical sampled
push histories across runs.

## 3. How can damping be nonzero if the joint is still?

For the audited ankle, DART's internal forward-dynamics acceleration is about
206.706 rad/s² before constraint corrections. With a 0.001 s timestep and nearly
zero starting velocity:

```
predicted velocity = starting velocity + timestep × free acceleration
                   ≈ 0 + 0.001 × 206.706
                   ≈ +0.206706 rad/s

implicit damping = -passive damping coefficient × predicted velocity
                 ≈ -2.5 × 0.206706
                 ≈ -0.516765 N m
```

Contact and joint constraints then correct the velocity; the final measured speed
is about 4e-11 rad/s. The predicted speed is an intermediate solver variable. It
does not mean the visible robot actually moves at that speed between recorded
frames, and it is not the velocity used by our Python derivative feedback.

Keep these three effects separate:

- The Python controller's Kd term uses measured speed and is effectively zero here.
- DART's implicit **passive model damping** uses its internal prediction and is
  nonzero in this numerical step.
- Joint constraints contribute an impulse; dividing by the 0.001 s timestep gives
  the signed torque-equivalent contribution used in the balance.

The joint constraint contribution is within the configured ±0.2 N m Coulomb bound.
The earlier approximately 0.41 N m unexplained difference was not a measurement
of friction above that bound: it combined terms we had not separately observed.

![Split-step torque balance](../evidence/physics-audit/dart-split-step-1N/torque-balance.png)

## 4. What we checked along the command path

For both a fresh no-push reference and a fresh 1 N push:

1. **Command delivery:** final ROS, Gazebo input, DART command and DART force values
   agree within 1e-5 N m; observed differences are much smaller. The Gazebo observer
   also records min/mean/max over every ten physics steps, so a pulsed input cannot
   pass merely because its average looks right.
2. **Actuator mode:** all six joints report DART FORCE mode, not a velocity servo.
3. **Travel limits:** the ankle is roughly 0.614 rad from its nearer travel stop;
   no final joint is close enough to a stop to explain the static offset.
4. **Constraint magnitude:** the recorded generalized joint constraint impulses,
   divided by timestep, remain within the nominal Coulomb bound in the final window.
   The final speeds are also far from the velocity limits.
5. **External push:** the recorded external force input is zero throughout the
   final window. Its observed physics-input impulse is 0.202 N s during the push.
   This is a simulation input audit, not independently calibrated hardware force.
6. **Gravity:** masses and centre-of-mass positions are read from Gazebo's actual
   entities. The upper body above the ankle cut has mass 12.3 kg. The resulting
   gravity moment agrees closely with the transmitted ankle moment.
7. **Torque accounting:** motor input + implicit damping + generalized constraint
   impulse/timestep reproduces the transmitted moment for all six joints.

The old unclaimed-joint hold leaves a zero-valued velocity-command component in
the entity manager. Its presence alone is not evidence of an active servo. DART
reports FORCE mode, and Gazebo gives the force component priority when both exist;
see [Gazebo Physics](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/physics/Physics.cc).

## 5. Why an engine force buffer can read zero afterward

The native log also records `command_nm` and `force_nm` at the transmitted-wrench
getter. They read zero after the step. That is not a dropped input: DART's world
step clears internal force/command buffers after integration. The `fd_command_nm`
and `fd_force_nm` columns observe them during forward dynamics, before clearing.
This is why the point in the update cycle matters when inspecting simulator data.

The transmitted wrench itself is a resulting force/moment pair at the joint
origin, expressed in the joint frame. It includes the dynamics/constraint result;
it is not simply the commanded motor torque. Gazebo's DART adapter obtains it from
the child's body force and transforms it into the joint frame in
[JointFeatures](https://github.com/gazebosim/gz-physics/blob/gz-physics7/dartsim/src/JointFeatures.cc).
For this URDF, each child origin coincides with its joint origin. The observer
rotates the moment to world coordinates when comparing it with gravity.

## 6. What was added

| File | Responsibility |
| --- | --- |
| Package `src/physics_probe.cpp` | Optional Gazebo system observer: reads commands after command-producing PreUpdate systems, then reads resulting joint wrenches/COMs after physics. No entity components are written. |
| Package `src/dart_probe.cpp` | Optional DART 6.13 native observer: calls the original forward-dynamics/getter functions and reads their intermediate and final values. No forces or state are set by the observer. |
| Package `my_robot_description/physics_audit.py` | Checks matching stationary windows, command agreement, force mode, constraint bounds and split-step torque balance. |
| Workspace `scripts/analyze_physics_probe.py` | Reads the saved CSV/controller files and writes `physics-audit.json`. |
| Workspace `scripts/plot_physics_audit.py` | Generates the explanatory diagram and torque waterfall. |
| `tests/test_physics_audit.py` | Checks accounting, bad timesteps, dropped/pulsed commands, wrong actuator modes, constraint bounds, mismatched windows and proximity to travel stops. |

`CMakeLists.txt` now builds two optional-use shared libraries; `package.xml` declares
the installed Gazebo/DART vendor dependencies. The launch file can add the observer
to a temporary copy of the world, and can preload the native observer into its
Gazebo process. The runner exposes explicit flags for these options. The ordinary
launch defaults load neither observer.

The native observer is deliberately version-specific. It is compiled against the
installed vendor DART 6.13.2, with a compile guard for the 6.13 API, and resolves the
original functions before calling them. It does not patch or replace files in
`/opt/ros` and does not change the user's shell environment. Its `LD_PRELOAD` is
local to that optional Gazebo process; a temporary whitespace-free symlink handles
our workspace path containing a space. Review this observer before using another
DART version.

## 7. Run the audit yourself

First read the existing result without starting a simulation:

```bash
cd '/home/parispoll/Documents/ChatGPT/robotics carrer'
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 scripts/analyze_physics_probe.py evidence/physics-audit/dart-split-step-1N
python3 scripts/plot_physics_audit.py evidence/physics-audit/dart-split-step-1N
```

To collect a fresh instrumented pair:

```bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash

python3 scripts/run_effort_push.py \
  --physics-probe \
  --dart-probe \
  --real-time-factor 0.5 \
  --output evidence/my-physics-audit

python3 scripts/analyze_physics_probe.py evidence/my-physics-audit
python3 scripts/plot_physics_audit.py evidence/my-physics-audit
```

The native logging adds overhead. An initial full-speed attempt stopped on stale
measurements; that evidence was kept in `evidence/physics-audit/dart-observed-1N`.
The successful run uses half wall-clock speed to reduce scheduling pressure.
This preserves the **1 ms simulation timestep**, confirmed directly in DART, and
the same controller settings. A 10 s simulated recovery interval consequently
requires about 20 wall seconds. Freshness thresholds were not relaxed.

Each trial writes `physics.csv`, `dart.csv`, `controller.jsonl`, normal experiment
recordings and logs. The analysis selects a complete stationary window and verifies
that the separate streams describe the same joint positions. It does not combine
moving, unsynchronized snapshots into a purported exact dynamic force balance.

`summary.json` still says the **push recovery failed**. `physics-audit.json` says
the **force-path/accounting checks passed**. These are different questions.
All 75 Python tests pass, and both native libraries build successfully.

## 8. What this establishes, and what comes next

We have accounted for the previously unexplained torque in this simulator's
algorithm. The constant joint-angle error produces a constant PD correction,
which can coexist with gravity, implicit damping and constraint reactions at
nearly zero final speed. The simulator has not demonstrated return to the initial
pose, and this accounting does not establish equivalent behavior on real hardware.

A useful next controlled test is timestep sensitivity with the same gains and
physical coefficients. That will quantify how much of the remaining offset depends
on the numerical method before we choose changes to the physical model or feedback.
The diagnosis does not by itself justify increasing gains or adding an integrator.

That first follow-up is now recorded in [the timestep comparison](TIMESTEP_COMPARISON.md).
It compares two resolutions; the normal 1 ms default remains in place.
