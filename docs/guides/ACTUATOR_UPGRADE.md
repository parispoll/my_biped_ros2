# What our old motors assumed, and the P54-class upgrade

> Later development: the physical-pulse milestone adds P54-only roll velocity damping and a level front camera. See [the current physical-pulse guide](../history/P54_LATERAL_PUSH_PANEL.md). Results below retain their original controller version.

The old robot did not contain a manufacturer-specific motor model. It used ideal joint torque commands with provisional software limits. The new selectable `robotis_p54` profile adds documented continuous-output budgets, output-speed guards and actuator mass/inertia estimates. It remains an educational simulation abstraction, not a calibrated motor or mechanically complete THORMANG3 replica.

## 1. What we originally assumed

| Item | Original assumption |
|---|---|
| Hip roll / hip pitch | Maximum requested motor torque ±6 Nm |
| Knee pitch | Maximum requested motor torque ±10 Nm |
| Ankle pitch / ankle roll | Maximum requested motor torque ±6 Nm |
| Control | Joint PID plus model-based static feedforward, nominal 250 Hz |
| Actuation | Ideal Gazebo joint effort input after software clipping |
| Carrier masses | 80 g each, provisional structural bodies |
| Other body masses | Generic model assumptions, not an actuator bill of materials |
| Passive damping | Separate explicit viscous law; not a measured gearbox model |
| Electrical/thermal behavior | Not modeled |
| Manufacturer basis for 6/10 Nm | None; provisional standing budgets |

The old pitch-joint URDF declarations were larger than these requested motor budgets. The ros2_control effort interfaces and posture code applied the smaller 6/10 Nm limits. Thus reading an 80 Nm URDF pitch limit did not mean the controller was commanding an 80 Nm motor.

These choices were enough to study bounded effort standing and small transfers. They did not establish a mechanically sized walking robot. Our recent torque calculations exposed that limitation; increasing PID gains would not increase the available torque.

## 2. What other humanoids use

### ROBOTIS OP3: small servo humanoid

ROBOTIS lists OP3 at about 3.5 kg and 510 mm high, using XM430-W350-R actuators. Its actuator comparison lists 4.1 Nm **stall torque** and 46 rpm no-load speed. Our original model is similar in height but much heavier at 14.42 kg, so copying OP3's torque number directly would be inappropriate. Stall torque is not a continuous holding budget. [Official OP3 specifications](https://emanual.robotis.com/docs/en/platform/op3/introduction/)

### ROBOTIS THORMANG3: larger geared actuator family

ROBOTIS lists THORMANG3 at 42 kg and 1.375 m, with PH54-200, PH54-100 and PH42-020 actuators. We use the P54 family as a documented reference, not the exact joint assignment or mechanical design of that robot. [Official THORMANG3 specifications](https://emanual.robotis.com/docs/en/platform/thormang3/introduction/)

The manufacturer lists these continuous characteristics at 24 V:

| Actuator | Continuous torque | Continuous speed | No-load speed | Mass | Case dimensions |
|---|---:|---:|---:|---:|---|
| PH54-100-S500-R | 25.3 Nm | 29.2 rpm | 33.3 rpm | 0.740 kg | 54 × 108 × 54 mm |
| PH54-200-S500-R | 44.7 Nm | 29.0 rpm | 33.1 rpm | 0.855 kg | 54 × 126 × 54 mm |

Sources: [PH54-100 official manual](https://emanual.robotis.com/docs/en/dxl/p/ph54-100-s500-r/), [PH54-200 official manual](https://emanual.robotis.com/docs/en/dxl/p/ph54-200-s500-r/).

Continuous and no-load speeds describe different operating conditions. We do not combine a stall torque with a no-load speed and pretend both are simultaneously available. These published characteristics also do not replace thermal testing of an actual installation.

## 3. Our selected assignment

| Joint, on each leg | Legacy cap | New continuous-output cap | Reference model |
|---|---:|---:|---|
| Hip roll | 6 Nm | 44.7 Nm | PH54-200 |
| Hip pitch | 6 Nm | 25.3 Nm | PH54-100 |
| Knee pitch | 10 Nm | 44.7 Nm | PH54-200 |
| Ankle pitch | 6 Nm | 25.3 Nm | PH54-100 |
| Ankle roll | 6 Nm | 25.3 Nm | PH54-100 |

This is our simulation design choice: stronger units at hip roll and knee, where our static analysis identified high demand. It is not a claim that THORMANG3 uses this exact assignment. All ten units use the documented 24 V characteristics; no stall/peak torque boost is enabled.

The new operating-speed guard is **2.4 rad/s** (about 22.9 rpm), below both published continuous speeds. At rest, a standing motor's mechanical power `P = torque × speed` is zero even when it produces substantial torque; electrical heating can still occur and is not simulated.

The existing 80% static planning threshold remains separate from the motor cap. For example, the planning allowance is `0.8 × 44.7 = 35.76 Nm` for a PH54-200-class joint and `0.8 × 25.3 = 20.24 Nm` for a PH54-100-class joint.

## 4. Stronger motors also weigh more

We treat the original generic masses as the structural baseline, then add all ten actuator masses. This is an explicit conservative modeling choice because the old masses were not a detailed parts inventory. It must be reconciled against a real mechanical bill of materials before hardware construction.

Per leg: `0.855 + 0.740 + 0.855 + 0.740 + 0.740 = 3.930 kg`.

Whole robot: `14.42 + 2 × 3.930 = 22.28 kg`.

Therefore the new weight is `22.28 × 9.81 = 218.5668 N`, with roughly 109.2834 N per foot in centered standing.

Adding mass without moving its COM or changing inertia would be inconsistent. For this first abstraction, each actuator case is centered at its output joint, attached to the corresponding child link. Its mass and uniform-box inertia are merged with that link's original inertia using the parallel-axis theorem:

`combined_COM = (m_link COM_link + m_motor COM_motor)/(m_link + m_motor)`

`combined_I = I_link_at_its_COM + m_link distance_to_combined_COM² + I_motor_at_its_COM + m_motor distance_to_combined_COM²`.

The implementation applies this separately to each principal axis. It keeps eleven links and ten revolute joints, so the existing geometry and observers continue to work.

Actual actuator casing orientation, shaft position, rotor inertia, cable/bracket mass and mechanical mounting are not established by this approximation. We do not add casing collision meshes or claim that all ten cases fit the co-located joint layout. A real mechanical design still needs that packaging work.

## 5. What changed in code

| File | Change |
|---|---|
| `config/roll_actuators.yaml` | Single named data source for caps, speed guards, actuator masses, dimensions and specification links |
| `my_robot_description/roll_actuators.py` | Loads/validates profile name before robot modules initialize |
| `my_robot_description/roll_stance.py` | Uses the selected caps; recomputes standing geometry/feedforward with the actual model mass |
| `urdf/components/roll_actuators.xacro` | Composite link + actuator inertial macro |
| `urdf/components/leg_roll.xacro` | Adds selected actuator mass/inertia and matching new-profile joint declarations |
| `urdf/roll_effort.urdf.xacro` | ros2_control command bounds use the same profile data |
| `scripts/hold_roll_stance.py` | Records actuator metadata and enforces the P54 speed guard |
| Workspace `scripts/experiments/run_roll_standing.py`, `scripts/experiments/run_lateral_shift.py` | Accept `--actuator-profile legacy` or `--actuator-profile robotis_p54` before imports/process launch |
| `tests/test_roll_actuators.py` | Checks mass addition, parallel-axis inertia, continuous caps, operating speed and geometry/control consistency in independent processes |

Package-relative paths in the table are under `src/my_robot_description/`. Controller gains and passive damping are unchanged for a fair initial comparison.

## 6. What is still idealized

Gazebo still receives ideal joint effort commands. The new profile does not reproduce the actuator's current regulator, electrical supply, communication delay, gearbox friction/backlash, reflected rotor inertia, thermal limits or full torque–speed envelope. The speed guard stops a trial when measured speed exceeds the selected envelope; it is not a hardware velocity servo or a guarantee that an external push cannot drive a joint faster.

The cap and speed guard are a documented, simplified operating envelope. They do not certify continuous performance for every temperature, duty cycle or mounting arrangement. In particular, commanding a low-speed holding torque is not evidence that a real motor could remain cool indefinitely.

## 7. How to use the new motors

The `legacy` profile remains the default when no profile is selected, preserving earlier educational baselines. Select the upgrade explicitly:

```bash
cd '/home/parispoll/Documents/ChatGPT/robotics carrer'
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash

python3 scripts/experiments/run_roll_standing.py --gui --actuator-profile robotis_p54 \
  --output evidence/my-p54-standing-01
```

Use a fresh output folder. The runner propagates one profile to the model, posture process and audits; `joints.actuators.json` records it.

A later small bilateral shift uses the same explicit option:

```bash
python3 scripts/experiments/run_lateral_shift.py --gui --actuator-profile robotis_p54 \
  --foot-loads --displacement 0.01 --output evidence/my-p54-shift-01
```

For direct ROS launch or offline analysis, choose the environment before starting Python/ROS:

```bash
BIPED_ACTUATOR_PROFILE=robotis_p54 ros2 launch my_robot_description roll_standing.launch.py gui:=true
BIPED_ACTUATOR_PROFILE=robotis_p54 python3 scripts/studies/search_support_postures.py \
  --output evidence/my-p54-static-search-01
```

Do not change the profile inside an already-running process; imported caps must agree with its model. This upgrade applies to the separate ten-joint robot. The older six-joint push experiments retain their original model.

## 8. Verified results

The package build and all **271 unit tests passed**, including legacy regressions and independent new-profile model/control checks.

The GUI run `evidence/p54-standing-01` passed 30 simulated seconds of standing: pose, both contacts, IMU, independent COM mass, command composition and physical damping audits all passed. The recorded mass was 22.28 kg and there were no saturated standing samples.

Largest settled joint error: **9.8545576e-05 rad**. Largest active recorded motor speed, including settling: **0.122181 rad/s**, below the new 2.4 rad/s guard.

The heavier-model static search `evidence/p54-static-search-01` found a sampled zero-load endpoint at 55 mm shift and 60 mm lowering, with peak static utilization **46.35%**. Its loaded hip-roll demand was 20.72 Nm, knee 19.57 Nm and ankle-roll 10.84 Nm. These are new-mass predictions, not the old lighter robot torque figures divided by larger caps. No zero-load motion or foot lift was tested.

The evidence folder contains `actuator-comparison.png`, `verification.json`, the build/test logs and raw standing records. The guide and comparison keep measured standing separate from predicted single-support feasibility.
