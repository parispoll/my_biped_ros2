# Step 5b: understand the standing torque before commanding it

For annotated robot sketches and worked calculations for all three joints, open
the [four-page visual guide](../output/pdf/standing-torque-visual-guide.pdf).
Regenerate it from the current model with `python3 scripts/make_statics_guide.py`
after sourcing ROS and this workspace. The script uses ReportLab from the desktop's
bundled PDF runtime when it is not installed in system Python.

This step only calculates. It does not start Gazebo or send commands. The working
position-controlled humanoid and the separate torque bench retain their behavior.

## 1. Three quantities to keep separate

- Angle, in radians: where a joint is. Our knee target is 0.4 rad, about 23 degrees.
- Force, in newtons (N): a push or a weight. Our robot weighs 138.321 N.
- Torque, in newton-metres (N·m): a turning effect about a joint.

A 10 N force acting perpendicular to a 0.1 m lever produces 1 N·m.
Our earlier 40 N push was an external force; it was not a 40 N·m joint command.

## 2. Why the pendulum calculation is insufficient

The bench rod hangs from a fixed joint. The humanoid stands on two feet and has
a floating pelvis. Each leg receives an upward force from the floor and downward
forces from its own weight. Ignoring the floor would calculate a hanging leg's
torque, not a standing robot's torque.

For this first estimate assume: both feet are flat, the pelvis is upright, the
robot is motionless, each foot supports half the total weight, there are no
horizontal ground forces, and ground pressure acts at each foot's centre.
Joint friction is excluded. These assumptions describe one ideal equilibrium;
they do not describe every possible distribution of contact forces.

## 3. Calculate the floor force

The URDF masses give:

```
total mass = pelvis + 2 × (thigh + shin + foot)
           = 7.5 + 2 × (1.4 + 1.0 + 0.9)
           = 14.1 kg
weight     = 14.1 × 9.81 = 138.321 N
each foot's upward force = 138.321 / 2 = 69.1605 N
```

The calculator reads these masses from the robot description through the existing
validated standing model. They are not separately hardcoded in the new calculation.

## 4. Balance the knee's turning effects

Imagine cutting the leg at the knee and examining only the shin and foot.
For our 0.4 rad crouch, horizontal coordinates relative to the pelvis are:

```
knee:        +0.057774 m
foot centre: +0.012271 m
```

The floor force acts 0.045504 m behind the knee. Its turning effect about the
knee is +3.147060 N·m. The shin and foot weights together produce -0.624948 N·m.
Positive/negative follow the model's +Y joint axes, using the right-hand rule.

```
motor torque + floor moment + weight moment = 0
motor torque = -(3.147060 - 0.624948)
             = -2.522112 N·m
```

The negative sign means a knee-extension torque: it opposes further bending.
Both knees need the same torque because our pose and load assumptions are symmetric.

## 5. Read the result carefully

| Joint | Left holding torque | Right holding torque |
| --- | ---: | ---: |
| Hip | approximately 0 N·m | approximately 0 N·m |
| Knee | -2.522112 N·m | -2.522112 N·m |
| Ankle | approximately 0 N·m | approximately 0 N·m |

Hip moments cancel for this specially balanced pose. Ankle moments vanish because
the assumed pressure centre and foot mass centre lie directly below the ankle.
Tiny printed values such as `-4.44e-16` are floating-point rounding near zero.

Zero equilibrium torque does not mean a joint is unnecessary or stable when
uncontrolled. A small lean, shifted foot pressure, unequal support or acceleration
changes the required torques. Static equilibrium alone does not establish stability.
These values do not select a real motor: movement, disturbances, transmission,
thermal limits and safety margins still require separate analysis.

## 6. Run it yourself

From the workspace terminal:

```bash
cd '/home/parispoll/Documents/ChatGPT/robotics carrer'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/plan_standing_torques.py --knee 0.4
```

No simulation needs to be open. To change the posture being calculated:

```bash
python3 scripts/plan_standing_torques.py --knee 0.6
```

Each knee now needs approximately -3.750192 N·m. The deeper crouch changes the
lever arms, so more holding torque is needed. `--knee 0` gives zero static motor
torque under the ideal straight-leg assumptions; that does not prove stability.
Invalid poses are rejected by the existing model rather than silently accepted.

## 7. Walk through the new code

Open `src/my_robot_description/my_robot_description/statics.py` first.

1. `standing_loads(model, knee, gravity=9.81)` takes the existing robot model,
   your chosen knee bend and gravitational acceleration.
2. The finite/positive check rejects invalid gravity values.
3. `model.pose(knee)` finds matching hip and ankle angles, keeping the feet flat
   and the total centre of mass above the foot centres.
4. `model.geometry(...)` gives the horizontal foot position.
5. `knee_x = -model.thigh * math.sin(hip)` projects the thigh length horizontally.
6. `ankle_x` is also the foot centre's horizontal coordinate for our flat feet.
7. `bodies` stores each leg body's mass and horizontal mass-centre position.
   The thigh centre is halfway to the knee; the shin centre is halfway between
   knee and ankle; the foot centre is directly below the ankle.
8. `ground_force = model.mass * gravity / 2` calculates the upward force per foot.
9. The loop considers hip, knee and ankle separately. `bodies[index:]` selects
   only the bodies below that joint: all three at the hip, shin/foot at the knee,
   and only the foot at the ankle.
10. `ground_moment` calculates the floor force's turning effect about that joint.
    For a vertical force, the +Y moment is `-horizontal_distance * vertical_force`.
11. `weight_moment` adds the downward weights' turning effects. Downward force
    reverses the sign, giving `horizontal_distance * mass * gravity`.
12. `motor_torque` is the opposite of their sum, so the total moment becomes zero.
13. The returned dictionary includes the intermediate moments, both legs' joint
    torques and the assumptions, so you can inspect how the answer was obtained.

Next open `scripts/plan_standing_torques.py`. It handles the terminal argument,
expands the robot's Xacro into URDF, loads the validated model, calls
`standing_loads`, and prints its result as JSON. Keeping this separate means the
physics calculation has no dependency on a running ROS node or Gazebo process.

## 8. What changed, and what was checked

Added `statics.py`, the terminal script, `tests/test_statics.py` and this guide.
Added a link in README. There was no old torque-calculation code to repair here;
this is a new calculation built on the existing stance model.

Four new tests check straight-pose loads, crouch signs and symmetry, invalid
inputs, and an independent energy calculation. In that last test, the change
in gravitational potential energy along the balanced crouch path must equal
the work done by both legs' motor torques. It agrees at three different bends.
All 50 project tests passed after rebuilding and sourcing the workspace.

The next implementation step is a separate effort-standing mode using these
holding torques plus joint-angle/velocity feedback. That controller has not been
added in this step; joint posture control will also need to be distinguished
from body-balance feedback.
