# Educational biped robot lab

Open this repository folder in VS Code. Historical experiment evidence and local runtime dependencies are not included in this repository snapshot; some older guides refer to them.

The current model has ten leg joints, finite P54-profile motor limits, ROS 2 Jazzy and Gazebo Harmonic. A recorded **ten-placement gait with 75 mm steps and 2× movement speed passed all six audits on 14 November 2026**, progressing 712.8 mm. It remains slow, finite quasi-static simulation walking with a stable stop. Continuous dynamic walking, turning and disturbance recovery during walking are future work.

## Interactive simulation

To launch once and command the robot from a terminal, follow [Launch once, command the robot](docs/guides/INTERACTIVE_SIMULATION.md). It supports `step`, `walk --steps N`, controlled `stop`, `stand` (hold), and `status`, with ten placements available per fresh session. The existing panel remains the measured experiment interface.

## Start reading here

1. [Project map and code flow](docs/learning/01-project-map.md).
2. [How one walking step is calculated](docs/learning/02-walking-code.md).
3. [How to find a fault](docs/learning/03-debugging.md).
4. [Walking experiment, mathematics and measured results](docs/guides/ALTERNATING_WALK.md).
5. [Longer and faster walking profiles](docs/guides/LONGER_FASTER_WALK.md).
6. [What moved and what was removed](docs/REORGANIZATION.md).

## Folder responsibilities

```text
src/my_robot_description/
  my_robot_description/   Reusable Python code, grouped by responsibility
    model/                Geometry, masses, joint order, motor limits, kinematics
    motion/               Lowering, transfer, lift, step and walking plans
    control/              PID, integral limits, timing and feedback safety
    sensing/              Contact/force/IMU/COM interpretation
    validation/           Independent audits of saved measurements
    visualization/        COM and force markers
    runtime/              Launch preparation and startup dependencies
    experiments/          Force-pulse helpers
    studies/              Earlier diagnostic and comparison mathematics
  nodes/                  Executable ROS adapters: control, sensing, legacy
  plugins/                Gazebo C++ probes and passive damping
  urdf/ config/ launch/ worlds/  Robot, parameters and simulation setup
scripts/                  CLI tools: experiments, planning, checks, plots, analysis
tools/force_panel/        Local experiment GUI
tests/                    Regression and behavior checks
docs/learning/            Current educational guide
docs/guides/              Current model and experiment references
docs/history/             Earlier research and frozen handoffs
evidence/                 Original measurements, summaries and plots
```

Each source/tool folder has its own `README.md` listing its files and responsibilities. Old flat Python imports were replaced with explicit responsibility paths; there are no duplicate compatibility modules.

## Build and open the GUI

```bash
cd /path/to/biped-robot
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
bash scripts/start_force_panel_background.sh
```

Open [the local panel](http://127.0.0.1:8765/). The stable service start scripts remain in `scripts/`, so the installed login service keeps its entry point. Choose **Three alternating steps measured**, initial support **Left**, enable Gazebo, then run. The panel retains the prerequisite checks and historical results.

CLI equivalent (use a new output directory):

```bash
python3 -m scripts.experiments.run_lateral_shift \
  --actuator-profile robotis_p54 --alternating-walk \
  --real-time-factor 1.0 --direction left --gui \
  --output evidence/my-next-walking-run
```

For an offline preview, use `python3 -m scripts.planning.preview_alternating_walk --support left --output evidence/my-next-preview`. A preview never starts Gazebo or sends motor commands.

## Check a code change

After sourcing ROS and the built workspace:

```bash
ROS_LOG_DIR=/tmp/biped-test-logs python3 -m unittest discover -s tests
```

[Current guide index](docs/README.md) · [Experiment roadmap](docs/guides/GUI_ROADMAP.md) · [Folder migration and verification](docs/REORGANIZATION.md)

The organization change does not retune gains, alter geometry, change motor limits or modify historical evidence. Historical source hashes refer to the old layout; the exact old source is retained in `evidence/preparation/code-organization/before-source.tar.gz`.

## RoboKit offline integration

The separate [RoboKit lab](tools/robokit_lab/README.md) checks both robot models and
reproduces the biped's 75 mm first-step geometry on the CPU. It includes named-joint
mapping, COM and Jacobian agreement checks; it sends no commands.

MuJoCo development: [standalone simulator and migration guide](docs/guides/MUJOCO_SIMULATION.md).
