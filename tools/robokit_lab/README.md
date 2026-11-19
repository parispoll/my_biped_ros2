# Offline RoboKit model adapter

This lab adds RoboKit 1.0.0 as a separate CPU tool for both the ten-joint biped
and fourteen-joint Aria **practice** model. It does not publish ROS commands,
start physics, modify robot parameters, or change stance/walking acceptance.

## Run

From the workspace root:

```bash
bash tools/robokit_lab/run.sh --output evidence/robokit/my-new-check
```

Use a fresh output directory. The launcher sources ROS Jazzy for Xacro and uses
`.venv-robokit/bin/python`. Dependencies are isolated from the system environment.
The environment currently installed in this workspace is ready to use.

To recreate it after removal (downloads required):

```bash
python3 -m venv --system-site-packages .venv-robokit
.venv-robokit/bin/python -m pip install -r tools/robokit_lab/requirements.txt -c tools/robokit_lab/constraints-tested.txt
```

The venv uses system NumPy/SciPy and ROS's Xacro after sourcing ROS. No PyTorch,
CUDA device, MuJoCo, Newton, cloud model, or external robot asset is required.
`constraints-tested.txt` records the locally installed package versions; the
system dependencies used for validation were Python 3.12, NumPy 1.26.4, SciPy 1.11.4.

## What is checked

- Exact named-joint mapping, including RoboKit's different biped joint ordering.
- Finite scalar joint positions, URDF limits, and proper rigid base transforms.
- Every link transform against the existing independent double-precision URDF FK.
- Foot/contact origins, inertial-offset-aware COM, and backend total mass.
- RoboKit spatial Jacobians converted to world point Jacobians, compared with
  independent ancestor/axis cross-product derivatives.
- A translated and 90-degree-rotated floating base.
- Aria nominal and perturbed poses, initialized explicitly because zero knees
  are outside its valid range. These checks use the generated URDF; they do not
  independently certify mechanical JSON/CAD consistency.

The biped trajectory comes from the existing `FirstStepProgram(advance=0.075)`.
RoboKit evaluates its joint/base trajectory at 108 samples, covering preparation,
weight transfer, lift, forward advance, landing, reload and final hold.
This is **forward-kinematic reproduction**, not a new IK or optimized solution.
It uses the original 241-second first-step clock, not the faster ten-placement
profile's clock. No new dynamic feasibility or successful physics run is claimed.

## Output

- `summary.json`: pass/fail, maximum errors, tolerances, joint mapping, model and
  source hashes, unchanged-source check, swing travel and support-foot drift.
- `trajectory.json`: named joint positions, base matrices, world foot positions
  and COM, with explicit units.
- `biped.urdf`, `aria.urdf`: model snapshots.
- `step-reproduction.png`: offline foot travel and lift plot.

The CLI exits nonzero when a tolerance fails. It never overwrites evidence.
Position/COM tolerance is 2 micrometres; point-Jacobian tolerance is 3e-6 m/rad.
These tolerate float32 arithmetic and are not physical sensor tolerances.
The plot's height refers to the foot link origin relative to its starting height.

## Interface

`RobotAdapter(urdf_path, joint_names)` accepts a complete name→radian dictionary
and optional world-from-base 4×4 matrix. It returns transforms, world COM and the
RoboKit state. It currently supports the fixed/revolute models used here.
Backend joint arrays must never be sent directly to a controller: use names.

## Tests

```bash
XDG_CACHE_HOME=/tmp/robokit-cache .venv-robokit/bin/python -m unittest tools.robokit_lab.test_adapter
```

Tests cover reordered joints, rigid-base convention, invalid/missing/extra state,
limits, duplicate configuration, and finite-difference verification of the
independent reference Jacobian. The full CLI supplies real-model agreement checks.

## Integration progression

Stage 2 below adds multi-foot IK using the validated mapping. Subsequent candidate
optimization must retain the static checks and Gazebo audits. Aria's M4 stance
gate remains open.

Upstream references: [robot model](https://github.com/openrobokit/robokit/blob/main/docs/concepts/robot.md),
[IK](https://github.com/openrobokit/robokit/blob/main/docs/guide/ik.md).

## Stage 2: actual inverse kinematics

```bash
bash tools/robokit_lab/run_ik.sh --output evidence/robokit/my-new-ik-check
```

This uses the saved `validation-002` model and 108-sample baseline by default.
Use `--baseline PATH` to select another model-agreement evidence directory.
It needs no additional downloads or ROS launch.

`FootIK` solves both foot poses together using RoboKit's position, rotation and
joint-limit terms with 60 single-candidate Levenberg–Marquardt iterations per pose.
The pelvis follows the baseline. Targets are expressed in the root frame, so
there is no free-base optimization in this stage. The first seed has 0.02 rad
added to every baseline joint; later seeds are the preceding IK result.
The desired joint solution is not supplied at every sample.

Because optimizer costs are soft, every result is checked against hard joint
limits and independent double-precision FK. Acceptance requires position error
at most 10 µm and orientation error at most 0.0001 rad. Unreachable goals and
malformed target transforms are rejected by tests.

Every solved sample receives the existing conservative collision-envelope check.
From t=117 s onward, static loading follows the existing first-step force-share
schedule, common CoP offset and motor reserve calculation, retaining the 10 mm
sole margin and 80% utilization cap. Earlier preparation phases are checked for
geometry/collision only. Speeds are sampled finite differences on the original
241-second clock, limited to 0.05 rad/s; this does not bound between-sample peaks.

Final evidence is `evidence/robokit/ik-002/`: `summary.json`, named-joint
`trajectory.json`, model snapshot and `ik-comparison.png`. `ik-001` is retained as
an earlier development run. Stage 2 passed with maximum position error about
2.13 µm, peak static utilization 53.06%, and sampled peak speed 0.03694 rad/s.
The independent solution agrees with the baseline within 0.0000193 rad per joint.
It demonstrates that RoboKit can reproduce this motion, not that it improves it.

Run both test groups with:

```bash
XDG_CACHE_HOME=/tmp/robokit-cache .venv-robokit/bin/python -m unittest tools.robokit_lab.test_adapter tools.robokit_lab.test_ik
```

The next motion-development milestone is to vary prescribed pelvis/foot targets,
rank candidates using explicit COM/static/collision constraints, and retain the
existing Gazebo audits for physics acceptance. Aria stance remains a separate gate.

## Stage 3: bounded pelvis candidate search

```bash
bash tools/robokit_lab/run_search.sh --output evidence/robokit/my-new-search
```

The search evaluates 25 combinations of fore–aft and vertical pelvis offsets,
each in {-10, -5, 0, 5, 10} mm. World foot poses stay fixed to the saved 75 mm
baseline. A quintic envelope starts at zero at 117 s, reaches full offset at
132 s, holds through 169 s, then fades to zero at 233 s. Endpoint position,
velocity and acceleration offsets are zero. Timing, joint/motor limits, CoP
margins, static reserve and collision checks retain their previous values.

The ranking objective is time-average squared **maximum static motor-cap
fraction** over 117–233 s. It is not measured power, energy or walking efficiency.
Candidates that fail a gate or materially increase peak static utilization are
excluded. A 0.5% relative objective reduction is the minimum improvement criterion;
a smaller change retains the baseline. This is a screening criterion for this
experiment, not a general robotics acceptance standard.

The best search candidate and the baseline are then solved on the same fivefold
interpolated grid (536 samples). Linear interpolation of the saved level base and
flat world foot poses defines that comparison path; it does not recover the exact
original planner between samples. Collision and speed checks remain sampled.

Final evidence: `evidence/robokit/search-002/`. All 25 coarse candidates passed.
The best has a +5 mm forward, +10 mm upward peak pelvis offset, preserving the
75 mm foot placement. The denser objective decreases only about **0.41%**.
Peak static utilization remains **53.06%**, while sampled joint speed increases
from **0.03878 to 0.04564 rad/s**, below the unchanged 0.05 rad/s envelope.
The disposition is **retain_existing_baseline**. No motion is promoted to a
controller and no Gazebo or Aria stance result is claimed.

Outputs include `candidates.json` (every result/rejection), `summary.json`, the
candidate and baseline dense trajectories, URDF snapshot, and comparison plot.
`search-001` retains the earlier development run. Search selection and envelope
regression tests are included in `test_search.py`.

```bash
XDG_CACHE_HOME=/tmp/robokit-cache .venv-robokit/bin/python -m unittest tools.robokit_lab.test_adapter tools.robokit_lab.test_ik tools.robokit_lab.test_search
```
