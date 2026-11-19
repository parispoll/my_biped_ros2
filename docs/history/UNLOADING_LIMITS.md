# Can the current sideways motion unload a foot completely?

The offline calculation says **no for the current fixed-height, upright-pelvis path with sole-center reaction forces**. This is a limitation of this motion and its pressure assumption, not proof that the robot cannot balance on one leg in any posture.

## Results

| Constraint reached | Pelvis shift | COM shift | Lighter foot load | Loaded hip-roll torque |
|---|---:|---:|---:|---:|
| 80% of current motor cap | 57.05 mm | 40.61 mm | 36.94 N (26.11%) | 4.8 Nm |
| Full current motor cap | 66.75 mm | 47.52 mm | 31.19 N (22.05%) | 6.0 Nm |
| Last reachable sampled pose | 103.30 mm | 73.50 mm | 9.57 N (6.76%) | 11.52 Nm — exceeds cap |

At 103.4 mm the inverse kinematics rejects the target as unreachable or near a straight-leg singularity. Geometry was sampled every 0.1 mm; the 103.30 mm value is the last sampled reachable point, not an exact singularity boundary. The torque crossings were refined by bisection. Mirroring the scan gives equal load fractions and torque utilization on the opposite leg.

The 80% point is a static planning bound. Dynamic corrections need torque too, so it is not a validated command target. Existing live trials remain limited to the previously verified 40% and 35% lighter-foot targets, with their 25% measured share floor.

## Why zero load requires more movement

For our assumed sole-center reactions, moment balance is

`F_L + F_R = W`

`b (F_L − F_R) = W y_COM`, with `b = 0.085 m`.

Consequently `F_R/W = (1 − y_COM/b)/2`. To make the right reaction zero, this assumption requires the COM at `y_COM = b = 85 mm`.

The current path nearly straightens the legs while reaching only about 73.5 mm of COM shift. It also reaches the loaded hip-roll torque budget much earlier. Both conclusions come from the actual URDF dimensions and masses.

## Why the legs straighten

The pelvis stays at its original height while its distance sideways from each ankle increases. For the pitch-chain inverse kinematics, the target downward distance becomes `sqrt(ankle_z² + displacement²)`. With fixed thigh and shin lengths, the knee must straighten as this distance grows. At full extension there is no further reach available.

## Why the hip-roll torque increases

`double_support_loads` sums gravity moments of every downstream body and the prescribed ground-reaction moment about each joint axis. The requested holding torque is the negative of that external moment. For a single vertical force, the roll-axis moment is `lateral lever arm × vertical force`; the complete calculation includes all distal masses and their individual lever arms.

As load transfers, both the force and its lever arm at the supporting hip change. The resulting hip-roll demand grows from approximately 2.56 Nm at the tested 65/35 pose to 4.8 Nm and then 6 Nm at the two boundaries above. Knee torque is not the first limiting motor in this scan.

## Code and verification

Added `scripts/analyze_unloading_limits.py`: a read-only model analysis. It evaluates geometry and static torques beyond the live command budget solely to diagnose the constraint. It does not launch Gazebo, publish torques, increase limits or change the controller.

Added `tests/test_unloading_limits.py`: checks agreement with the existing planner inside its valid budget, force balance, the two bracketed torque boundaries, mirror symmetry, and rejection of the unreachable target. The full scan also checks fixed-foot position error below 1e-10 m and agreement with the previously validated 35% target.

Output: `evidence/unloading-limits-01/limits.json` contains the complete sampled scan and refined crossings; `limits.png` plots the lighter-foot fraction and individual loaded-leg torque utilization. These are model predictions, not new measured simulation trials.

To reproduce from the workspace:

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/analyze_unloading_limits.py --output evidence/my-unloading-limits-01
```

Use a fresh output directory.

## Next development step

Before increasing motor caps or lifting a foot, examine a small family of alternative postures and ground-pressure locations. Lowering the pelvis can restore knee-bend reach; changing posture can redistribute gravity moments. Finite feet can also place their center of pressure away from the sole center, changing the reaction-force split and ankle torques. Those effects must be calculated together, with joint and motor constraints, then tested with measured center of pressure and foot forces.

A new feasible static pose would still need a gradual transfer test, validated handling of genuine zero-contact readings, and single-support balance control before a foot-lift experiment.

Validation: all 257 unit tests passed with the ROS environment sourced and ROS logs directed to the workspace. The evidence folder preserves earlier test invocations that failed due to missing environment paths or an unwritable default ROS log directory.
