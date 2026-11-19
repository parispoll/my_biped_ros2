# Planning

Calculate or preview geometry and motion without publishing motor commands.

Run tools from the workspace root with `python3 -m scripts.planning.<tool> --help`. Direct file execution also works.

| Tool | Purpose |
|---|---|
| [calculate_geometry_limits.py](calculate_geometry_limits.py) | Offline educational P54 geometry audit. Does not command ROS or Gazebo. |
| [calculate_step_clearance.py](calculate_step_clearance.py) | Design and check a flat-ground 50 mm swing; never commands a robot. |
| [plan_partial_unload.py](plan_partial_unload.py) | Derive a fixed-foot partial unloading target; no simulation or motor commands. |
| [plan_standing_torques.py](plan_standing_torques.py) | Calculate static holding torques; does not launch or command the robot. |
| [preview_alternating_walk.py](preview_alternating_walk.py) | Offline full alternating-stance preflight, without simulation or commands. |
| [preview_first_step.py](preview_first_step.py) | Offline first-step geometry/static/speed preflight; no motor commands. |
| [preview_foot_lift.py](preview_foot_lift.py) | Save a P54 2 mm foot-lift feasibility preview. Never starts Gazebo or ROS nodes. |
| [preview_sagittal_step.py](preview_sagittal_step.py) | Preview one geometric foot swing; never starts Gazebo or publishes commands. |
| [roll_model_geometry.py](roll_model_geometry.py) | Compatibility import for offline roll-model tools; math lives in the package. |
