# 3. Find a problem by responsibility

Keep the failed output directory. The first important error is usually in `launch.log`; the last traceback often only reports its consequences.

| Symptom | Inspect first | Responsibility |
|---|---|---|
| Panel unreachable | Stable `scripts/start_force_panel_background.sh`, panel service | Local service startup |
| Invalid settings / prerequisite blocked | `tools/force_panel/config.py`, `jobs.py` | GUI request validation |
| Gazebo or a controller never starts | `launch/roll_standing.launch.py`, `runtime/startup.py` | Startup dependency chain |
| Planner says unreachable / collision / torque reserve | `motion/` and `model/` | Desired geometry and statics |
| Live force/geometry guard stops motion | Joint log's `load_guard` / `lift_guard`, raw contacts and COM | Measured support and freshness |
| Tracking or motor saturation | `control/roll_posture.py`, target/position/requested/command columns | Joint feedback and caps |
| Missing IMU or stale messages | `sensing/`, bridge setup and source timestamps | Measurement transport |
| COM animation lags | `nodes/sensing/show_center_of_mass.py`, `visualization/` | Drawing; independent capture must continue |
| Motion finishes but `passed` is false | `summary.json`, `validation/` | Evidence requirements |
| Only a plot fails | `scripts/plots/` | Rendering; do not invent a pass result |

## Read one record in order

1. Identify the experiment ID, mode, actuator profile and source snapshot.
2. Confirm readiness and the first phase's simulated-time origin.
3. Locate the failing phase and the earliest guard reason.
4. Compare requested foot position and measured sole corners, including units.
5. Check source timestamp ages and coverage before interpreting an absent contact or force.
6. Check requested torque, clipped command and integral. Saturation is a motor limit, not permission to increase it without calculation.
7. Inspect the independent audit and plot together.

## Changes must preserve evidence

A reorganization changes source paths and hashes even if calculations are unchanged. Older measurements remain historical results from their recorded source version. The pre-organization source snapshot is in `evidence/preparation/code-organization/before-source.tar.gz`; the path migration manifest is alongside it. Do not overwrite old summaries to pretend they came from new code.

For a new run use a new output directory. The GUI does this automatically. For a code-only change run relevant tests and build first; for a dynamics change also run a new measured simulation.
