# Analysis

Read saved experiments, compare results, or re-audit them. No robot commands.

Run tools from the workspace root with `python3 -m scripts.analysis.<tool> --help`. Direct file execution also works.

| Tool | Purpose |
|---|---|
| [analyze_balance_cap_study.py](analyze_balance_cap_study.py) | Compare original guarded partial trace and candidate full recovery evidence. |
| [analyze_body_damping_study.py](analyze_body_damping_study.py) | Compare full gain-4/gain-8 responses and direction-aware rebound metrics. |
| [analyze_damped_neighborhood.py](analyze_damped_neighborhood.py) | Summarize completed and guarded neighborhood trials without conflating them. |
| [analyze_foot_loads.py](analyze_foot_loads.py) | Audit/plot captured contact forces; never synthesize missing measurements. |
| [analyze_force_response.py](analyze_force_response.py) | Diagnose one measured trial, including partial guard-stopped recordings. |
| [analyze_load_feedback.py](analyze_load_feedback.py) | Compare independently captured forces in matched biased-reference trials. |
| [analyze_physics_probe.py](analyze_physics_probe.py) | Check the final stationary second of a physics + DART probe experiment. |
| [analyze_unloaded_ankle.py](analyze_unloaded_ankle.py) | Approximate free-foot sampled ankle dynamics; not full-robot stability proof. |
| [analyze_unloading_limits.py](analyze_unloading_limits.py) | Offline scan of the current fixed-height, fixed-sole unloading path. |
| [analyze_zero_support.py](analyze_zero_support.py) | Finite-difference URDF gravity/mass model at a saved zero-load endpoint. |
| [body_damping_metrics.py](body_damping_metrics.py) | Direction-aware measured response metrics; no ROS or plotting. |
| [compare_active_balance.py](compare_active_balance.py) | Compare nominally matched off/on push recordings; report measured differences. |
| [compare_lateral_pulse.py](compare_lateral_pulse.py) | Compare a matched P54 reference with an externally applied lateral pulse. |
| [duration_repeat_analysis.py](duration_repeat_analysis.py) | Verify duration-study evidence and describe two observed repeats per direction. |
| [reaudit_alternating_walk.py](reaudit_alternating_walk.py) | Read-only reanalysis of a physically completed walking recording. |
| [reaudit_lateral_shift.py](reaudit_lateral_shift.py) | Re-audit immutable raw lateral records with the recorded actuator profile. |
| [summarize_force_repeat_check.py](summarize_force_repeat_check.py) | Verify fixed-pulse repeats and plot the observed, possibly partial response. |
| [verify_damped_neighborhood.py](verify_damped_neighborhood.py) | Verify saved neighborhood evidence, including control math in partial trials. |
