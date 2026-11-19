# Plots

Draw recorded or predicted results. A plot is a view of evidence, not the pass/fail authority.

Run tools from the workspace root with `python3 -m scripts.plots.<tool> --help`. Direct file execution also works.

| Tool | Purpose |
|---|---|
| [plot_actuator_upgrade.py](plot_actuator_upgrade.py) | Compare actuator budgets with mass-aware static and standing evidence. |
| [plot_alternating_walk.py](plot_alternating_walk.py) | Plot both feet, clearance, measured load exchange and forward COM progress. |
| [plot_ankle_diagnosis.py](plot_ankle_diagnosis.py) | Visual audit of recorded ankle commands against a conditional static model. |
| [plot_balance_observer.py](plot_balance_observer.py) | Plot body measurements and hypothetical ankle torque, without applying it. |
| [plot_balance_repeats.py](plot_balance_repeats.py) | Show individual off/on repeats and observed ranges, not confidence intervals. |
| [plot_com_response.py](plot_com_response.py) | Plot recorded COM relative to the feet; no dynamic-stability verdict. |
| [plot_duration_comparison.py](plot_duration_comparison.py) | Plot the recorded responses of a completed duration comparison. |
| [plot_duration_repeats.py](plot_duration_repeats.py) | Show individual duration repeats and observed peak ranges. |
| [plot_effort_push.py](plot_effort_push.py) | Plot a no-push reference beside a gentle effort-mode disturbance. |
| [plot_effort_standing.py](plot_effort_standing.py) | Show measured torque terms for the settled left leg; right leg is symmetric. |
| [plot_first_step.py](plot_first_step.py) | Plot planned or independently measured single forward placement. |
| [plot_force_limit_study.py](plot_force_limit_study.py) | Plot completed force-study cases; omitted/incomplete cases are not passes. |
| [plot_lateral_experiment.py](plot_lateral_experiment.py) | Panel artifact for a P54 transfer: measured COM, forces and lateral IMU. |
| [plot_lateral_shift.py](plot_lateral_shift.py) | Visualize measured COM/foot motion separately from static force estimates. |
| [plot_lean_support.py](plot_lean_support.py) | Visualize saved static lean-search evidence; no commands or simulation. |
| [plot_lowering.py](plot_lowering.py) | Plot measured bilateral lowering and return, separate from lateral pulses. |
| [plot_physics_audit.py](plot_physics_audit.py) | Explain the measured DART split-step ankle torque balance visually. |
| [plot_push_trials.py](plot_push_trials.py) | Plot command timing, pitch, joint errors and contact freshness from push trials. |
| [plot_recovery_control.py](plot_recovery_control.py) | Compare explicit PD/PID evidence and show what the added integral does. |
| [plot_roll_standing.py](plot_roll_standing.py) | Plot a roll-standing recording, optionally alongside an unsuccessful startup. |
| [plot_support_search.py](plot_support_search.py) | Render saved offline posture-search results without recomputing the search. |
| [plot_timestep_comparison.py](plot_timestep_comparison.py) | Plot timestep sensitivity without conflating recovery and physics-audit success. |
| [plot_torque_bench.py](plot_torque_bench.py) | Show measured torque saturation, limited holding, and mechanical-stop tests. |
| [plot_tracking.py](plot_tracking.py) | Compare a failed and successful run using recorded controller measurements. |
| [render_first_step_replay.py](render_first_step_replay.py) | Educational replay reconstructed from measured joints and physics sole corners. |
