# Studies

Batch comparisons and historical research tools; outside the main walking path.

Run tools from the workspace root with `python3 -m scripts.studies.<tool> --help`. Direct file execution also works.

| Tool | Purpose |
|---|---|
| [run_active_balance_comparison.py](run_active_balance_comparison.py) | Run fresh off/on pairs for +/-2 N, optionally reusing one completed active case. |
| [run_balance_cap_study.py](run_balance_cap_study.py) | Fresh baseline plus repeated 25 N candidate checks, with read-only COM. |
| [run_balance_repeat_study.py](run_balance_repeat_study.py) | Reuse one verified matched study, run a reversed-order repeat, summarize both. |
| [run_body_damping_study.py](run_body_damping_study.py) | Compare pitch-rate gain 4 versus 8 at the same 1.5 N m ankle correction cap. |
| [run_damped_neighborhood.py](run_damped_neighborhood.py) | Repeat two nearby pulses with fixed damped feedback and preserve guard stops. |
| [run_duration_comparison.py](run_duration_comparison.py) | Compare 2 N pulses of .20/.30 s using fresh paired experiments. |
| [run_duration_repeat_study.py](run_duration_repeat_study.py) | Reuse verified direction studies and run reversed-duration repeats sequentially. |
| [run_force_limit_study.py](run_force_limit_study.py) | Escalate sagittal force, preserving fresh references and the first failure. |
| [run_force_repeat_check.py](run_force_repeat_check.py) | Repeat one fixed pulse in fresh measured worlds, including guard outcomes. |
| [run_push_trials.py](run_push_trials.py) | Own fresh isolated simulations for a no-push control and repeated small pushes. |
| [run_timestep_comparison.py](run_timestep_comparison.py) | Compare physics steps and repeats with fixed effort-control settings. |
| [search_lean_support.py](search_lean_support.py) | Offline asymmetric-leg and pelvis-roll support search; no robot commands. |
| [search_support_postures.py](search_support_postures.py) | Search static pelvis/pressure candidates offline; sends no robot commands. |
