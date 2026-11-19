# Independent experiment validation

Replay saved observations and controller records. Report pass/fail and diagnostics without issuing robot commands.

| File | Responsibility |
|---|---|
| [alternating_walk_audit.py](alternating_walk_audit.py) | Independent raw-contact, sole, pose and motor replay for a forward step. |
| [balance_audit.py](balance_audit.py) | Check observation coverage, candidate math and unchanged motor composition. |
| [com_audit.py](com_audit.py) | Coverage checks for read-only COM observations in measured trials. |
| [experiment_failure.py](experiment_failure.py) | Extract an actionable child-process failure for experiment runner errors. |
| [first_step_audit.py](first_step_audit.py) | Independent raw-contact, sole, pose and motor replay for a forward step. |
| [lateral_audit.py](lateral_audit.py) | Audit the small bilateral COM-shift experiment without claiming foot forces. |
| [lowering_audit.py](lowering_audit.py) | Separate height/pressure checks for bilateral lowering; no unloading claim. |
| [motion_audit.py](motion_audit.py) | Shared command and body-sensor checks for single steps and walking. |
| [physics_audit.py](physics_audit.py) | Checks for steady-state physics commands and DART's split-step torque balance. |
| [roll_audit.py](roll_audit.py) | Independent evidence checks for ten-joint standing and passive damping. |
| [tracking.py](tracking.py) | Record controller references and measurements without changing commands. |
