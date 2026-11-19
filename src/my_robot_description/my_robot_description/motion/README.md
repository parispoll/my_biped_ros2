# Motion planning and support policy

Build safe joint targets and static feedforward torques over simulated time. The plan says what should happen; independent measurements determine whether it did.

| File | Responsibility |
|---|---|
| [alternating_walk.py](alternating_walk.py) | Finite slow alternating gait: right/left/right placements and a stable stop. |
| [bilateral_lowering.py](bilateral_lowering.py) | First lowered-posture experiment: centered COM, flat planted feet, no lift. |
| [first_step.py](first_step.py) | One slow forward placement and reload into a new staggered stance. |
| [foot_lift.py](foot_lift.py) | Offline first-lift feasibility only. No ROS, commands or measured results. |
| [lateral_shift.py](lateral_shift.py) | Small fixed-foot pelvis translation and quasi-static feedforward; no ROS. |
| [live_foot_lift.py](live_foot_lift.py) | Guarded vertical lift at 2, 10 or 30 mm, with landing and return. |
| [load_feedback.py](load_feedback.py) | Bounded bilateral load-share admittance; independent of ROS. |
| [lowered_shift.py](lowered_shift.py) | 40 mm lowered posture plus a small fixed-foot lateral translation. |
| [lowered_unload.py](lowered_unload.py) | Separate positive-load targets at a lowered posture; never a foot-lift mode. |
| [partial_unload.py](partial_unload.py) | Derive a conservative bilateral unloading plan within the existing model. |
| [zero_load.py](zero_load.py) | Explicit zero-load transition. A fixed-sole reference is not a foot lift. |
