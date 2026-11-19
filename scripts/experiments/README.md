# Experiments

Run one isolated simulation and save its evidence. The main walking runner is run_lateral_shift.py.

Run tools from the workspace root with `python3 -m scripts.experiments.<tool> --help`. Direct file execution also works.

| Tool | Purpose |
|---|---|
| [effort_push_experiment.py](effort_push_experiment.py) | Record a gentle push on effort standing; normally started by run_effort_push.py. |
| [push_experiment.py](push_experiment.py) | Record one small push on a settled crouch; prefer run_push_trials.py for owned simulations. |
| [record_lateral_sensors.py](record_lateral_sensors.py) | Start read-only sensor capture before warm-up; never command robot joints. |
| [repeat_crouch.py](repeat_crouch.py) | Repeat fresh isolated crouch launches, preserving logs and tracking data. |
| [run_contact_transition_bench.py](run_contact_transition_bench.py) | Test actual Gazebo -> ROS bridge -> existing sampler contact/air/contact. |
| [run_effort_push.py](run_effort_push.py) | Run a no-push reference and a gentle push in separate fresh effort simulations. |
| [run_effort_standing.py](run_effort_standing.py) | Launch, verify and stop one isolated effort-standing experiment. |
| [run_lateral_shift.py](run_lateral_shift.py) | Capture before warm-up, run a small bilateral COM shift, audit closed files. |
| [run_roll_standing.py](run_roll_standing.py) | Verify isolated ten-joint two-foot standing, never the old six-joint controller. |
| [run_torque_bench.py](run_torque_bench.py) | Run independent effort-joint tests, each with a fresh owned simulator. |
| [run_visual_push.py](run_visual_push.py) | One visual force pulse, without a reference trial or recovery assessment. |
| [torque_bench_experiment.py](torque_bench_experiment.py) | Record one effort-controlled pendulum test; use run_torque_bench.py for isolated runs. |
