# Disturbance experiment helpers

Configure and publish simulation disturbances and collect their timing. Requested force is not calibrated measured force.

| File | Responsibility |
|---|---|
| [disturbance.py](disturbance.py) | Configuration and analysis for bounded sagittal push experiments. |
| [effort_push.py](effort_push.py) | Configuration and quantitative checks for effort-mode disturbance limit experiments. |
| [lateral_pulse.py](lateral_pulse.py) | Small scheduled Gazebo lateral wrench with guaranteed cleanup attempt. |
