# Joint feedback control

Turn target angles and measured joint feedback into bounded motor torques. Keep simulated time, anti-windup and freshness checks here.

| File | Responsibility |
|---|---|
| [balance_control.py](balance_control.py) | Gate and ramp the optional double-support ankle correction; no ROS. |
| [feedback_timing.py](feedback_timing.py) | Distinguish fresh feedback after dropped messages from stale feedback. |
| [imu_watchdog.py](imu_watchdog.py) | Separate first-message discovery from an established IMU stream's watchdog. |
| [integral.py](integral.py) | Bounded torque-valued integral state with conditional-integration anti-windup. |
| [posture_effort.py](posture_effort.py) | Joint posture control math; no ROS and no body-balance feedback. |
| [posture_setup.py](posture_setup.py) | Read controller-written startup metadata without querying a live feedback node. |
| [roll_posture.py](roll_posture.py) | Bounded ten-joint posture PID; no ROS, gait or body-roll feedback. |
| [roll_timing.py](roll_timing.py) | Check latest IMU freshness against the shared clock, not another sensor stamp. |
