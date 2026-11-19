# Sensor interpretation

Decode IMU, contact, force and COM measurements. Fresh empty contacts mean airborne; missing or stale messages never mean zero load.

| File | Responsibility |
|---|---|
| [balance.py](balance.py) | Sagittal balance proposals only: no ROS, publishers, or actuator commands. |
| [com_geometry.py](com_geometry.py) | Pure geometry for COM telemetry; no ROS, Gazebo or motor commands. |
| [contact_state.py](contact_state.py) | Explicit source-stamped contact states; silence is never zero support. |
| [contacts.py](contacts.py) | Interpret raw collision pairs; this module does not estimate support forces. |
| [foot_loads.py](foot_loads.py) | World-frame ground forces from this Harmonic foot-first contact schema. |
