# Robot geometry and physical model

Lengths, masses, joint axes, joint order, motor caps and forward/inverse kinematics. No ROS nodes or motor command publishing.

| File | Responsibility |
|---|---|
| [roll_actuators.py](roll_actuators.py) | Named output-shaft actuator budgets shared with the Xacro model. |
| [roll_geometry.py](roll_geometry.py) | Offline URDF tree/FK checks for the separate roll-joint prototype; no ROS. |
| [roll_stance.py](roll_stance.py) | Ten-joint symmetric standing geometry and static loads, independent of ROS. |
| [sagittal_step.py](sagittal_step.py) | Offline sagittal leg geometry and smooth swing targets; no ROS or commands. |
| [stance.py](stance.py) | Planar standing geometry, independent of ROS execution. |
| [statics.py](statics.py) | Static joint loads for symmetric, flat-footed, two-foot support. |
| [torque.py](torque.py) | Pendulum model and bounded joint torque law; no Gazebo/ROS dependencies. |
