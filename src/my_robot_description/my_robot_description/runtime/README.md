# ROS/Gazebo startup composition

Prepare descriptions and launch nodes in dependency order. Refuse to continue after a failed essential startup stage.

| File | Responsibility |
|---|---|
| [controllers.py](controllers.py) | Controller activation helpers; controller parameters belong in config/. |
| [description.py](description.py) | Load robot XML and prepare the controller configuration for Gazebo. |
| [roll_description.py](roll_description.py) | Prepare the roll standing description and explicit physical damping. |
| [sensors.py](sensors.py) | ROS bridges for measurements, independent of joint commands. |
| [simulation.py](simulation.py) | Gazebo process, robot creation, and simulation clock bridge. |
| [startup.py](startup.py) | Startup ordering, input validation, and explicit failure handling. |
