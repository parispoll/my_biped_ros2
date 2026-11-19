# Gazebo physics plugins

These C++ libraries run inside Gazebo, separately from ROS Python callbacks.

- `com_probe.cpp`: read-only mass-weighted COM and actual sole corners, in world metres.
- `contact_state_probe.cpp`: read-only physics-stamped contact/air evidence, including explicit empty packets after sensor initialization.
- `viscous_damping.cpp`: passive physical damping, `torque = -coefficient × velocity`, independent of requested motor effort.
- `physics_probe.cpp`, `dart_probe.cpp`: retained backend diagnostics from earlier torque-accounting studies.

Library filenames and registered plugin names stay unchanged. CMake's shared installation helper removes repeated build rules without changing the linked dependencies.
