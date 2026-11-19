# Executable ROS node adapters

- `control/hold_roll_stance.py`: current ten-joint adapter. Receives feedback, checks readiness/guards, samples motion, calls PID and publishes efforts.
- `sensing/show_center_of_mass.py`: independent COM/sole observation capture and optional markers. Capture is buffered separately from drawing.
- `sensing/sample_contacts.py`: rate-limited forwarding with original contact source timestamps.
- `legacy/`: earlier six-joint stance and balance nodes, still needed by historical GUI modes and bench experiments.

CMake installs the same executable basenames as before. Launch files therefore keep stable ROS executable names while the source tree explains each node's role.
