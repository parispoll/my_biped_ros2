# Regression and behavior checks

Source ROS and `install/setup.bash`, then run `ROS_LOG_DIR=/tmp/biped-test-logs python3 -m unittest discover -s tests` from the workspace root. Test filenames name the feature they protect. Their module descriptions explain the contract.

Tests use the same responsibility-based imports as application code. Planner tests can take around a minute because they solve and collision-check hundreds of poses. Unit tests and saved-recording replay verify organization changes without requiring a new walking simulation.
