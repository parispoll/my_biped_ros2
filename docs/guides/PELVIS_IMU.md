# Step 3a: measure pelvis orientation with an IMU

Update: the intermittent crouch startup was investigated and mitigated with a balanced
initial knee clearance. See [the tracking investigation](../history/CROUCH_TRACKING_DIAGNOSIS.md).

Only the IMU step is implemented here. Foot contacts, disturbance experiments and
feedback balance remain later, separate steps.

## Signal path

```
Pelvis motion (base_link)
  -> Gazebo pelvis_imu sensor, 100 Hz
  -> Gazebo transport /biped/imu, gz.msgs.IMU
  -> pelvis_imu_bridge
  -> ROS /biped/imu, sensor_msgs/msg/Imu
```

The sensor has no additional link or mass. Its pose is zero relative to base_link:
its origin and axes coincide with the existing pelvis coordinate frame.

## Exactly what changed

Paths are relative to `src/my_robot_description` unless specified otherwise.

### New: urdf/components/imu.xacro

A separate component contains:

```xml
<gazebo reference="base_link">
  <sensor name="pelvis_imu" type="imu">
    <pose>0 0 0 0 0 0</pose>
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <topic>/biped/imu</topic>
    <imu>
      <orientation_reference_frame>
        <localization>CUSTOM</localization>
        <custom_rpy parent_frame="world">0 0 0</custom_rpy>
      </orientation_reference_frame>
    </imu>
  </sensor>
</gazebo>
```

- `reference="base_link"` attaches the sensor to the existing pelvis.
- `type="imu"` selects the sensor implementation.
- `pose` gives zero translation and rotation relative to that link.
- `always_on` requests continuous updates.
- `update_rate` requests 100 measurements per simulated second.
- `topic` selects the Gazebo transport topic.
- The reference frame requests orientation relative to the world rather than
  automatically treating the initial sensor orientation as upright.

No artificial sensor noise is configured for this first integration test.

### Changed: urdf/my_biped.urdf.xacro

Added one include:

```xml
<xacro:include filename="components/imu.xacro"/>
```

The entry point assembles this sensor component alongside the existing robot
components. Segment dimensions, masses and joint interfaces are unchanged.

### Changed: worlds/standing.sdf

Added a world system:

```xml
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu" />
```

The sensor definition says what to measure and where. This world system actually
updates the IMU sensors during simulation. An IMU needs no camera renderer.

### New: my_robot_description/sensors.py

Added `make_imu_bridge()`, separate from joint commands and the clock bridge:

```python
arguments=['/biped/imu@sensor_msgs/msg/Imu[gz.msgs.IMU']
parameters=[{'override_frame_id': 'base_link', 'use_sim_time': True}]
```

The `[` selects one-way Gazebo-to-ROS forwarding. The bridge changes the message
format, not the robot motion. Its frame override labels measurements as base_link,
which is correct because the sensor is aligned and colocated with that link.
A frame-name override would not rotate data if we later mounted the IMU differently.

### Changed: my_robot_description/startup.py

Imports and constructs the new bridge, adds it to startup, and monitors it as an
essential process under the diagnostic stage `imu-bridge`. If the bridge process
exits unexpectedly, launch stops. Process lifetime alone does not prove that fresh
measurements arrive; the separate checker tests that.

### New: scripts/checks/check_imu.py at the workspace root

A stationary-sensor checker subscribes with sensor-data QoS and verifies:

- Expected `base_link` frame and available orientation/angular-rate/acceleration.
- Finite values and normalized orientation quaternion.
- Strictly advancing sensor timestamps.
- Approximately 100 received samples per simulated second (90–110 Hz tolerance).
- Near-upright orientation and near-zero angular speed.
- Specific force near `(0, 0, +9.81)` m/s² while stationary.

It has a wall-clock timeout, so missing or paused data cannot make it wait forever.
The default measurement duration is five simulated seconds after a settling period.
This is a stationary validation tool, not a balance controller.

### New: tests/test_imu.py at the workspace root

Checks upright measurements, quaternion-to-pitch sign conversion using synthetic
positive/negative Y rotations, rejection of incorrect frames, unavailable data,
invalid quaternions and excessive body tilt. Synthetic sign checks do not replace
a future controlled physical tilt experiment in Gazebo.

Existing package dependencies already included the bridge and `sensor_msgs`, so
package metadata and build installation rules did not need changing.

## How to run and inspect it

In the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
export ROS_DOMAIN_ID=42
export GZ_PARTITION=biped_recovery
ros2 launch my_robot_description standing.launch.py gui:=true stance:=crouch
```

In a second terminal, with the same setup and environment:

```bash
ros2 topic echo /biped/imu --once
```

After `[biped:standing]` appears, run:

```bash
python3 scripts/checks/check_imu.py
```

To run the checker immediately after starting a new simulation, allow the crouch
to finish with `--settle-time 15 --wall-timeout 35`.

## Reading the message

- `orientation` is a quaternion: x/y/z/w are not roll/pitch/yaw angles.
- `angular_velocity.y` is rotation rate around the pelvis Y axis, in rad/s.
- `linear_acceleration` is expressed in the sensor's local axes, in m/s².
- A stationary accelerometer measures specific force: upright Z is approximately
  +9.81 m/s² here, not zero. Do not subtract gravity blindly during motion.
- `header.frame_id` names the sensor axes, while the configured orientation
  reference is the world frame. Those are different roles.
- Default zero covariance entries indicate unspecified covariance; they do not
  certify perfect real-world sensor accuracy.

A real IMU normally needs orientation estimation/calibration and realistic noise.
The simulator currently supplies ideal orientation; that is sufficient for learning
and validating the signal path before adding feedback.

## Recorded verification, 2026-10-26

- Package build passed and all 20 automated tests passed.
- Sensor survived URDF-to-SDF conversion.
- Expanded physical model remained identical after removing only the new sensor extension.
- ROS IMU check: 502 samples in 5.01 simulated seconds, 100.0 Hz, frame `base_link`.
- Maximum absolute pitch approximately 0 rad; angular speed approximately 0 rad/s.
- Mean acceleration approximately `(0, 0, 9.81)` m/s².
- Repeated crouch completed; separate 30-second body check passed near the predicted 0.55449 m height, with near-zero tilt.
- Verification shut the test simulator down after both checks completed.

Evidence: `evidence/imu-model-check.json`, `evidence/imu-stationary-check.json`,
`evidence/imu-launch.log`, and the separate body regression check
`evidence/imu-standing-check.json`.

One repeated crouch startup aborted when knee tracking error exceeded the existing
0.08 rad path tolerance. The diagnostic shutdown worked and the tolerance was not
relaxed. The same test was repeated; see the successful/failing launch logs separately.
The cause of the intermittent transition failure is not established. Investigate this
before using the crouch transition as a reliability baseline for balance experiments.
Failure evidence is preserved in `evidence/imu-crouch-failure.log`.

No disturbances were applied. Dynamic response and physical pitch-sign verification
are separate future experiments.

References:
- https://gazebosim.org/docs/harmonic/sensors/
- https://index.ros.org/p/ros_gz_bridge/
