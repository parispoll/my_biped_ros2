"""ROS bridges for measurements, independent of joint commands."""

from launch_ros.actions import Node


def make_imu_bridge():
    """Forward the pelvis IMU to ROS, expressed in the existing base_link frame.

    The sensor is colocated and aligned with base_link, so the frame override is
    a label correction, not a rotation or translation of measurements.
    """
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="pelvis_imu_bridge",
        output="screen",
        arguments=["/biped/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"],
        parameters=[{"override_frame_id": "base_link", "use_sim_time": True}],
    )


def make_contact_bridge(side):
    """Forward raw contact pairs for one foot; no coordinate-frame relabeling.

    Consumers identify ground contact by collision names. This bridge does not
    interpret contact forces or turn missing messages into 'no contact'.
    """
    if side not in ("left", "right"):
        raise ValueError("Contact bridge side must be left or right")
    topic = f"/biped/{side}_foot/contacts"
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"{side}_foot_contact_bridge",
        output="screen",
        arguments=[f"{topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"],
        parameters=[{"use_sim_time": True}],
    )
