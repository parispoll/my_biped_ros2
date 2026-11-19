"""Controller activation helpers; controller parameters belong in config/."""

from launch_ros.actions import Node

STATE_BROADCASTER = "joint_state_broadcaster"
TRAJECTORY_CONTROLLER = "joint_trajectory_controller"


def make_controller_spawner(controller_name, timeout):
    """Activate one controller so a failure identifies the exact stage."""
    return Node(
        package="controller_manager",
        executable="spawner",
        name=f"activate_{controller_name}",
        arguments=[
            controller_name,
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            str(timeout),
        ],
        output="screen",
    )
