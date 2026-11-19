"""Public launch entry point. Component construction lives in the Python package."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from my_robot_description.runtime.startup import launch_setup


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                choices=["true", "false"],
                description="Open the Gazebo graphical client.",
            ),
            DeclareLaunchArgument(
                "spawn_height",
                default_value="0.57",
                description="Initial robot origin height in metres.",
            ),
            DeclareLaunchArgument(
                "controller_timeout",
                default_value="60",
                description="Seconds to wait for controller manager services.",
            ),
            DeclareLaunchArgument(
                "stance",
                default_value="straight",
                choices=["straight", "crouch"],
                description="Hold straight or execute a calculated slow crouch.",
            ),
            DeclareLaunchArgument(
                "knee_bend",
                default_value="0.4",
                description="Crouch knee angle in radians.",
            ),
            DeclareLaunchArgument(
                "transition_duration",
                default_value="8.0",
                description="Crouch transition duration, 4 to 60 seconds.",
            ),
            DeclareLaunchArgument(
                "initial_knee_bend",
                default_value="0.02",
                description="Crouch startup knee angle; 0 reproduces the hard-stop baseline. Straight mode stays at zero.",
            ),
            DeclareLaunchArgument(
                "tracking_log",
                default_value="",
                description="Optional JSONL file for controller desired/actual/error diagnostics.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
