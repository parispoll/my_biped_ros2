"""Persistent forward-walking simulation: wait for commands, hold after landing."""

import os
import time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Both launch-time geometry calculations and child nodes must use this profile.
    os.environ["BIPED_ACTUATOR_PROFILE"] = "robotis_p54"
    source = (
        Path(get_package_share_directory("my_robot_description"))
        / "launch/roll_standing.launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "real_time_factor",
                default_value="0.5",
                choices=["0.25", "0.5", "1.0"],
                description="Wall-clock pacing; use 0.5 for GUI sensor-delivery headroom",
            ),
            SetEnvironmentVariable("BIPED_ACTUATOR_PROFILE", "robotis_p54"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(source)),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "real_time_factor": LaunchConfiguration("real_time_factor"),
                    "interactive_enabled": "true",
                    "fault_report": str(
                        Path.cwd()
                        / "evidence/runtime"
                        / f"controller-fault-{time.time_ns()}.json"
                    ),
                    "alternating_walk_enabled": "true",
                    "lowering_enabled": "true",
                    "lowering_distance": "0.04",
                    "walk_steps": "10",
                    "walk_step_length": "0.075",
                    "walk_time_scale": "0.5",
                    "unload_direction": "left",
                }.items(),
            ),
        ]
    )
