"""Gazebo process, robot creation, and simulation clock bridge."""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

WORLD_NAME = "standing"
ROBOT_NAME = "my_biped"


def make_simulator(world_file, gui, verbosity=None, extra_env=None, camera="side"):
    """Run the local world with ROS plugin directories available."""
    if camera not in ("side", "front"):
        raise ValueError("Camera must be side or front")
    flags = [] if gui else ["-s"]
    if gui:
        flags += [
            "--gui-config",
            str(
                Path(get_package_share_directory("my_robot_description"))
                / (
                    "config/gazebo_front.config"
                    if camera == "front"
                    else "config/gazebo_horizontal.config"
                )
            ),
        ]
    if verbosity is not None:
        if verbosity not in range(5):
            raise ValueError("Gazebo verbosity must be 0–4")
        flags += ["-v", str(verbosity)]
    paths = os.pathsep.join(
        filter(
            None,
            [
                os.environ.get("LD_LIBRARY_PATH", ""),
                os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
            ],
        )
    )
    from my_robot_description.runtime.diagnostic_compat import gazebo_compatibility_env

    return ExecuteProcess(
        cmd=["gz", "sim", "-r", *flags, str(world_file)],
        name="gazebo",
        output="screen",
        # Debug receipts must reach the runner during recording, not at pipe flush.
        emulate_tty=verbosity == 4,
        additional_env={
            "GZ_SIM_SYSTEM_PLUGIN_PATH": paths,
            **gazebo_compatibility_env(),
            **(extra_env or {}),
        },
    )


def make_clock_bridge():
    """Forward Gazebo simulation time into ROS /clock."""
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="simulation_clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )


def make_robot_spawner(spawn_height):
    """Create a free robot from the published description, above the ground."""
    return Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_biped",
        arguments=[
            "-world",
            WORLD_NAME,
            "-topic",
            "robot_description",
            "-name",
            ROBOT_NAME,
            "-z",
            str(spawn_height),
        ],
        output="screen",
    )
