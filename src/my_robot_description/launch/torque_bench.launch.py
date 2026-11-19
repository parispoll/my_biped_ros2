"""A fixed-base single-joint effort bench; independent of standing.launch.py."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    LogInfo,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from my_robot_description.runtime.description import prepare_description
from my_robot_description.runtime.simulation import make_simulator, make_clock_bridge
from my_robot_description.runtime.controllers import make_controller_spawner
from my_robot_description.runtime.startup import after_success, stop_on_exit


def setup(context):
    share = Path(get_package_share_directory("my_robot_description"))
    xml, temporary = prepare_description(
        share / "urdf/torque_bench.urdf.xacro", share / "config/torque_bench.yaml"
    )
    gazebo = make_simulator(
        share / "worlds/standing.sdf",
        LaunchConfiguration("gui").perform(context) == "true",
    )
    clock = make_clock_bridge()
    publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": xml, "use_sim_time": True}],
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world",
            "standing",
            "-topic",
            "robot_description",
            "-name",
            "torque_bench",
        ],
        output="screen",
    )
    states = make_controller_spawner("joint_state_broadcaster", 60)
    effort = make_controller_spawner("bench_effort_controller", 60)

    def cleanup(context):
        temporary.cleanup()
        return []

    handlers = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))
    ]
    for name, action in [
        ("gazebo", gazebo),
        ("clock", clock),
        ("description", publisher),
    ]:
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(target_action=action, on_exit=stop_on_exit(name))
            )
        )
    for name, action, following in [
        ("bench-spawn", spawn, [states]),
        ("bench-states", states, [effort]),
        (
            "bench-effort",
            effort,
            [LogInfo(msg="[bench:ready] Effort interface active.")],
        ),
    ]:
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=action, on_exit=after_success(name, following)
                )
            )
        )
    return [*handlers, gazebo, publisher, clock, spawn]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui", default_value="false", choices=["true", "false"]
            ),
            OpaqueFunction(function=setup),
        ]
    )
