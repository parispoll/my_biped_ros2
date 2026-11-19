"""Launch the ten-joint robot, effort controller, sensing and selected motion.

Standing, transfers and walking share startup; mutually exclusive parameters
select the motion. Installed ROS executable names stay stable after refactoring."""

from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from my_robot_description.runtime.roll_description import prepare_roll_description
from my_robot_description.runtime.simulation import (
    make_simulator,
    make_clock_bridge,
    make_robot_spawner,
)
from my_robot_description.runtime.sensors import make_imu_bridge, make_contact_bridge
from my_robot_description.runtime.controllers import make_controller_spawner
from my_robot_description.runtime.startup import after_success, stop_on_exit


def setup(context):
    setting = lambda n: LaunchConfiguration(n).perform(context)
    share = Path(get_package_share_directory("my_robot_description"))
    prefix = Path(get_package_prefix("my_robot_description"))
    xml, temporary, plan = prepare_roll_description(share, prefix)
    walking = setting("alternating_walk_enabled") == "true"
    step = setting("first_step_enabled") == "true" or walking
    lift = setting("foot_lift_enabled") == "true" or step
    zero = setting("zero_load_enabled") == "true" or lift
    world = ET.parse(share / "worlds/standing.sdf")
    world.getroot().find("world/physics/real_time_factor").text = setting(
        "real_time_factor"
    )
    ET.SubElement(
        world.getroot().find("world"),
        "plugin",
        name="biped::ComProbe",
        filename=str(prefix / "lib/libbiped_com_probe.so"),
    )
    if zero:
        ET.SubElement(
            world.getroot().find("world"),
            "plugin",
            name="biped::ContactStateProbe",
            filename=str(prefix / "lib/libbiped_contact_state_probe.so"),
        )
    path = Path(temporary.name) / "world.sdf"
    world.write(path, encoding="unicode")
    env = {}
    if setting("damping_recording"):
        env["BIPED_DAMPING_LOG"] = str(Path(setting("damping_recording")).resolve())
    simulator = make_simulator(
        path,
        setting("gui") == "true",
        verbosity=4 if setting("wrench_debug") == "true" else 1,
        extra_env=env,
        camera="side" if step else "front",
    )
    description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": xml, "use_sim_time": True}],
        output="screen",
    )
    clock = make_clock_bridge()
    bridges = [
        make_imu_bridge(),
        make_contact_bridge("left"),
        make_contact_bridge("right"),
    ]
    if zero:
        bridges += [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    f"/biped/state/{side}_foot/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
            for side in ["left", "right"]
        ]
    spawn = make_robot_spawner(plan["expected_pelvis_height_m"])
    states = make_controller_spawner("joint_state_broadcaster", 60)
    effort = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "roll_effort_controller",
            "--inactive",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )
    hold = Node(
        package="my_robot_description",
        executable="hold_roll_stance.py",
        parameters=[
            {
                "robot_description": xml,
                "use_sim_time": True,
                "interactive_enabled": setting("interactive_enabled") == "true",
                "recording": setting("recording"),
                "fault_report": setting("fault_report"),
                "shift_enabled": setting("shift_enabled") == "true",
                "shift_displacement": float(setting("shift_displacement")),
                "unload_enabled": setting("unload_enabled") == "true",
                "load_feedback_enabled": setting("load_feedback_enabled") == "true",
                "load_reference_bias": float(setting("load_reference_bias")),
                "lowering_enabled": setting("lowering_enabled") == "true",
                "lowering_distance": float(setting("lowering_distance")),
                "lowering_lateral_shift": float(setting("lowering_lateral_shift")),
                "lowered_unload_fraction": float(setting("lowered_unload_fraction")),
                "zero_load_enabled": zero,
                "foot_lift_enabled": lift,
                "foot_lift_clearance": float(setting("foot_lift_clearance")),
                "first_step_enabled": step,
                "alternating_walk_enabled": walking,
                "walk_steps": int(setting("walk_steps")),
                "walk_step_length": float(setting("walk_step_length")),
                "walk_time_scale": float(setting("walk_time_scale")),
                "walk_velocity_feedforward": setting("walk_velocity_feedforward")
                == "true",
                "walk_transfer_time_scale": float(setting("walk_transfer_time_scale")),
                "unload_fraction": float(setting("unload_fraction")),
                "unload_direction": setting("unload_direction"),
            }
        ],
        output="screen",
    )
    com = Node(
        package="my_robot_description",
        executable="show_center_of_mass.py",
        parameters=[{"use_sim_time": True, "recording": setting("com_recording")}],
        output="screen",
    )
    sampler = (
        Node(
            package="my_robot_description",
            executable="sample_contacts.py",
            parameters=[{"use_sim_time": True}],
            output="screen",
            remappings=(
                [
                    (
                        f"/biped/{side}_foot/contacts",
                        f"/biped/state/{side}_foot/contacts",
                    )
                    for side in ["left", "right"]
                ]
                + [
                    (
                        f"/biped/balance/{side}_foot/contacts",
                        f"/biped/state/balance/{side}_foot/contacts",
                    )
                    for side in ["left", "right"]
                ]
                if zero
                else []
            ),
        )
        if any(
            setting(n) == "true"
            for n in ["shift_enabled", "unload_enabled", "lowering_enabled"]
        )
        and setting("interactive_enabled") != "true"
        else None
    )

    def cleanup(context):
        temporary.cleanup()
        return []

    handlers = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))
    ]
    for name, action in [
        ("gazebo", simulator),
        ("clock", clock),
        ("description", description),
        ("posture", hold),
        ("com", com),
        *[(f"bridge-{i}", b) for i, b in enumerate(bridges)],
    ]:
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(target_action=action, on_exit=stop_on_exit(name))
            )
        )
    if sampler:
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=sampler, on_exit=stop_on_exit("shift-contact-sampler")
                )
            )
        )
    for name, action, following in [
        ("spawn", spawn, [states]),
        ("states", states, [effort]),
        ("configure", effort, [hold, com, *([sampler] if sampler else [])]),
    ]:
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=action, on_exit=after_success(name, following)
                )
            )
        )
    return [*handlers, simulator, description, clock, *bridges, spawn]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "interactive_enabled", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument("recording", default_value=""),
            DeclareLaunchArgument("fault_report", default_value=""),
            DeclareLaunchArgument("com_recording", default_value=""),
            DeclareLaunchArgument("damping_recording", default_value=""),
            DeclareLaunchArgument(
                "wrench_debug", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "shift_enabled", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument("shift_displacement", default_value="0.01"),
            DeclareLaunchArgument(
                "unload_enabled", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "lowering_enabled", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument("lowering_distance", default_value="0.01"),
            DeclareLaunchArgument("lowering_lateral_shift", default_value="0.0"),
            DeclareLaunchArgument("lowered_unload_fraction", default_value="0.0"),
            DeclareLaunchArgument(
                "foot_lift_clearance",
                default_value="0.002",
                choices=["0.002", "0.01", "0.03"],
            ),
            DeclareLaunchArgument(
                "real_time_factor", default_value="0.25", choices=["0.25", "0.5", "1.0"]
            ),
            DeclareLaunchArgument("walk_steps", default_value="3"),
            DeclareLaunchArgument(
                "walk_velocity_feedforward",
                default_value="false",
                choices=["false", "true"],
            ),
            DeclareLaunchArgument(
                "walk_transfer_time_scale",
                default_value="0.0",
                choices=["0.0", "0.25", "0.375", "0.5", "1.0"],
            ),
            DeclareLaunchArgument(
                "walk_step_length",
                default_value="0.05",
                choices=["0.05", "0.075", "0.1"],
            ),
            DeclareLaunchArgument(
                "walk_time_scale",
                default_value="1.0",
                choices=["1.0", "0.5", "0.4", "0.25"],
            ),
            DeclareLaunchArgument(
                "alternating_walk_enabled",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "first_step_enabled", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "foot_lift_enabled", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "zero_load_enabled", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "load_feedback_enabled",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument("load_reference_bias", default_value="0.0"),
            DeclareLaunchArgument(
                "unload_fraction", default_value="0.35", choices=["0.4", "0.35", "0.3"]
            ),
            DeclareLaunchArgument(
                "unload_direction", default_value="left", choices=["left", "right"]
            ),
            OpaqueFunction(function=setup),
        ]
    )
