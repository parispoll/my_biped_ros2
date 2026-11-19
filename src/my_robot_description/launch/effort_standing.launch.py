"""Separate effort posture experiment; the position-controlled launch is unchanged."""

from pathlib import Path
import os
import math
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory, get_package_prefix
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
from my_robot_description.model.stance import model_from_urdf
from my_robot_description.runtime.simulation import (
    make_simulator,
    make_clock_bridge,
    make_robot_spawner,
)
from my_robot_description.runtime.sensors import make_imu_bridge, make_contact_bridge
from my_robot_description.runtime.controllers import make_controller_spawner
from my_robot_description.runtime.startup import after_success, stop_on_exit


def setup(context):
    setting = lambda name: LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory("my_robot_description"))
    visual_keep_alive = setting("visual_keep_alive") == "true"

    def process_exit(name):
        if visual_keep_alive and name in (
            "posture",
            "contact-sampler",
            "balance-observer",
        ):
            return [
                LogInfo(
                    msg=f"[visual:controller-stopped] {name} exited; Gazebo remains open for fall observation. Inspect launch.log for the cause."
                )
            ]
        return stop_on_exit(name)

    if setting("balance_enabled") == "true" and (
        setting("controller_mode") != "pid" or setting("damping_model") != "explicit"
    ):
        raise ValueError("Active balance requires explicit damping and PID posture")
    balance_kd = float(setting("balance_kd"))
    if balance_kd not in (4.0, 8.0):
        raise ValueError("Supported body pitch-rate gains are 4 and 8")
    ankle_balance_limit = float(setting("ankle_balance_limit"))
    if ankle_balance_limit not in (0.5, 1.5):
        raise ValueError("Supported ankle balance caps are 0.5 and 1.5 N m")
    knee = float(setting("knee_bend"))
    if not 0.05 <= knee <= 0.6:
        raise ValueError(
            "This effort-standing experiment supports knee_bend 0.05–0.6 rad"
        )
    xml, temporary = prepare_description(
        share / "urdf/effort_standing.urdf.xacro",
        share / "config/effort_standing.yaml",
        initial_knee=knee,
        joint_friction=float(setting("joint_friction")),
        damping_model=setting("damping_model"),
    )
    model = model_from_urdf(xml)
    height = model.geometry(model.pose(knee))["expected_pelvis_height"]
    world = share / "worlds/standing.sdf"
    factor = float(setting("real_time_factor"))
    step = float(setting("physics_step"))
    if not math.isfinite(factor) or not 0.25 <= factor <= 1:
        raise ValueError("Diagnostic real_time_factor must be 0.25–1")
    if step not in (0.001, 0.0005, 0.00025):
        raise ValueError("This timestep comparison supports 0.001, 0.0005 or 0.00025 s")
    com_visual = setting("com_visual") == "true"
    copy_world = (
        bool(setting("physics_recording")) or factor != 1 or step != 0.001 or com_visual
    )
    if copy_world:
        tree = ET.parse(world)
        tree.getroot().find("world/physics/real_time_factor").text = str(factor)
        tree.getroot().find("world/physics/max_step_size").text = str(step)
    if setting("physics_recording"):
        path = Path(setting("physics_recording")).resolve()
        if path.exists():
            raise FileExistsError(f"Physics recording already exists: {path}")
        path.parent.mkdir(parents=True, exist_ok=True)
        plugin = ET.SubElement(
            tree.getroot().find("world"),
            "plugin",
            name="biped::PhysicsProbe",
            filename=str(
                Path(get_package_prefix("my_robot_description"))
                / "lib/libbiped_physics_probe.so"
            ),
        )
        ET.SubElement(plugin, "gz:system_priority").text = "100000"
        # A namespace declaration keeps this prefixed Gazebo extension valid XML.
        tree.getroot().set("xmlns:gz", "https://gazebosim.org/schemas")
        ET.SubElement(plugin, "output").text = str(path)
    if com_visual:
        ET.SubElement(
            tree.getroot().find("world"),
            "plugin",
            name="biped::ComProbe",
            filename=str(
                Path(get_package_prefix("my_robot_description"))
                / "lib/libbiped_com_probe.so"
            ),
        )
    if copy_world:
        world = Path(temporary.name) / "observed-world.sdf"
        tree.write(world, encoding="unicode")
    extra_env = {}
    if setting("dart_recording"):
        path = Path(setting("dart_recording")).resolve()
        if path.exists():
            raise FileExistsError(f"DART recording already exists: {path}")
        path.parent.mkdir(parents=True, exist_ok=True)
        library = (
            Path(get_package_prefix("my_robot_description"))
            / "lib/libbiped_dart_probe.so"
        )
        # LD_PRELOAD separates names on whitespace; use a whitespace-free symlink.
        preload = Path(temporary.name) / "libbiped_dart_probe.so"
        preload.symlink_to(library)
        extra_env = {
            "LD_PRELOAD": str(preload)
            + (":" + os.environ["LD_PRELOAD"] if os.environ.get("LD_PRELOAD") else ""),
            "BIPED_DART_PROBE_OUTPUT": str(path),
        }
    if setting("damping_recording"):
        path = Path(setting("damping_recording")).resolve()
        if path.exists():
            raise FileExistsError(f"Damping recording already exists: {path}")
        path.parent.mkdir(parents=True, exist_ok=True)
        extra_env["BIPED_DAMPING_LOG"] = str(path)
    simulator = make_simulator(
        world,
        setting("gui") == "true",
        verbosity=int(setting("verbosity")),
        extra_env=extra_env,
    )
    clock = make_clock_bridge()
    bridges = [
        make_imu_bridge(),
        make_contact_bridge("left"),
        make_contact_bridge("right"),
    ]
    description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": xml, "use_sim_time": True}],
        output="screen",
    )
    # Place flat feet at ground level, avoiding a falling/drop transient at startup.
    spawn = make_robot_spawner(height)
    states = make_controller_spawner("joint_state_broadcaster", 60)
    effort = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "standing_effort_controller",
            "--inactive",
            "--controller-manager-timeout",
            "60",
        ],
    )
    hold = Node(
        package="my_robot_description",
        executable="hold_effort_stance.py",
        output="screen",
        parameters=[
            str(Path(temporary.name) / "controllers.yaml"),
            str(share / "config/balance_control.yaml"),
            {
                "robot_description": xml,
                "use_sim_time": True,
                "knee_bend": knee,
                "controller_mode": setting("controller_mode"),
                "visual_fast_ready": visual_keep_alive
                and setting("fast_visual_ready") == "true",
                "balance_enabled": setting("balance_enabled") == "true",
                "balance_limit": ankle_balance_limit,
                "balance_kd": balance_kd,
                "recording": setting("recording"),
            },
        ],
    )

    def cleanup(context):
        temporary.cleanup()
        return []

    handlers = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))
    ]
    observers = []
    if com_visual:
        com = Node(
            package="my_robot_description",
            executable="show_center_of_mass.py",
            output="screen",
            parameters=[{"use_sim_time": True, "recording": setting("com_recording")}],
        )
        observers.append(com)
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=com,
                    on_exit=[
                        LogInfo(
                            msg="[com:stopped] Optional COM visual stopped; inspect its log."
                        )
                    ],
                )
            )
        )
    if setting("balance_enabled") == "true":
        sampler = Node(
            package="my_robot_description",
            executable="sample_contacts.py",
            parameters=[{"use_sim_time": True}],
            output="screen",
        )
        observers.append(sampler)
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=sampler, on_exit=process_exit("contact-sampler")
                )
            )
        )
    if setting("balance_recording"):
        observer = Node(
            package="my_robot_description",
            executable="observe_balance.py",
            output="screen",
            parameters=[
                str(share / "config/balance_observer.yaml"),
                {"use_sim_time": True, "recording": setting("balance_recording")},
            ],
        )
        observers.append(observer)
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=observer, on_exit=process_exit("balance-observer")
                )
            )
        )
    for name, action in [
        ("gazebo", simulator),
        ("clock", clock),
        ("description", description),
        ("posture", hold),
        *[(f"bridge-{i}", b) for i, b in enumerate(bridges)],
    ]:
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(target_action=action, on_exit=process_exit(name))
            )
        )
    for name, action, following in [
        ("spawn", spawn, [states]),
        ("states", states, [effort]),
        ("configure-effort", effort, [hold]),
    ]:
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=action, on_exit=after_success(name, following)
                )
            )
        )
    # Start the IMU observer with the posture node, after sensor bridges exist.
    if observers:
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=effort,
                    on_exit=after_success("start-balance-observer", observers),
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
                "com_visual", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument("com_recording", default_value=""),
            DeclareLaunchArgument(
                "fast_visual_ready", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "visual_keep_alive", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "verbosity", default_value="1", choices=["0", "1", "2", "3", "4"]
            ),
            DeclareLaunchArgument("knee_bend", default_value="0.4"),
            DeclareLaunchArgument("joint_friction", default_value="0.2"),
            DeclareLaunchArgument(
                "damping_model",
                default_value="implicit",
                choices=["implicit", "explicit"],
            ),
            DeclareLaunchArgument(
                "controller_mode", default_value="pd", choices=["pd", "pid"]
            ),
            DeclareLaunchArgument("damping_recording", default_value=""),
            DeclareLaunchArgument("recording", default_value=""),
            DeclareLaunchArgument("balance_recording", default_value=""),
            DeclareLaunchArgument(
                "balance_kd", default_value="4.0", choices=["4.0", "8.0"]
            ),
            DeclareLaunchArgument(
                "ankle_balance_limit", default_value="0.5", choices=["0.5", "1.5"]
            ),
            DeclareLaunchArgument(
                "balance_enabled", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument("physics_recording", default_value=""),
            DeclareLaunchArgument("dart_recording", default_value=""),
            DeclareLaunchArgument("real_time_factor", default_value="1"),
            DeclareLaunchArgument("physics_step", default_value="0.001"),
            OpaqueFunction(function=setup),
        ]
    )
