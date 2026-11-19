"""Startup ordering, input validation, and explicit failure handling."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from my_robot_description.runtime.description import prepare_description
from my_robot_description.runtime.simulation import (
    make_simulator,
    make_clock_bridge,
    make_robot_spawner,
)
from my_robot_description.runtime.sensors import make_imu_bridge, make_contact_bridge
from my_robot_description.runtime.controllers import (
    make_controller_spawner,
    STATE_BROADCASTER,
    TRAJECTORY_CONTROLLER,
)


def fail(stage, returncode):
    """Stop the launch rather than continue with a partially working robot."""
    reason = (
        f"[biped:{stage}] Process exited with code {returncode}; stopping simulation."
    )
    return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]


def after_success(stage, next_actions):
    """Create an exit callback that starts the next stage only after success."""

    def callback(event, context):
        if event.returncode != 0:
            return fail(stage, event.returncode)
        return [LogInfo(msg=f"[biped:{stage}] Complete."), *next_actions]

    return callback


def stop_on_exit(stage):
    """Essential long-running processes must stay alive during simulation."""

    def callback(event, context):
        if context.is_shutdown:
            return []
        return fail(stage, event.returncode)

    return callback


def launch_setup(context):
    """Wire independently defined components into a checked startup sequence."""
    share = Path(get_package_share_directory("my_robot_description"))

    def setting(name):
        return LaunchConfiguration(name).perform(context)

    gui_text = setting("gui").lower()
    if gui_text not in ("true", "false"):
        raise ValueError("[configuration] gui must be true or false")
    spawn_height = float(setting("spawn_height"))
    timeout = float(setting("controller_timeout"))
    if not 0 < spawn_height < 10 or not 0 < timeout <= 300:
        raise ValueError(
            "[configuration] spawn_height must be in (0, 10) m; timeout in (0, 300] s"
        )
    world = share / "worlds/standing.sdf"
    if not world.is_file():
        raise FileNotFoundError(f"[world] Missing world: {world}")
    description, temporary = prepare_description(
        share / "urdf/my_biped.urdf.xacro",
        share / "config/controllers.yaml",
        initial_knee=(
            float(setting("initial_knee_bend"))
            if setting("stance") == "crouch"
            else 0.0
        ),
    )
    gazebo = make_simulator(world, gui_text == "true")
    clock = make_clock_bridge()
    imu_bridge = make_imu_bridge()
    contact_bridges = {side: make_contact_bridge(side) for side in ("left", "right")}
    publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": description, "use_sim_time": True}],
    )
    spawn = make_robot_spawner(spawn_height)
    states = make_controller_spawner(STATE_BROADCASTER, timeout)
    trajectory = make_controller_spawner(TRAJECTORY_CONTROLLER, timeout)

    stance_actions = [LogInfo(msg="[biped:ready] Both controllers are active.")]
    if setting("stance") == "crouch":
        stance = Node(
            package="my_robot_description",
            executable="move_to_stance.py",
            output="screen",
            parameters=[
                {
                    "robot_description": description,
                    "use_sim_time": True,
                    "knee_bend": float(setting("knee_bend")),
                    "tracking_log": setting("tracking_log"),
                    "transition_duration": float(setting("transition_duration")),
                }
            ],
        )
        stance_actions.append(stance)
    else:
        stance = None

    def cleanup(context):
        temporary.cleanup()
        return []

    handlers = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))
    ]
    for stage, action in [
        ("gazebo", gazebo),
        ("clock", clock),
        ("description", publisher),
        ("imu-bridge", imu_bridge),
        *[
            (f"{side}-contact-bridge", bridge)
            for side, bridge in contact_bridges.items()
        ],
    ]:
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(target_action=action, on_exit=stop_on_exit(stage))
            )
        )
    for stage, action, following in [
        ("spawn", spawn, [states]),
        ("joint-states", states, [trajectory]),
        ("trajectory", trajectory, stance_actions),
    ]:
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=action, on_exit=after_success(stage, following)
                )
            )
        )
    if stance is not None:
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=stance,
                    on_exit=after_success(
                        "stance",
                        [
                            LogInfo(
                                msg="[biped:standing] Crouch trajectory complete; verify body pose separately."
                            )
                        ],
                    ),
                )
            )
        )
    # Register handlers before starting any process so fast exits cannot be missed.
    return [
        *handlers,
        LogInfo(msg="[biped:start] Starting Gazebo and robot description."),
        gazebo,
        publisher,
        clock,
        imu_bridge,
        *contact_bridges.values(),
        spawn,
    ]
