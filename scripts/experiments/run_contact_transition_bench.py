#!/usr/bin/env python3
"""Test actual Gazebo -> ROS bridge -> existing sampler contact/air/contact.

A 1 kg box represents a sensor fixture, not the humanoid. No robot motors.
"""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse, json, os, signal, subprocess, sys, time, uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


def world(path, state_source):
    tree = ET.parse(ROOT / "src/my_robot_description/worlds/standing.sdf")
    w = tree.getroot().find("world")
    w.set("name", "contact_bench")
    w.find("physics/real_time_factor").text = ".5"
    if state_source:
        from ament_index_python.packages import get_package_prefix

        ET.SubElement(
            w,
            "plugin",
            name="biped::ContactStateProbe",
            filename=str(
                Path(get_package_prefix("my_robot_description"))
                / "lib/libbiped_contact_state_probe.so"
            ),
        )
    model = ET.fromstring(
        """<model name="my_biped"><pose>0 0 0.025 0 0 0</pose><link name="left_foot"><inertial><mass>1</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial><collision name="left_foot_collision"><geometry><box><size>0.1 0.1 0.05</size></box></geometry></collision><visual name="box"><geometry><box><size>0.1 0.1 0.05</size></box></geometry></visual><sensor name="contact" type="contact"><always_on>true</always_on><contact><collision>left_foot_collision</collision><topic>/biped/left_foot/contacts</topic></contact></sensor></link></model>"""
    )
    w.append(model)
    tree.write(path, encoding="unicode")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source", choices=["raw", "state"], default="state")
    args = parser.parse_args()
    folder = args.output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    world(folder / "world.sdf", args.source == "state")
    os.environ.update(
        ROS_DOMAIN_ID="83",
        GZ_PARTITION="contact_bench_" + uuid.uuid4().hex,
        ROS_LOG_DIR=str(ROOT / "log/runtime"),
    )
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from ros_gz_interfaces.msg import Contacts
    from rosidl_runtime_py.convert import message_to_ordereddict
    from my_robot_description.sensing.contact_state import contact_state

    rclpy.init()
    node = Node(
        "contact_transition_bench",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    processes = []
    streams = []
    latest = None
    raw_latest = None
    rows = []
    raw_rows = []
    events = []
    passed = False
    error = None
    output = (folder / "states.jsonl").open("x", buffering=1)

    def receive(msg):
        nonlocal latest
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        latest = dict(side="left", sim_time=t, message=message_to_ordereddict(msg))

    def receive_raw(msg):
        nonlocal raw_latest
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        raw_latest = dict(side="left", sim_time=t, message=message_to_ordereddict(msg))

    sub = node.create_subscription(
        Contacts,
        (
            "/biped/state/balance/left_foot/contacts"
            if args.source == "state"
            else "/biped/balance/left_foot/contacts"
        ),
        receive,
        qos_profile_sensor_data,
    )
    raw_sub = node.create_subscription(
        Contacts, "/biped/left_foot/contacts", receive_raw, qos_profile_sensor_data
    )

    def start(name, cmd):
        stream = (folder / (name + ".log")).open("w")
        streams.append(stream)
        p = subprocess.Popen(
            cmd,
            cwd=ROOT,
            stdout=stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        processes.append(p)

    last = None
    last_raw = None

    def tick():
        nonlocal last, last_raw
        rclpy.spin_once(node, timeout_sec=0.005)
        now = node.get_clock().now().nanoseconds / 1e9
        state = contact_state(latest, now, "left")
        state["now"] = now
        # Sample at most once per clock step, including stale states during silence.
        if now != last and now > 0:
            rows.append(state)
            output.write(json.dumps(state) + "\n")
            last = now
        if raw_latest is not None and raw_latest["sim_time"] != last_raw:
            raw_rows.append(contact_state(raw_latest, now, "left"))
            last_raw = raw_latest["sim_time"]
        if any(p.poll() is not None for p in processes):
            raise RuntimeError("Bench process exited unexpectedly")
        return now, state

    def until(predicate, timeout=20):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            now, state = tick()
            if predicate(now, state):
                return now
        raise TimeoutError("Contact transition criterion not observed")

    try:
        start(
            "bridge",
            [
                "ros2",
                "run",
                "ros_gz_bridge",
                "parameter_bridge",
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/biped/left_foot/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                *(
                    [
                        "/biped/state/left_foot/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
                    ]
                    if args.source == "state"
                    else []
                ),
            ],
        )
        start(
            "sampler",
            [
                sys.executable,
                str(ROOT / "src/my_robot_description/nodes/sensing/sample_contacts.py"),
                *(
                    [
                        "--ros-args",
                        "-r",
                        "/biped/left_foot/contacts:=/biped/state/left_foot/contacts",
                        "-r",
                        "/biped/balance/left_foot/contacts:=/biped/state/balance/left_foot/contacts",
                    ]
                    if args.source == "state"
                    else []
                ),
            ],
        )
        start("gazebo", ["gz", "sim", "-r", "-s", str(folder / "world.sdf")])
        t = until(lambda now, s: s["state"] == "loaded_contact" and now > 1.0)
        until(lambda now, s: now >= t + 0.5 and s["state"] == "loaded_contact")
        for cycle in range(3):
            before = len(rows)
            events.append(
                dict(
                    event="lift_requested",
                    cycle=cycle + 1,
                    sim_time=node.get_clock().now().nanoseconds / 1e9,
                )
            )
            cmd = [
                "gz",
                "service",
                "-s",
                "/world/contact_bench/set_pose",
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "2000",
                "--req",
                'name: "my_biped", position: {x: 0, y: 0, z: 1}',
            ]
            completed = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            (folder / f"pose-{cycle+1}.log").write_text(
                completed.stdout + completed.stderr
            )
            if completed.returncode or "true" not in completed.stdout:
                raise RuntimeError("Gazebo rejected fixture relocation")
            t = until(lambda now, s: s["state"] == "no_contact", timeout=5)
            events.append(
                dict(event="fresh_no_contact_observed", cycle=cycle + 1, sim_time=t)
            )
            until(
                lambda now, s: now >= t + 0.08 and s["state"] == "no_contact", timeout=5
            )
            t = until(
                lambda now, s: s["state"] == "loaded_contact" and now > t + 0.15,
                timeout=6,
            )
            until(
                lambda now, s: now >= t + 0.5 and s["state"] == "loaded_contact",
                timeout=6,
            )
            events.append(
                dict(
                    event="loaded_contact_restored",
                    cycle=cycle + 1,
                    sim_time=node.get_clock().now().nanoseconds / 1e9,
                )
            )
        passed = True
    except Exception as exc:
        error = str(exc)
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
                try:
                    p.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)
                    p.wait()
        output.close()
        for stream in streams:
            stream.close()
        node.destroy_node()
        rclpy.try_shutdown()
        counts = {
            state: sum(r["state"] == state for r in rows)
            for state in [
                "loaded_contact",
                "touching_unloaded",
                "no_contact",
                "stale",
                "unknown",
                "invalid",
            ]
        }
        raw_counts = {
            state: sum(r["state"] == state for r in raw_rows) for state in counts
        }
        report = dict(
            completed=True,
            passed=passed,
            error=error,
            bench_kind="contact_transition",
            robot_motor_commands_sent=False,
            contact_source=args.source,
            events=events,
            counts=counts,
            raw_counts=raw_counts,
            raw_source_samples=len(raw_rows),
            note="Sensor fixture only; does not validate humanoid zero-load balance.",
        )
        (folder / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        labels = [
            "unknown",
            "invalid",
            "stale",
            "no_contact",
            "touching_unloaded",
            "loaded_contact",
        ]
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.step(
            [r["now"] for r in rows],
            [labels.index(r["state"]) for r in rows],
            where="post",
        )
        ax.set_yticks(range(len(labels)), labels)
        ax.set_xlabel("Simulation time (s)")
        ax.set_title(
            "Contact sensor transition bench — "
            + ("passed" if passed else "not passed")
        )
        ax.grid(alpha=0.2)
        fig.tight_layout()
        fig.savefig(folder / "response.png", dpi=150)
        plt.close(fig)
    print(json.dumps(report, indent=2))
    if not passed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
