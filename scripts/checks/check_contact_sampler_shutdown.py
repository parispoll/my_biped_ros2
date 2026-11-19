#!/usr/bin/env python3
"""Stress the real ROS sampler under traffic and alternating SIGINT/SIGTERM.

No Gazebo or motor commands. Use a separate ROS_DOMAIN_ID from experiments.
"""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node("contact_sampler_shutdown_check")
    process = None
    results = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    pubs = {
        s: node.create_publisher(Contacts, f"/biped/{s}_foot/contacts", qos)
        for s in ["left", "right"]
    }
    seen = {s: 0 for s in pubs}
    published = set()
    errors = []

    def receive(msg, side):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp not in published:
            errors.append("Forwarded stamp was not an original published stamp")
        seen[side] += 1

    subs = [
        node.create_subscription(
            Contacts,
            f"/biped/balance/{s}_foot/contacts",
            lambda m, s=s: receive(m, s),
            qos,
        )
        for s in pubs
    ]

    def traffic():
        msg = Contacts()
        msg.header.stamp = node.get_clock().now().to_msg()
        published.add((msg.header.stamp.sec, msg.header.stamp.nanosec))
        for pub in pubs.values():
            pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.001)

    try:
        for i, sig in enumerate([signal.SIGINT, signal.SIGTERM] * 3):
            seen.update(left=0, right=0)
            with (args.output / f"run-{i+1}.log").open("w") as log:
                process = subprocess.Popen(
                    [
                        sys.executable,
                        str(
                            ROOT
                            / "src/my_robot_description/nodes/sensing/sample_contacts.py"
                        ),
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
                deadline = time.monotonic() + 12
                while min(seen.values()) < 20:
                    if process.poll() is not None:
                        raise RuntimeError(
                            "Sampler exited before traffic was forwarded"
                        )
                    if time.monotonic() > deadline:
                        raise TimeoutError("Sampler did not forward both feet")
                    traffic()
                before = dict(seen)
                process.send_signal(sig)
                deadline = time.monotonic() + 5
                while process.poll() is None:
                    if time.monotonic() > deadline:
                        raise TimeoutError("Sampler did not stop promptly")
                    traffic()
                code = process.wait()
                text = (args.output / f"run-{i+1}.log").read_text()
                ok = code == 0 and "Traceback" not in text and not errors
                results.append(
                    dict(
                        signal=sig.name,
                        exit_code=code,
                        forwarded_before_signal=before,
                        passed=ok,
                    )
                )
                if not ok:
                    raise RuntimeError("Sampler shutdown/forwarding check failed")
    finally:
        if process and process.poll() is None:
            process.kill()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()
        report = dict(
            passed=len(results) == 6
            and all(r["passed"] for r in results)
            and not errors,
            runs=results,
            stamp_errors=errors,
            note="Real ROS traffic and process signals; no Gazebo or robot motor commands.",
        )
        (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
