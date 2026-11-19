#!/usr/bin/env python3
"""Bounded pose check; use the same GZ_PARTITION as the simulator."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import math
import os
import re
import selectors
import subprocess
import time


def field(text, key, default=0.0):
    found = re.search(r"\b" + key + r": ([\deE+.-]+)", text)
    return float(found[1]) if found else default


def parse_pose(message):
    match = re.search(r'name: "my_biped"(.*?)(?=\npose \{|\Z)', message, re.S)
    stamp = re.search(r"stamp \{(.*?)\}", message, re.S)
    if not match or not stamp:
        return None
    position = re.search(r"position \{(.*?)\}", match[1], re.S)
    orientation = re.search(r"orientation \{(.*?)\}", match[1], re.S)
    if not position or not orientation:
        raise ValueError("Robot pose is missing position or orientation")
    x, y, z, w = [field(orientation[1], key) for key in ("x", "y", "z", "w")]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("Invalid orientation quaternion")
    x, y = x / norm, y / norm
    sample = {
        "sim_time": field(stamp[1], "sec") + field(stamp[1], "nsec") / 1e9,
        "height": field(position[1], "z"),
        "tilt_radians": math.acos(max(-1, min(1, 1 - 2 * (x * x + y * y)))),
    }
    if not all(math.isfinite(value) for value in sample.values()):
        raise ValueError("Non-finite robot pose")
    return sample


def measure(args):
    process = subprocess.Popen(
        ["gz", "topic", "-e", "-t", "/world/standing/pose/info"], stdout=subprocess.PIPE
    )
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    buffer = b""
    samples = []
    deadline = time.monotonic() + args.wall_timeout
    try:
        while time.monotonic() < deadline:
            if not selector.select(timeout=0.5):
                continue
            chunk = os.read(process.stdout.fileno(), 65536)
            if not chunk:
                raise RuntimeError("Gazebo pose stream ended")
            buffer += chunk
            while b"\n\n" in buffer:
                block, buffer = buffer.split(b"\n\n", 1)
                sample = parse_pose(block.decode())
                if sample is None:
                    continue
                if samples and sample["sim_time"] < samples[-1]["sim_time"]:
                    raise RuntimeError("Simulation time moved backwards")
                samples.append(sample)
                if (
                    sample["height"] < args.min_height
                    or sample["tilt_radians"] > args.max_tilt
                ):
                    raise RuntimeError(f"Robot fell or tilted: {sample}")
                if (
                    args.expected_height is not None
                    and abs(sample["height"] - args.expected_height)
                    > args.height_tolerance
                ):
                    raise RuntimeError(f"Height differs from prediction: {sample}")
                if sample["sim_time"] - samples[0]["sim_time"] >= args.duration:
                    return {
                        "passed": True,
                        "samples": len(samples),
                        "first": samples[0],
                        "last": samples[-1],
                        "min_height": min(s["height"] for s in samples),
                        "max_height": max(s["height"] for s in samples),
                        "max_tilt_radians": max(s["tilt_radians"] for s in samples),
                        "expected_height": args.expected_height,
                    }
        raise TimeoutError(
            "No complete measurement before wall-clock deadline; inspect Gazebo and GZ_PARTITION"
        )
    finally:
        selector.close()
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        process.stdout.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30)
    parser.add_argument("--wall-timeout", type=float, default=90)
    parser.add_argument("--min-height", type=float, default=0.5)
    parser.add_argument("--max-tilt", type=float, default=0.15)
    parser.add_argument("--expected-height", type=float)
    parser.add_argument("--height-tolerance", type=float, default=0.01)
    args = parser.parse_args()
    if any(
        not math.isfinite(value) for value in vars(args).values() if value is not None
    ):
        parser.error("All thresholds must be finite")
    if min(args.duration, args.wall_timeout, args.height_tolerance, args.max_tilt) <= 0:
        parser.error("Duration, timeout and tolerances must be positive")
    print(json.dumps(measure(args), indent=2))


if __name__ == "__main__":
    main()
