#!/usr/bin/env python3
"""Repeat one fixed pulse in fresh measured worlds, including guard outcomes."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import signal
import subprocess
from my_robot_description.experiments.effort_push import EffortPushConfig
from scripts.studies.run_duration_comparison import ROOT, source_snapshot, compare
from scripts.studies.run_force_limit_study import classify


def assess(summary, code, launch_log, events):
    outcome = classify(summary, code)
    if outcome != "incomplete_requires_diagnosis":
        return outcome
    reference = summary.get("reference", {})
    published = any(e.get("event") == "push_published" for e in events)
    guards = (
        "Pelvis tilt exceeded 0.15 rad",
        "Posture exceeded this stationary experiment",
    )
    if reference.get("passed") and published and any(g in launch_log for g in guards):
        return "posture_guard_stopped_partial_trial"
    return outcome


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--force", type=int, choices=range(1, 101), default=25)
    parser.add_argument("--duration", type=float, default=0.2)
    parser.add_argument("--ros-domain-id", type=int, choices=range(101), default=69)
    args = parser.parse_args()
    EffortPushConfig(force_x=args.force, duration=args.duration).validate()
    folder = args.output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    snapshot = source_snapshot()
    report = dict(
        completed=False,
        source_hashes=snapshot,
        force_n=args.force,
        duration_s=args.duration,
        cases=[],
        note="Two fresh reference/push pairs per direction; reverse direction order on repeat 2. Guard stops remain partial trials, not completed recovery measurements.",
    )

    def save():
        (folder / "repeat-study.json").write_text(json.dumps(report, indent=2) + "\n")

    save()
    try:
        for repeat, directions in (
            (1, ("forward", "backward")),
            (2, ("backward", "forward")),
        ):
            for direction in directions:
                if source_snapshot() != snapshot:
                    raise RuntimeError("Robot sources changed")
                name = f"{direction}-repeat-{repeat}"
                case = folder / name
                force = args.force * (1 if direction == "forward" else -1)
                command = [
                    sys.executable,
                    str(ROOT / "scripts/experiments/run_effort_push.py"),
                    "--force-x",
                    str(force),
                    "--duration",
                    str(args.duration),
                    "--balance-enabled",
                    "--damping-model",
                    "explicit",
                    "--controller-mode",
                    "pid",
                    "--real-time-factor",
                    ".25",
                    "--ros-domain-id",
                    str(args.ros_domain_id),
                    "--output",
                    str(case),
                ]
                print("[repeat] Starting " + name, flush=True)
                with (folder / f"{name}.log").open("w") as log:
                    process = subprocess.Popen(
                        command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT
                    )
                    try:
                        code = process.wait()
                    except BaseException:
                        process.send_signal(signal.SIGINT)
                        try:
                            process.wait(timeout=25)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait()
                        raise
                summary = (
                    json.loads((case / "summary.json").read_text())
                    if (case / "summary.json").exists()
                    else {}
                )
                launch = (
                    (case / "push/launch.log").read_text()
                    if (case / "push/launch.log").exists()
                    else ""
                )
                events = (
                    json.loads((case / "push/recording/events.json").read_text())
                    if (case / "push/recording/events.json").exists()
                    else []
                )
                outcome = assess(summary, code, launch, events)
                row = dict(
                    name=name,
                    repeat=repeat,
                    direction=direction,
                    path=str(case),
                    force_x_n=force,
                    returncode=code,
                    outcome=outcome,
                    events=events,
                )
                if summary.get("completed"):
                    row["response"] = compare([(args.duration, summary)])[0]
                else:
                    row["error"] = summary.get("error", "Missing summary")
                report["cases"].append(row)
                save()
                print("[repeat] " + json.dumps(row), flush=True)
                if outcome == "incomplete_requires_diagnosis":
                    report["outcome"] = outcome
                    save()
                    return
        if source_snapshot() != snapshot:
            raise RuntimeError("Robot sources changed during final case")
        report.update(completed=True, outcome="fixed_pulse_repeats_finished")
        save()
    except BaseException as error:
        report["error"] = str(error)
        save()
        raise


if __name__ == "__main__":
    main()
