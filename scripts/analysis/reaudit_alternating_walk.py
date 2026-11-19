#!/usr/bin/env python3
"""Read-only reanalysis of a physically completed walking recording.

Preserves the original failed report. Only the documented 30 ms integer-stamp
pairing correction may differ from the package sources used for measurement.
"""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse, hashlib, json, os

ROOT = Path(__file__).resolve().parents[2]
FIX = """# ROS stamps have integer-nanosecond resolution. Floating subtraction
        # can turn exactly 30 ms into 0.030000000000001 s. Preserve the
        # same inclusive 30 ms boundary without accepting even 1 ns more.
        if round(t*1e9)-round(right['sim_time']*1e9)>30_000_000:raise ValueError('Stale paired contact state')"""
OLD = "if t-right['sim_time']>.03:raise ValueError('Stale paired contact state')"


def verify_runtime_sources(recorded):
    changed = []
    for name, expected in recorded.items():
        if not (ROOT / name).is_file():
            raise ValueError(
                "This recording uses an earlier source layout. Preserve its report "
                "and use the archived source snapshot documented in docs/REORGANIZATION.md."
            )
        data = (ROOT / name).read_bytes()
        if hashlib.sha256(data).hexdigest() == expected:
            continue
        if name != "src/my_robot_description/my_robot_description/motion/zero_load.py":
            raise ValueError("Unexpected post-run package change: " + name)
        text = data.decode()
        if (
            text.count(FIX) != 1
            or hashlib.sha256(text.replace(FIX, OLD).encode()).hexdigest() != expected
        ):
            raise ValueError(
                "Only the precise integer timestamp pairing fix is permitted"
            )
        changed.append(
            dict(
                path=name,
                recorded_sha256=expected,
                analysis_sha256=hashlib.sha256(data).hexdigest(),
                change="Inclusive 30 ms comparison in integer nanoseconds; live control functions unchanged",
            )
        )
    return changed


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    source = a.input.resolve()
    folder = a.output.resolve()
    if source.parent != ROOT / "evidence/gui":
        raise ValueError("Input must be a saved GUI recording")
    original = json.loads((source / "summary.json").read_text())
    if (
        not original.get("writers_closed_before_audit")
        or "[shift:done]" not in (source / "launch.log").read_text()
    ):
        raise ValueError(
            "Physical walking program must have finished and writers must be closed"
        )
    recorded = json.loads((source / "source-hashes.json").read_text())
    changes = verify_runtime_sources(recorded)
    from scripts.studies.run_duration_comparison import source_snapshot

    before = source_snapshot()
    inputs = {
        p.name: hashlib.sha256(p.read_bytes()).hexdigest()
        for p in source.iterdir()
        if p.is_file() and p.suffix in (".json", ".jsonl", ".csv", ".log")
    }
    folder.mkdir(parents=True, exist_ok=False)
    (folder / "original-summary.json").write_text(json.dumps(original, indent=2) + "\n")
    for name in inputs:
        if name not in ("summary.json", "response.png"):
            (folder / name).symlink_to(source / name)
    os.environ["BIPED_ACTUATOR_PROFILE"] = "robotis_p54"
    import xacro
    from my_robot_description.motion.alternating_walk import AlternatingWalkProgram
    from my_robot_description.validation.alternating_walk_audit import (
        audit_alternating_walk,
    )

    program = AlternatingWalkProgram(
        xacro.process_file(
            str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
        ).toxml(),
        original["motion_plan"]["supporting_foot"],
    )
    read = lambda n: [json.loads(l) for l in (source / n).read_text().splitlines()]
    audit = audit_alternating_walk(
        source, read("joints.jsonl"), read("com.jsonl"), program
    )
    if before != source_snapshot():
        raise ValueError("Sources changed during reanalysis")
    if any(
        hashlib.sha256((source / name).read_bytes()).hexdigest() != digest
        for name, digest in inputs.items()
    ):
        raise ValueError("Original recording changed during read-only reanalysis")
    result = dict(original)
    result.pop("error", None)
    result.update(
        audit,
        completed=True,
        source_unchanged=False,
        reanalysis_only=True,
        recording_source_job=source.name,
        analysis_source_changes=changes,
        runtime_sources_match_except_documented_audit_fix=True,
        source_unchanged_during_reanalysis=True,
        original_recording_unchanged=True,
        original_failure=original.get("error"),
        original_input_sha256=inputs,
    )
    (folder / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    (folder / "analysis-source-hashes.json").write_text(
        json.dumps(before, indent=2) + "\n"
    )
    from scripts.plots.plot_alternating_walk import plot

    plot(folder)
    print(json.dumps(result, indent=2))
    if not result["passed"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
