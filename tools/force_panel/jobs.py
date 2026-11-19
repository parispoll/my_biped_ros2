"""One owned experiment at a time, progress, cancellation and saved artifacts."""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
import uuid
from .config import ExperimentRequest

ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
TOKEN_ENV = "BIPED_FORCE_PANEL_JOB"
TERMINAL = {"completed", "failed", "stopped", "interrupted"}


def now():
    return datetime.now(timezone.utc).isoformat()


def desktop_environment(env):
    """A login-started server may predate desktop environment import.

    Refresh only display routing for a new GUI child, never unrelated manager
    variables or credentials. Keep the caller's environment if unavailable.
    """
    result = dict(env)
    try:
        response = subprocess.run(
            ["systemctl", "--user", "show-environment"],
            capture_output=True,
            text=True,
            timeout=2,
        )
    except (OSError, subprocess.TimeoutExpired):
        return result
    if response.returncode:
        return result
    for line in response.stdout.splitlines():
        key, sep, value = line.partition("=")
        if sep and key in ("DISPLAY", "WAYLAND_DISPLAY", "XAUTHORITY"):
            result[key] = value
    return result


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def tail(path, limit=14000):
    try:
        with path.open("rb") as stream:
            stream.seek(0, 2)
            size = stream.tell()
            stream.seek(max(0, size - limit))
            return ANSI.sub("", stream.read().decode("utf-8", errors="replace"))
    except OSError:
        return ""


def tagged_pids(token):
    """Read only the ownership marker; never return or log environment contents."""
    needle = f"{TOKEN_ENV}={token}".encode()
    owned = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if needle in (entry / "environ").read_bytes().split(b"\0"):
                owned.append(int(entry.name))
        except (OSError, PermissionError):
            pass
    return owned


def terminate_owned(token):
    # Last resort after the runner's own force-clear/simulator cleanup deadline.
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pid in tagged_pids(token):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        time.sleep(0.25)


def verified_lift(job, mode, side, mm):
    if job.get("status") != "completed" or not job.get("result", {}):
        return False
    r = job["result"]
    q = job["request"]
    actual = (
        r.get("foot_lift", {}).get("requested_clearance_m", 0.002)
        if mode == "foot_lift"
        else r.get("requested_clearance_m", 0.002)
    )
    return (
        bool(r.get("passed"))
        and q.get("mode") == mode
        and q.get("shift_side") == side
        and q.get("lift_mm", 2) == mm
        and abs(actual - mm / 1000) < 1e-12
    )


def completion_message(result):
    if "alternating_walk" in result:
        return (
            "Alternating steps, support exchanges and final stance passed"
            if result.get("passed")
            else "Walking trial completed; inspect failed checks"
        )
    if result.get("preview_kind") == "alternating_walk":
        return (
            "Offline alternating-walk preflight passed; no measured gait"
            if result.get("passed")
            else "Offline walking preflight failed"
        )
    if "first_step" in result:
        return (
            "Forward step, landing and staggered 50/50 reload passed"
            if result.get("passed")
            else "Step completed; inspect failed checks"
        )
    if result.get("preview_kind") == "first_step":
        return (
            "Offline forward-step preflight passed; no measured step"
            if result.get("passed")
            else "Offline step checks failed"
        )
    if "foot_lift" in result:
        return (
            "Measured foot lift, contact loss, landing and return passed"
            if result.get("passed")
            else "Foot-lift trial completed; inspect failed checks"
        )
    if result.get("preview_kind") == "foot_lift_feasibility":
        return (
            "Offline lift feasibility passed; no simulation or measured lift"
            if result.get("passed")
            else "Offline lift feasibility checks failed"
        )
    if "zero_load_transition" in result:
        return (
            "Zero-load force, contact-state, posture and return audits passed"
            if result.get("passed")
            else "Zero-load transition completed; inspect failed audits"
        )
    if result.get("bench_kind") == "contact_transition":
        return (
            "Contact/air/contact sensor diagnostic passed"
            if result.get("passed")
            else "Contact diagnostic did not pass: " + str(result.get("error"))
        )
    if result.get("motion_plan", {}).get("program_kind") == "lowered_shift":
        return (
            "Lowered transfer, measured loads, planted feet and return checks passed"
            if result.get("passed")
            else "Completed; lowered-transfer checks did not all pass"
        )
    if "lowering" in result:
        return (
            "Lowering, planted feet, measured loads and return checks passed"
            if result.get("passed")
            else "Completed; lowering checks did not all pass"
        )
    if result.get("passed"):
        return (
            "Transfer, force, posture and return checks passed"
            if result.get("actuator_profile", {}).get("name") == "robotis_p54"
            else "Recovery checks passed"
        )
    push = result.get("push", {})
    if not push.get("response_detected", True) or not result.get(
        "pitch_response_above_reference", True
    ):
        return "Completed; the response was too small for a validated push comparison"
    return "Completed; one or more recovery/contact checks did not pass"


def progress(output):
    if (output / "joints.shift.json").is_file():
        lines = tail(output / "joints.jsonl", 16000).splitlines()
        for line in reversed(lines):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            shift = r.get("shift")
            if shift:
                phase = shift["phase"]
                if (read_json(output / "joints.shift.json") or {}).get(
                    "experiment_kind"
                ) == "alternating_walk":
                    return (
                        phase,
                        f"Walking step {shift.get('step_index',1)}/{(read_json(output/'joints.shift.json') or {}).get('step_count',3)}: "
                        + phase.replace("_", " "),
                        (
                            2
                            if phase in ("lift", "advance")
                            else (
                                3
                                if phase
                                in (
                                    "lower_foot",
                                    "recontact",
                                    "reload",
                                    "staggered_hold",
                                )
                                else 0
                            )
                        ),
                    )
                if (read_json(output / "joints.shift.json") or {}).get(
                    "experiment_kind"
                ) == "first_step":
                    return (
                        phase,
                        "First step: " + phase.replace("_", " "),
                        (
                            2
                            if phase in ("lift", "advance")
                            else (
                                3
                                if phase
                                in (
                                    "lower_foot",
                                    "recontact",
                                    "reload",
                                    "staggered_hold",
                                )
                                else 0
                            )
                        ),
                    )
                pulse = read_json(output / "pulse-events.json") or []
                active = pulse and pulse[-1]["event"] == "push_published"
                plan = read_json(output / "joints.shift.json") or {}
                message = "P54 lowering: " + {
                    "move_out": "lowering pelvis",
                    "move_back": "rising to standing",
                }.get(phase, phase)
                if plan.get("program_kind") == "lowered_shift":
                    message = "P54 lowered transfer: " + phase.replace("_", " ")
                if plan.get("experiment_kind") == "live_foot_lift":
                    message = "P54 measured lift: " + phase.replace("_", " ")
                return (
                    phase,
                    (
                        message
                        if plan.get("program_kind")
                        in ("bilateral_lowering", "lowered_shift")
                        else (
                            "Lateral force pulse active"
                            if active
                            else "P54 weight transfer: " + phase.replace("_", " ")
                        )
                    ),
                    {
                        "baseline": 0,
                        "lower": 0,
                        "lowered_settle": 0,
                        "move_out": 1,
                        "hold": 2,
                        "lift": 2,
                        "lifted_hold": 2,
                        "lower_foot": 3,
                        "recontact": 3,
                        "move_back": 3,
                        "center_settle": 3,
                        "rise": 3,
                        "recovery": 3,
                        "done": 4,
                    }.get(phase, 0),
                )
            break
        return "warmup", "P54: waiting for bounded effort standing readiness", 0
    if (output / "visual-events.json").exists():
        events = read_json(output / "visual-events.json") or []
        released = any(e["event"] == "clear_published" for e in events)
        return (
            "visual",
            (
                "Visual pulse finished; Gazebo stays open until Stop"
                if released
                else "Visual force pulse active"
            ),
            3,
        )
    for trial in ("push", "reference"):
        path = output / trial
        if not path.exists():
            continue
        events = read_json(path / "recording/events.json") or []
        last = events[-1]["event"] if events else None
        if trial == "reference":
            if (path / "recording/summary.json").exists():
                return (
                    "reference_done",
                    "Reference complete; preparing the push trial",
                    1,
                )
            if events:
                return "reference_recording", "Recording the no-push reference", 0
            return "reference_warmup", "Reference: waiting for the posture to settle", 0
        if last == "clear_published":
            return "recovery", "Force cleared; observing recovery", 3
        if last == "push_published":
            return "force_active", "Force pulse has been published", 3
        if events:
            return "baseline", "Recording the pre-push baseline", 2
        return "push_warmup", "Push trial: waiting for the posture to settle", 1
    return "starting", "Starting the isolated simulation", 0


class JobManager:
    def __init__(self, root, runner_factory=None):
        self.root = Path(root).resolve()
        self.store = self.root / "evidence/gui/.panel"
        self.store.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.jobs = {}
        self.process = None
        self.worker = None
        self.active_id = None
        self.runner_factory = runner_factory
        for path in sorted(self.store.glob("*.json")):
            value = read_json(path)
            if (
                not value
                or value.get("id") != path.stem
                or not re.fullmatch(r"\d{8}-\d{6}-[a-f0-9]{8}", path.stem)
            ):
                continue
            if value.get("status") not in TERMINAL:
                value.update(
                    status="interrupted",
                    message="Panel restarted; this previous run is not owned by the new server.",
                )
            self.jobs[path.stem] = value

    def output(self, job_id):
        if job_id not in self.jobs:
            raise KeyError("Unknown experiment")
        return self.root / "evidence/gui" / job_id

    def _save(self, job):
        path = self.store / f'{job["id"]}.json'
        temp = path.with_suffix(".tmp")
        temp.write_text(json.dumps(job, indent=2) + "\n")
        temp.replace(path)

    def start(self, payload):
        config = ExperimentRequest.parse(payload)
        with self.lock:
            if self.active_id is not None:
                raise RuntimeError(
                    "An experiment is already running. Stop it or wait for completion."
                )
            if config.source_job:
                old = self.jobs.get(config.source_job)
                if (
                    not old
                    or old["request"].get("mode") != "alternating_walk"
                    or old["request"].get("source_job")
                    or old["request"].get("shift_side") != config.shift_side
                ):
                    raise ValueError(
                        "Re-audit requires an original same-side walking recording"
                    )
                original = (
                    read_json(self.output(config.source_job) / "summary.json") or {}
                )
                if not original.get(
                    "writers_closed_before_audit"
                ) or "[shift:done]" not in tail(
                    self.output(config.source_job) / "launch.log", 12000
                ):
                    raise ValueError(
                        "Walking program must finish with its writers closed before re-audit"
                    )
            from my_robot_description.model.planning_identity import (
                planning_fingerprint,
            )

            if config.mode in ("alternating_walk", "alternating_walk_preview"):
                if not any(
                    j["status"] == "completed"
                    and j.get("result")
                    and j["result"].get("passed")
                    and j["request"].get("mode") == "first_step"
                    and j["request"].get("shift_side") == config.shift_side
                    for j in self.jobs.values()
                ):
                    raise ValueError(
                        "Alternating walking requires a measured first step on the selected starting side"
                    )
                if config.mode == "alternating_walk" and not any(
                    j["status"] == "completed"
                    and j.get("result")
                    and j["result"].get("passed")
                    and j["result"].get("preview_kind") == "alternating_walk"
                    and j["request"].get("mode") == "alternating_walk_preview"
                    and j["result"].get("motion_plan", {}).get("step_count", 3)
                    == config.walk_steps
                    and abs(
                        j["result"]
                        .get("motion_plan", {})
                        .get("nominal_step_length_m", 0.05)
                        - config.walk_step_mm / 1000
                    )
                    < 1e-12
                    and j["result"].get("motion_plan", {}).get("time_scale", 1.0)
                    == config.walk_time_scale
                    and (
                        not config.walk_velocity_feedforward
                        or j["result"]
                        .get("motion_plan", {})
                        .get("planning_fingerprint")
                        == planning_fingerprint(self.root / "src/my_robot_description")
                    )
                    and j["result"]
                    .get("motion_plan", {})
                    .get("velocity_feedforward", False)
                    == config.walk_velocity_feedforward
                    and j["result"]
                    .get("motion_plan", {})
                    .get(
                        "transfer_time_scale",
                        j["result"].get("motion_plan", {}).get("time_scale", 1.0),
                    )
                    == (config.walk_transfer_time_scale or config.walk_time_scale)
                    and j["request"].get("shift_side") == config.shift_side
                    for j in self.jobs.values()
                ):
                    raise ValueError(
                        "Alternating walking requires a matching full-path offline preflight"
                    )
            if config.mode in ("first_step", "first_step_preview"):
                if not any(
                    verified_lift(j, "foot_lift", config.shift_side, 30)
                    for j in self.jobs.values()
                ):
                    raise ValueError(
                        "First step requires same-side measured 30 mm lift"
                    )
                if config.mode == "first_step" and not any(
                    j["status"] == "completed"
                    and j.get("result")
                    and j["result"].get("passed")
                    and j["result"].get("preview_kind") == "first_step"
                    and j["request"].get("mode") == "first_step_preview"
                    and j["request"].get("shift_side") == config.shift_side
                    for j in self.jobs.values()
                ):
                    raise ValueError(
                        "First step requires same-side offline step preflight"
                    )
            if (
                config.mode in ("foot_lift", "foot_lift_preview")
                and config.lift_mm >= 10
            ):
                if not any(
                    verified_lift(j, "foot_lift", config.shift_side, 2)
                    for j in self.jobs.values()
                ):
                    raise ValueError(
                        "Larger clearance requires same-side measured 2 mm lift first"
                    )
            if config.mode == "foot_lift":
                if not any(
                    j["status"] == "completed"
                    and j.get("result")
                    and j["result"].get("passed")
                    and j["request"].get("mode") == "zero_load"
                    and j["request"].get("shift_side") == config.shift_side
                    for j in self.jobs.values()
                ):
                    raise ValueError(
                        "Live foot lift requires same-side zero-load baseline"
                    )
                if not any(
                    verified_lift(
                        j, "foot_lift_preview", config.shift_side, config.lift_mm
                    )
                    for j in self.jobs.values()
                ):
                    raise ValueError(
                        "Live foot lift requires same-side matching-clearance offline preview"
                    )
            if config.mode == "foot_lift_preview" and not any(
                j["status"] == "completed"
                and j.get("result")
                and j["result"].get("passed")
                and j["request"].get("mode") == "zero_load"
                and j["request"].get("shift_side") == config.shift_side
                for j in self.jobs.values()
            ):
                raise ValueError(
                    "First-lift preview requires same-side verified zero-load baseline"
                )
            if config.mode == "zero_load":
                sensor_ok = any(
                    j["status"] == "completed"
                    and j.get("result")
                    and j["result"].get("passed")
                    and j["result"].get("contact_source") == "state"
                    and j["request"].get("mode") == "contact_bench"
                    for j in self.jobs.values()
                )
                support_ok = any(
                    j["status"] == "completed"
                    and j.get("result")
                    and j["result"].get("passed")
                    and j["request"].get("mode") == "lowered_unload"
                    and j["request"].get("shift_side") == config.shift_side
                    and j["request"].get("unload_percent") == 5
                    for j in self.jobs.values()
                )
                if not sensor_ok or not support_ok:
                    raise ValueError(
                        "Zero-load mode requires passing physics contact diagnostic and same-side 5% unloading"
                    )
            if config.mode == "lowered_unload" and config.unload_percent < 20:
                previous = 20 if config.unload_percent == 10 else 10
                if not any(
                    j["status"] == "completed"
                    and j.get("result", {})
                    and j["result"].get("passed")
                    and j["request"].get("mode") == "lowered_unload"
                    and j["request"].get("shift_side") == config.shift_side
                    and j["request"].get("unload_percent") == previous
                    for j in self.jobs.values()
                ):
                    raise ValueError(
                        f"Pass the {previous}% remaining-load stage on this supporting side first"
                    )
            job_id = (
                datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
                + "-"
                + uuid.uuid4().hex[:8]
            )
            job = dict(
                id=job_id,
                status="running",
                phase="starting",
                message="Starting experiment",
                created_at=now(),
                finished_at=None,
                request=config.to_dict(),
                cancel_requested=False,
                error=None,
                output_relative=f"evidence/gui/{job_id}",
                plot_ready=False,
                result=None,
            )
            self.jobs[job_id] = job
            self.active_id = job_id
            self._save(job)
            self.worker = threading.Thread(
                target=self._run, args=(job_id, config), daemon=True
            )
            self.worker.start()
            return job_id

    def _run(self, job_id, config):
        job = self.jobs[job_id]
        output = self.output(job_id)
        log = self.store / f"{job_id}.log"
        env = os.environ.copy()
        env[TOKEN_ENV] = job_id
        if config.gazebo_gui or config.mode == "visual":
            env = desktop_environment(env)
        env.setdefault("MPLCONFIGDIR", "/tmp/biped-matplotlib")
        try:
            with log.open("w") as stream:
                command = (
                    self.runner_factory(config, output)
                    if self.runner_factory
                    else config.command(self.root, output)
                )
                with self.lock:
                    if job["cancel_requested"]:
                        job["status"] = "stopped"
                        return
                    job["command"] = command
                    self.process = subprocess.Popen(
                        command,
                        cwd=self.root,
                        env=env,
                        stdout=stream,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                    )
                deadline = None
                while self.process.poll() is None:
                    if job["cancel_requested"]:
                        if deadline is None:
                            self.process.send_signal(signal.SIGINT)
                            deadline = time.monotonic() + 30
                        if time.monotonic() > deadline:
                            terminate_owned(job_id)
                            self.process.wait(timeout=5)
                    time.sleep(0.15)
                code = self.process.returncode
                with self.lock:
                    self.process = None
                    job["returncode"] = code
                if job["cancel_requested"]:
                    job.update(
                        status="stopped",
                        message="Stopped. The owned experiment was asked to clear its force and close its simulation.",
                    )
                    return
                result = read_json(output / "summary.json")
                if code != 0 and not (
                    config.mode
                    in (
                        "load_transfer",
                        "lowering",
                        "lowered_shift",
                        "lowered_unload",
                        "contact_bench",
                        "zero_load",
                        "foot_lift_preview",
                        "foot_lift",
                        "first_step_preview",
                        "first_step",
                        "alternating_walk",
                        "alternating_walk_preview",
                    )
                    and code == 2
                    and result
                    and result.get("completed")
                ):
                    raise RuntimeError(
                        "The experiment stopped before completion. Inspect the run log below."
                    )
                if not result or not result.get("completed"):
                    raise RuntimeError(
                        "The runner did not produce a complete experiment summary."
                    )
                job["result"] = result
                job["phase"] = "plotting"
                job["message"] = "Experiment complete; drawing the response plot"
                if (
                    self.runner_factory is None
                    and not config.source_job
                    and config.mode
                    not in (
                        "contact_bench",
                        "foot_lift_preview",
                        "first_step_preview",
                        "alternating_walk_preview",
                    )
                ):
                    self.process = subprocess.Popen(
                        [
                            sys.executable,
                            str(
                                self.root
                                / (
                                    "scripts/plots/plot_alternating_walk.py"
                                    if config.mode == "alternating_walk"
                                    else (
                                        "scripts/plots/plot_first_step.py"
                                        if config.mode == "first_step"
                                        else (
                                            "scripts/plots/plot_lowering.py"
                                            if config.mode
                                            in (
                                                "lowering",
                                                "lowered_shift",
                                                "lowered_unload",
                                                "zero_load",
                                                "foot_lift",
                                            )
                                            else (
                                                "scripts/plots/plot_lateral_experiment.py"
                                                if config.mode == "load_transfer"
                                                else "scripts/plots/plot_effort_push.py"
                                            )
                                        )
                                    )
                                )
                            ),
                            str(output),
                        ],
                        cwd=self.root,
                        env=env,
                        stdout=stream,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                    )
                    while self.process.poll() is None:
                        if job["cancel_requested"]:
                            self.process.send_signal(signal.SIGINT)
                        time.sleep(0.1)
                    plot_code = self.process.returncode
                    self.process = None
                    if job["cancel_requested"]:
                        job["status"] = "stopped"
                        return
                    if plot_code:
                        job["plot_error"] = (
                            "The recording is complete, but plotting failed; inspect the run log."
                        )
                job["plot_ready"] = (output / "response.png").exists()
                job.update(status="completed", message=completion_message(result))
        except Exception as error:
            job.update(
                status="stopped" if job["cancel_requested"] else "failed",
                error=str(error),
                message=str(error),
            )
        finally:
            with self.lock:
                job["finished_at"] = now()
                self._save(job)
                self.process = None
                self.active_id = None

    def reaudit(self, job_id):
        old = self.jobs.get(job_id)
        if not old:
            raise ValueError("Unknown recording")
        payload = dict(old["request"])
        payload.update(source_job=job_id, gazebo_gui=False)
        return self.start(payload)

    def cancel(self, job_id):
        with self.lock:
            if job_id != self.active_id:
                raise ValueError("This experiment is not running")
            self.jobs[job_id].update(
                cancel_requested=True,
                status="stopping",
                message="Stopping: clearing the push and closing the owned simulation",
            )
            self._save(self.jobs[job_id])

    def snapshot(self):
        with self.lock:
            values = []
            for job in list(self.jobs.values())[-10:][::-1]:
                value = dict(job)
                if (
                    job["id"] == self.active_id
                    and job["status"] == "running"
                    and job["phase"] != "plotting"
                ):
                    phase, message, step = progress(self.output(job["id"]))
                    if job["request"].get("source_job"):
                        phase, message, step = (
                            "reaudit",
                            "Replaying the saved walking measurements; no simulation or robot commands",
                            4,
                        )
                    if job["request"].get("mode") == "foot_lift_preview":
                        phase, message, step = (
                            "planning",
                            "Offline: checking IK, static torques and independent gravity gradients; no simulation",
                            0,
                        )
                    if job["request"].get("mode") == "visual" and phase == "starting":
                        message = "Visual simulation: waiting for standing readiness; no reference trial"
                    value.update(phase=phase, message=message, step=step)
                else:
                    value["step"] = (
                        5
                        if job["status"] == "completed"
                        else (4 if job["phase"] == "plotting" else 0)
                    )
                value["log"] = tail(self.store / f'{job["id"]}.log')
                if job["status"] not in TERMINAL:
                    for part in ("reference", "push"):
                        value["log"] += (
                            "\n"
                            + part.upper()
                            + "\n"
                            + tail(
                                self.output(job["id"]) / part / "experiment.log", 4000
                            )
                        )
                values.append(value)
            return dict(active_id=self.active_id, jobs=values, roadmap=self.roadmap())

    def roadmap(self):
        baseline = (
            read_json(self.root / "evidence/p54-bilateral-lowering-01/summary.json")
            or {}
        )
        lowering = [
            j
            for j in self.jobs.values()
            if j.get("request", {}).get("mode") == "lowering"
            and j.get("result", {})
            and j["result"].get("passed")
            and j["status"] == "completed"
        ]
        verified = lambda mm: any(j["request"]["lowering_mm"] == mm for j in lowering)
        unloaded = lambda percent, side: any(
            j["status"] == "completed"
            and j.get("result")
            and j["result"].get("passed")
            and j["request"].get("mode") == "lowered_unload"
            and j["request"].get("unload_percent") == percent
            and j["request"].get("shift_side") == side
            for j in self.jobs.values()
        )
        return [
            dict(
                title="Bilateral load transfer + small pulses",
                status=(
                    "verified"
                    if any(
                        j.get("result")
                        and j["result"].get("passed")
                        and j["request"].get("mode") == "load_transfer"
                        for j in self.jobs.values()
                    )
                    else "available"
                ),
                mode="load_transfer",
            ),
            dict(
                title="10 mm centered lowering + return",
                status=(
                    "verified"
                    if baseline.get("passed") or verified(10)
                    else "available"
                ),
                mode="lowering",
                lowering_mm=10,
            ),
            dict(
                title="20 mm centered lowering + return",
                status="verified" if verified(20) else "next experiment",
                mode="lowering",
                lowering_mm=20,
            ),
            dict(
                title="30 mm centered lowering + return",
                status="verified" if verified(30) else "next experiment",
                mode="lowering",
                lowering_mm=30,
            ),
            dict(
                title="40 mm centered lowering + return",
                status="verified" if verified(40) else "next experiment",
                mode="lowering",
                lowering_mm=40,
            ),
            *[
                dict(
                    title=f"40 mm lowered transfer: {mm} mm " + side,
                    status=(
                        "verified"
                        if any(
                            j.get("result")
                            and j["result"].get("passed")
                            and j["status"] == "completed"
                            and j["request"].get("mode") == "lowered_shift"
                            and j["request"].get("shift_side") == side
                            and j["request"].get("shift_mm", 5) == mm
                            for j in self.jobs.values()
                        )
                        else "ready to test" if mm >= 10 else "next experiment"
                    ),
                    mode="lowered_shift",
                    lowering_mm=40,
                    shift_side=side,
                    shift_mm=mm,
                )
                for mm in [5, 10, 20, 30, 40]
                for side in ["left", "right"]
            ],
            *[
                dict(
                    title=f"Lowered unloading: {percent}% remaining, {side} support",
                    status=(
                        "verified"
                        if unloaded(percent, side)
                        else (
                            "ready to test"
                            if percent == 20
                            or unloaded(20 if percent == 10 else 10, side)
                            else "requires previous stage"
                        )
                    ),
                    mode="lowered_unload",
                    lowering_mm=40,
                    unload_percent=percent,
                    shift_side=side,
                )
                for percent in [20, 10, 5]
                for side in ["left", "right"]
            ],
            dict(
                title="Contact sensor: contact / air / contact",
                status=(
                    "verified"
                    if any(
                        j.get("result")
                        and j["result"].get("passed")
                        and j["request"].get("mode") == "contact_bench"
                        for j in self.jobs.values()
                    )
                    else "ready to test"
                ),
                mode="contact_bench",
            ),
            *[
                dict(
                    title=f"Zero-load transition: {side} support",
                    status=(
                        "verified"
                        if any(
                            j["status"] == "completed"
                            and j.get("result")
                            and j["result"].get("passed")
                            and j["request"].get("mode") == "zero_load"
                            and j["request"].get("shift_side") == side
                            for j in self.jobs.values()
                        )
                        else "ready after sensor + 5% checks"
                    ),
                    mode="zero_load",
                    lowering_mm=40,
                    shift_side=side,
                )
                for side in ["left", "right"]
            ],
            *[
                dict(
                    title=f"{mm} mm foot-lift OFFLINE preview: {side} support",
                    status=(
                        "offline checked; live trial separate"
                        if any(
                            verified_lift(j, "foot_lift_preview", side, mm)
                            for j in self.jobs.values()
                        )
                        else (
                            "ready after zero-load check"
                            if mm == 2
                            else "requires same-side 2 mm lift"
                        )
                    ),
                    mode="foot_lift_preview",
                    shift_side=side,
                    lift_mm=mm,
                )
                for mm in [2, 10, 30]
                for side in ["left", "right"]
            ],
            *[
                dict(
                    title=f"{mm} mm measured foot lift: {side} support",
                    status=(
                        "verified"
                        if any(
                            verified_lift(j, "foot_lift", side, mm)
                            for j in self.jobs.values()
                        )
                        else "ready after zero-load + matching preview"
                    ),
                    mode="foot_lift",
                    lowering_mm=40,
                    shift_side=side,
                    lift_mm=mm,
                )
                for mm in [2, 10, 30]
                for side in ["left", "right"]
            ],
            *[
                dict(
                    title=f"50 mm forward step OFFLINE preflight: {side} support",
                    status=(
                        "offline checked"
                        if any(
                            j["status"] == "completed"
                            and j.get("result")
                            and j["result"].get("passed")
                            and j["request"].get("mode") == "first_step_preview"
                            and j["request"].get("shift_side") == side
                            for j in self.jobs.values()
                        )
                        else "requires measured 30 mm lift"
                    ),
                    mode="first_step_preview",
                    lowering_mm=40,
                    shift_side=side,
                    lift_mm=30,
                )
                for side in ["left", "right"]
            ],
            *[
                dict(
                    title=f"50 mm measured forward step: {side} support",
                    status=(
                        "verified"
                        if any(
                            j["status"] == "completed"
                            and j.get("result")
                            and j["result"].get("passed")
                            and j["request"].get("mode") == "first_step"
                            and j["request"].get("shift_side") == side
                            for j in self.jobs.values()
                        )
                        else "requires step preflight"
                    ),
                    mode="first_step",
                    lowering_mm=40,
                    shift_side=side,
                    lift_mm=30,
                )
                for side in ["left", "right"]
            ],
            *[
                dict(
                    title="Three alternating steps"
                    + (
                        " OFFLINE preflight"
                        if mode.endswith("preview")
                        else " measured"
                    ),
                    status=(
                        ("offline checked" if mode.endswith("preview") else "verified")
                        if any(
                            j["status"] == "completed"
                            and j.get("result")
                            and j["result"].get("passed")
                            and j["request"].get("mode") == mode
                            for j in self.jobs.values()
                        )
                        else "requires first step + walking preflight"
                    ),
                    mode=mode,
                    lowering_mm=40,
                    shift_side="left",
                    lift_mm=30,
                    walk_steps=3,
                    walk_step_mm=50,
                    walk_time_scale=1,
                )
                for mode in ["alternating_walk_preview", "alternating_walk"]
            ],
            dict(
                title="10 steps / 75 mm / 2× motion",
                status=(
                    "verified"
                    if any(
                        j["status"] == "completed"
                        and (j.get("result") or {}).get("passed")
                        and j["request"].get("mode") == "alternating_walk"
                        and j["request"].get("walk_steps") == 10
                        and j["request"].get("walk_step_mm") == 75
                        and j["request"].get("walk_time_scale") == 0.5
                        for j in self.jobs.values()
                    )
                    else "requires matching preflight"
                ),
                mode="alternating_walk",
                lowering_mm=40,
                shift_side="left",
                lift_mm=30,
                walk_steps=10,
                walk_step_mm=75,
                walk_time_scale=0.5,
            ),
            *[
                dict(title=t, status="planned", mode=None)
                for t in ["Continuous gait and disturbance recovery while walking"]
            ],
        ]

    def shutdown(self):
        with self.lock:
            if self.active_id:
                self.cancel(self.active_id)
            worker = self.worker
        if worker and worker.is_alive():
            worker.join(timeout=35)
        if worker and worker.is_alive():
            with self.lock:
                token = self.active_id
            if token:
                terminate_owned(token)
            worker.join(timeout=5)
