"""Typed GUI requests and fixed experiment command construction."""

from dataclasses import asdict, dataclass
import math
import re
from pathlib import Path
import sys
from my_robot_description.experiments.effort_push import EffortPushConfig

DIRECTIONS = {"forward": 1.0, "backward": -1.0}


@dataclass(frozen=True)
class ExperimentRequest:
    source_job: str = ""
    mode: str = "experiment"
    direction: str = "forward"
    force_n: float = 2.0
    duration_s: float = 0.2
    balance_enabled: bool = True
    balance_profile: str = "original"
    full_settling: bool = False
    gazebo_gui: bool = True
    loaded_side: str = "left"
    lowering_mm: float = 10.0
    shift_side: str = "left"
    shift_mm: float = 5.0
    unload_percent: int = 20
    lift_mm: int = 2
    walk_steps: int = 3
    walk_step_mm: int = 50
    walk_time_scale: float = 1.0
    walk_transfer_time_scale: float = 0.0
    walk_velocity_feedforward: bool = False

    @classmethod
    def parse(cls, payload):
        if not isinstance(payload, dict):
            raise ValueError("Expected an experiment object")
        if set(payload) - set(cls.__dataclass_fields__):
            raise ValueError("Unknown experiment setting")
        obj = cls(**payload)
        from my_robot_description.motion.walking_profile import WalkingProfile

        if type(obj.walk_step_mm) is not int or type(obj.walk_time_scale) not in (
            float,
            int,
        ):
            raise ValueError("Walking dimensions must be numeric")
        WalkingProfile(
            obj.walk_steps,
            obj.walk_step_mm / 1000,
            obj.walk_time_scale,
            obj.walk_transfer_time_scale,
            obj.walk_velocity_feedforward,
        ).validate()
        if (
            not isinstance(obj.source_job, str)
            or obj.source_job
            and (
                obj.mode != "alternating_walk"
                or not re.fullmatch(r"\d{8}-\d{6}-[a-f0-9]{8}", obj.source_job)
                or obj.gazebo_gui
            )
        ):
            raise ValueError(
                "Re-audit requires a saved walking job ID and no simulation"
            )
        if obj.balance_profile not in ("original", "ankle_response", "damped_response"):
            raise ValueError("Choose original or experimental ankle response")
        if (
            obj.balance_profile in ("ankle_response", "damped_response")
            and not obj.balance_enabled
        ):
            raise ValueError("Experimental ankle response requires IMU balance enabled")
        if obj.mode not in (
            "experiment",
            "visual",
            "load_transfer",
            "lowering",
            "lowered_shift",
            "lowered_unload",
            "contact_bench",
            "zero_load",
            "foot_lift_preview",
            "foot_lift",
            "first_step",
            "first_step_preview",
            "alternating_walk",
            "alternating_walk_preview",
        ):
            raise ValueError("Choose a supported experiment mode")
        if (
            isinstance(obj.lowering_mm, bool)
            or not isinstance(obj.lowering_mm, (int, float))
            or obj.lowering_mm not in (10, 20, 30, 40)
        ):
            raise ValueError("Choose a 10, 20, 30 or 40 mm lowering milestone")
        if type(obj.unload_percent) is not int or obj.unload_percent not in (20, 10, 5):
            raise ValueError("Choose 20%, 10% or 5% remaining load")
        if type(obj.lift_mm) is not int or obj.lift_mm not in (2, 10, 30):
            raise ValueError("Choose a 2, 10 or 30 mm foot clearance")
        if obj.shift_side not in ("left", "right"):
            raise ValueError("Shift side must be left or right")
        if (
            isinstance(obj.shift_mm, bool)
            or not isinstance(obj.shift_mm, (int, float))
            or obj.shift_mm not in (5, 10, 20, 30, 40)
        ):
            raise ValueError("Choose a 5, 10, 20, 30 or 40 mm lowered transfer")
        if (
            obj.mode
            in (
                "first_step",
                "first_step_preview",
                "alternating_walk",
                "alternating_walk_preview",
            )
            and obj.lift_mm != 30
        ):
            raise ValueError("First step requires exactly 30 mm clearance")
        if obj.mode in (
            "foot_lift_preview",
            "first_step_preview",
            "alternating_walk_preview",
        ):
            if (
                type(obj.gazebo_gui) is not bool
                or obj.gazebo_gui
                or type(obj.balance_enabled) is not bool
                or obj.balance_enabled
                or obj.balance_profile != "original"
                or type(obj.full_settling) is not bool
                or obj.full_settling
                or isinstance(obj.force_n, bool)
                or not isinstance(obj.force_n, (int, float))
                or obj.force_n != 0
            ):
                raise ValueError(
                    "Foot-lift preview is offline, zero-force and has no Gazebo or controller commands"
                )
            return obj
        if obj.mode == "contact_bench":
            if (
                type(obj.balance_enabled) is not bool
                or type(obj.full_settling) is not bool
                or type(obj.gazebo_gui) is not bool
                or isinstance(obj.force_n, bool)
                or not isinstance(obj.force_n, (int, float))
            ):
                raise ValueError(
                    "Diagnostic settings require booleans and numeric zero force"
                )
            if (
                obj.force_n != 0
                or obj.balance_enabled
                or obj.balance_profile != "original"
                or obj.full_settling
            ):
                raise ValueError(
                    "Contact diagnostic has no robot commands or external force"
                )
            return obj
        if obj.mode in (
            "lowering",
            "lowered_shift",
            "lowered_unload",
            "zero_load",
            "foot_lift",
            "first_step",
            "alternating_walk",
        ):
            if (
                obj.mode
                in (
                    "lowered_shift",
                    "lowered_unload",
                    "zero_load",
                    "foot_lift",
                    "first_step",
                    "alternating_walk",
                )
                and obj.lowering_mm != 40
            ):
                raise ValueError("Combined shift requires 40 mm lowering")
            if any(
                type(v) is not bool
                for v in [obj.gazebo_gui, obj.balance_enabled, obj.full_settling]
            ):
                raise ValueError("Controller and Gazebo options must be true or false")
            if isinstance(obj.force_n, bool) or not isinstance(
                obj.force_n, (int, float)
            ):
                raise ValueError("Force must be numeric zero for lowering")
            if (
                obj.force_n != 0
                or obj.balance_enabled
                or obj.balance_profile != "original"
                or obj.full_settling
            ):
                raise ValueError(
                    "Lowering is a no-push joint-PID experiment with full measured readiness"
                )
            return obj
        lateral = obj.mode == "load_transfer"
        if obj.loaded_side not in ("left", "right"):
            raise ValueError("Loaded side must be left or right")
        if not isinstance(obj.direction, str) or obj.direction not in (
            ("left", "right") if lateral else DIRECTIONS
        ):
            raise ValueError(
                "Choose left/right for P54 or forward/backward for the legacy robot"
            )
        if lateral and obj.balance_profile != "original":
            raise ValueError(
                "P54 mode uses measured-load feedback, not legacy ankle profiles"
            )
        for name in ("force_n", "duration_s"):
            value = getattr(obj, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(f"{name} must be a finite number")
        if lateral:
            if not 0 <= obj.force_n <= 1 or not 0.02 <= obj.duration_s <= 0.2:
                raise ValueError(
                    "P54 bilateral trials support 0–1 N and 0.02–0.2 s; 0 N runs a reference"
                )
        elif not 0.1 <= obj.force_n <= 100.0:
            raise ValueError(
                "This panel supports 0.1–100 N for recovery-limit experiments"
            )
        if (
            type(obj.balance_enabled) is not bool
            or type(obj.gazebo_gui) is not bool
            or type(obj.full_settling) is not bool
        ):
            raise ValueError("Balance and Gazebo options must be true or false")
        if not lateral:
            EffortPushConfig(force_x=obj.force_x, duration=obj.duration_s).validate()
        return obj

    @property
    def force_x(self):
        return DIRECTIONS[self.direction] * self.force_n

    def command(self, root, output):
        if self.mode == "alternating_walk" and self.source_job:
            return [
                sys.executable,
                str(Path(root) / "scripts/analysis/reaudit_alternating_walk.py"),
                "--input",
                str(Path(root) / "evidence/gui" / self.source_job),
                "--output",
                str(output),
            ]
        if self.mode == "alternating_walk":
            return [
                sys.executable,
                str(Path(root) / "scripts/experiments/run_lateral_shift.py"),
                "--actuator-profile",
                "robotis_p54",
                "--alternating-walk",
                "--walk-steps",
                str(self.walk_steps),
                "--walk-step-mm",
                str(self.walk_step_mm),
                "--walk-time-scale",
                str(float(self.walk_time_scale)),
                "--walk-transfer-scale",
                str(float(self.walk_transfer_time_scale)),
                *(
                    ["--walk-velocity-feedforward"]
                    if self.walk_velocity_feedforward
                    else []
                ),
                "--real-time-factor",
                "1.0",
                "--direction",
                self.shift_side,
                *(["--gui"] if self.gazebo_gui else []),
                "--output",
                str(output),
            ]
        if self.mode == "alternating_walk_preview":
            return [
                sys.executable,
                str(Path(root) / "scripts/planning/preview_alternating_walk.py"),
                "--walk-steps",
                str(self.walk_steps),
                "--walk-step-mm",
                str(self.walk_step_mm),
                "--walk-time-scale",
                str(float(self.walk_time_scale)),
                "--walk-transfer-scale",
                str(float(self.walk_transfer_time_scale)),
                *(
                    ["--walk-velocity-feedforward"]
                    if self.walk_velocity_feedforward
                    else []
                ),
                "--support",
                self.shift_side,
                "--output",
                str(output),
            ]
        if self.mode == "first_step":
            return [
                sys.executable,
                str(Path(root) / "scripts/experiments/run_lateral_shift.py"),
                "--actuator-profile",
                "robotis_p54",
                "--first-step",
                "--direction",
                self.shift_side,
                *(["--gui"] if self.gazebo_gui else []),
                "--output",
                str(output),
            ]
        if self.mode == "first_step_preview":
            return [
                sys.executable,
                str(Path(root) / "scripts/planning/preview_first_step.py"),
                "--support",
                self.shift_side,
                "--output",
                str(output),
            ]
        if self.mode == "foot_lift":
            return [
                sys.executable,
                str(Path(root) / "scripts/experiments/run_lateral_shift.py"),
                "--actuator-profile",
                "robotis_p54",
                "--foot-lift",
                "--lift-mm",
                str(self.lift_mm),
                "--direction",
                self.shift_side,
                *(["--gui"] if self.gazebo_gui else []),
                "--output",
                str(output),
            ]
        if self.mode == "foot_lift_preview":
            return [
                sys.executable,
                str(Path(root) / "scripts/planning/preview_foot_lift.py"),
                "--clearance-mm",
                str(self.lift_mm),
                "--support",
                self.shift_side,
                "--output",
                str(output),
            ]
        if self.mode == "zero_load":
            return [
                sys.executable,
                str(Path(root) / "scripts/experiments/run_lateral_shift.py"),
                "--actuator-profile",
                "robotis_p54",
                "--zero-load",
                "--direction",
                self.shift_side,
                *(["--gui"] if self.gazebo_gui else []),
                "--output",
                str(output),
            ]
        if self.mode == "contact_bench":
            return [
                sys.executable,
                str(Path(root) / "scripts/experiments/run_contact_transition_bench.py"),
                "--output",
                str(output),
            ]
        if self.mode == "lowered_unload":
            return [
                sys.executable,
                str(Path(root) / "scripts/experiments/run_lateral_shift.py"),
                "--actuator-profile",
                "robotis_p54",
                "--lowered-unload",
                str(self.unload_percent / 100),
                "--direction",
                self.shift_side,
                *(["--gui"] if self.gazebo_gui else []),
                "--output",
                str(output),
            ]
        if self.mode == "lowered_shift":
            displacement = (
                self.shift_mm / 1000 * (1 if self.shift_side == "left" else -1)
            )
            return [
                sys.executable,
                str(Path(root) / "scripts/experiments/run_lateral_shift.py"),
                "--actuator-profile",
                "robotis_p54",
                "--lowered-shift",
                str(displacement),
                *(["--gui"] if self.gazebo_gui else []),
                "--output",
                str(output),
            ]
        if self.mode == "lowering":
            return [
                sys.executable,
                str(Path(root) / "scripts/experiments/run_lateral_shift.py"),
                "--actuator-profile",
                "robotis_p54",
                "--lowering",
                str(self.lowering_mm / 1000),
                *(["--gui"] if self.gazebo_gui else []),
                "--output",
                str(output),
            ]
        if self.mode == "load_transfer":
            force = self.force_n * (1 if self.direction == "left" else -1)
            return [
                sys.executable,
                str(Path(root) / "scripts/experiments/run_lateral_shift.py"),
                "--actuator-profile",
                "robotis_p54",
                "--unload-fraction",
                "0.3",
                "--direction",
                self.loaded_side,
                "--pulse-force-y",
                str(force),
                "--pulse-duration",
                str(self.duration_s),
                *(["--load-feedback"] if self.balance_enabled else []),
                *(["--gui"] if self.gazebo_gui else []),
                "--output",
                str(output),
            ]
        profile_args = (
            ["--ankle-balance-limit", "1.5"]
            if self.balance_profile in ("ankle_response", "damped_response")
            else []
        )
        if self.balance_profile == "damped_response":
            profile_args += ["--balance-kd", "8.0"]
        if self.mode == "visual":
            return [
                sys.executable,
                str(Path(root) / "scripts/experiments/run_visual_push.py"),
                "--force-x",
                str(self.force_x),
                "--duration",
                str(self.duration_s),
                *(["--balance-enabled"] if self.balance_enabled else []),
                *profile_args,
                *(["--full-settle"] if self.full_settling else []),
                "--output",
                str(output),
            ]
        return [
            sys.executable,
            str(Path(root) / "scripts/experiments/run_effort_push.py"),
            "--damping-model",
            "explicit",
            "--controller-mode",
            "pid",
            "--real-time-factor",
            ".25",
            "--force-x",
            str(self.force_x),
            "--duration",
            str(self.duration_s),
            "--ros-domain-id",
            "58",
            "--balance-enabled" if self.balance_enabled else "--balance-observer",
            "--posture-delay",
            "0" if self.balance_enabled else "3",
            *(["--gui"] if self.gazebo_gui else []),
            *profile_args,
            "--output",
            str(output),
        ]

    def to_dict(self):
        return asdict(self)
