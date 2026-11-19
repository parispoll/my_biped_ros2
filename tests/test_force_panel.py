"""The GUI is an experiment client, not a second motor controller."""

import json
from pathlib import Path
import signal
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from force_panel.config import ExperimentRequest
from force_panel.jobs import (
    JobManager,
    completion_message,
    progress,
    desktop_environment,
)


class PanelConfigTests(unittest.TestCase):
    def test_live_lift_routes_and_requires_measured_baseline_and_preview(self):
        payload = dict(
            mode="foot_lift",
            shift_side="left",
            lowering_mm=40,
            force_n=0,
            balance_enabled=False,
            gazebo_gui=False,
        )
        request = ExperimentRequest.parse(payload)
        self.assertIn("--foot-lift", request.command(ROOT, ROOT / "evidence/test"))
        for bad in [dict(force_n=1), dict(lowering_mm=10), dict(balance_enabled=True)]:
            with self.assertRaises(ValueError):
                ExperimentRequest.parse(dict(payload, **bad))
        with tempfile.TemporaryDirectory() as tmp:
            manager = JobManager(tmp)
            with self.assertRaisesRegex(ValueError, "zero-load"):
                manager.start(payload)
            manager.jobs["baseline"] = dict(
                status="completed",
                request=dict(mode="zero_load", shift_side="left"),
                result=dict(passed=True),
            )
            with self.assertRaisesRegex(ValueError, "preview"):
                manager.start(payload)
            row = next(
                r
                for r in manager.roadmap()
                if r.get("mode") == "foot_lift" and r["shift_side"] == "left"
            )
            self.assertNotEqual(row["status"], "verified")

    def test_lift_preview_has_no_simulator_force_or_motor_command(self):
        payload = dict(
            mode="foot_lift_preview",
            shift_side="right",
            force_n=0,
            balance_enabled=False,
            gazebo_gui=False,
        )
        request = ExperimentRequest.parse(payload)
        command = request.command(ROOT, ROOT / "evidence/preview")
        self.assertIn(str(ROOT / "scripts/planning/preview_foot_lift.py"), command)
        self.assertEqual(command[command.index("--support") + 1], "right")
        self.assertNotIn("--gui", command)
        for bad in [
            dict(force_n=1),
            dict(force_n=True),
            dict(gazebo_gui=True),
            dict(balance_enabled=True),
            dict(full_settling=True),
        ]:
            with self.assertRaises(ValueError):
                ExperimentRequest.parse(dict(payload, **bad))
        self.assertIn(
            "Offline",
            completion_message(dict(preview_kind="foot_lift_feasibility", passed=True)),
        )
        with tempfile.TemporaryDirectory() as tmp:
            manager = JobManager(tmp)
            with self.assertRaisesRegex(ValueError, "zero-load"):
                manager.start(payload)
            manager.jobs["baseline"] = dict(
                id="baseline",
                status="completed",
                request=dict(mode="zero_load", shift_side="right"),
                result=dict(passed=True),
            )
            manager.jobs["preview"] = dict(
                id="preview",
                status="completed",
                request=payload,
                result=dict(passed=True),
            )
            row = next(
                r
                for r in manager.roadmap()
                if r.get("mode") == "foot_lift_preview" and r["shift_side"] == "right"
            )
            self.assertNotEqual(row["status"], "verified")
            self.assertIn("live trial separate", row["status"])

    def test_login_started_panel_refreshes_only_display_variables(self):
        from unittest.mock import patch
        from types import SimpleNamespace

        env = dict(DISPLAY=":old", BIPED_FORCE_PANEL_JOB="owned-job")
        response = SimpleNamespace(
            returncode=0,
            stdout="DISPLAY=:0\nXAUTHORITY=/run/user/1000/gdm/Xauthority\nUNRELATED_SECRET=not-imported\n",
        )
        with patch("force_panel.jobs.subprocess.run", return_value=response):
            result = desktop_environment(env)
        self.assertEqual(result["DISPLAY"], ":0")
        self.assertEqual(result["BIPED_FORCE_PANEL_JOB"], "owned-job")
        self.assertNotIn("UNRELATED_SECRET", result)
        self.assertEqual(env["DISPLAY"], ":old")
        with patch(
            "force_panel.jobs.subprocess.run", side_effect=OSError("no user manager")
        ):
            self.assertEqual(desktop_environment(env), env)

    def test_direction_and_units_map_to_existing_runner_arguments(self):
        config = ExperimentRequest.parse(
            dict(
                direction="backward",
                force_n=1.5,
                duration_s=0.12,
                balance_enabled=True,
                gazebo_gui=False,
            )
        )
        command = config.command(ROOT, ROOT / "evidence/gui/test path")
        self.assertEqual(command[command.index("--force-x") + 1], "-1.5")
        self.assertEqual(command[command.index("--duration") + 1], "0.12")
        self.assertIn("--balance-enabled", command)
        self.assertNotIn("--gui", command)
        self.assertEqual(command[-1], str(ROOT / "evidence/gui/test path"))
        self.assertEqual(command[command.index("--ros-domain-id") + 1], "58")

    def test_invalid_or_out_of_profile_values_are_rejected(self):
        for update in (
            dict(direction="left"),
            dict(direction=[]),
            dict(force_n=0),
            dict(force_n=100.1),
            dict(force_n=-1),
            dict(force_n=float("nan")),
            dict(force_n=True),
            dict(duration_s=0.01),
            dict(duration_s=0.51),
            dict(duration_s=float("inf")),
            dict(gazebo_gui="true"),
            dict(output="/tmp/other"),
        ):
            with self.subTest(update=update), self.assertRaises(ValueError):
                ExperimentRequest.parse(update)

    def test_expanded_limits_reach_runner_in_both_directions(self):
        for direction, signed in (("forward", "100.0"), ("backward", "-100.0")):
            config = ExperimentRequest.parse(
                dict(direction=direction, force_n=100.0, duration_s=0.5)
            )
            command = config.command(ROOT, Path("/tmp/limit-run"))
            self.assertEqual(command[command.index("--force-x") + 1], signed)
            self.assertEqual(command[command.index("--duration") + 1], "0.5")

    def test_visual_mode_uses_single_simulation_runner(self):
        config = ExperimentRequest.parse(
            dict(mode="visual", direction="backward", force_n=20.0, duration_s=0.3)
        )
        command = config.command(ROOT, Path("/tmp/visual-run"))
        self.assertTrue(command[1].endswith("run_visual_push.py"))
        self.assertEqual(command[command.index("--force-x") + 1], "-20.0")
        self.assertNotIn("--balance-observer", command)
        with self.assertRaises(ValueError):
            ExperimentRequest.parse(dict(mode="unknown"))

    def test_experimental_profile_is_explicit_for_both_runners(self):
        self.assertNotIn(
            "--ankle-balance-limit",
            ExperimentRequest.parse({}).command(ROOT, Path("/tmp/original")),
        )
        for mode in ("visual", "experiment"):
            command = ExperimentRequest.parse(
                dict(mode=mode, balance_profile="ankle_response")
            ).command(ROOT, Path("/tmp/candidate"))
            self.assertEqual(command[command.index("--ankle-balance-limit") + 1], "1.5")
        for update in (
            dict(balance_profile="unknown"),
            dict(balance_profile="ankle_response", balance_enabled=False),
        ):
            with self.assertRaises(ValueError):
                ExperimentRequest.parse(update)

    def test_damped_profile_forwards_gain_eight_with_same_correction_cap(self):
        for mode in ("visual", "experiment"):
            command = ExperimentRequest.parse(
                dict(mode=mode, balance_profile="damped_response")
            ).command(ROOT, Path("/tmp/damped"))
            self.assertEqual(command[command.index("--balance-kd") + 1], "8.0")
            self.assertEqual(command[command.index("--ankle-balance-limit") + 1], "1.5")
        with self.assertRaises(ValueError):
            ExperimentRequest.parse(
                dict(balance_profile="damped_response", balance_enabled=False)
            )

    def test_visual_full_settling_is_explicit_and_default_stays_fast(self):
        original = ExperimentRequest.parse(dict(mode="visual")).command(
            ROOT, Path("/tmp/fast")
        )
        full = ExperimentRequest.parse(dict(mode="visual", full_settling=True)).command(
            ROOT, Path("/tmp/full")
        )
        self.assertNotIn("--full-settle", original)
        self.assertIn("--full-settle", full)
        with self.assertRaises(ValueError):
            ExperimentRequest.parse(dict(full_settling="true"))

    def test_off_profile_uses_observer_and_matched_settling(self):
        cmd = ExperimentRequest.parse({"balance_enabled": False}).command(
            ROOT, Path("/tmp/run")
        )
        self.assertIn("--balance-observer", cmd)
        self.assertEqual(cmd[cmd.index("--posture-delay") + 1], "3")

    def test_small_response_is_not_mislabeled_as_a_recovery_failure(self):
        self.assertIn(
            "too small",
            completion_message(
                {"passed": False, "pitch_response_above_reference": False}
            ),
        )
        self.assertIn(
            "recovery/contact",
            completion_message({"passed": False, "push": {"response_detected": True}}),
        )


class PanelJobTests(unittest.TestCase):
    def wait(self, manager, seconds=8):
        deadline = time.monotonic() + seconds
        while manager.active_id is not None and time.monotonic() < deadline:
            time.sleep(0.03)
        self.assertIsNone(manager.active_id)

    def test_successful_job_saves_summary_and_history_without_overwriting(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            code = "import pathlib,json,sys;p=pathlib.Path(sys.argv[1]);p.mkdir();(p/'summary.json').write_text(json.dumps({'completed':True,'passed':True,'push':{}}))"
            manager = JobManager(
                root, lambda config, output: [sys.executable, "-c", code, str(output)]
            )
            first = manager.start({})
            self.wait(manager)
            self.assertEqual(manager.jobs[first]["status"], "completed")
            second = manager.start({})
            self.wait(manager)
            self.assertNotEqual(first, second)
            self.assertTrue((manager.output(first) / "summary.json").exists())
            restored = JobManager(root)
            self.assertEqual(len(restored.snapshot()["jobs"]), 2)

    def test_failure_does_not_become_success(self):
        with tempfile.TemporaryDirectory() as d:
            manager = JobManager(
                d, lambda *_: [sys.executable, "-c", 'raise RuntimeError("test error")']
            )
            job = manager.start({})
            self.wait(manager)
            self.assertEqual(manager.jobs[job]["status"], "failed")
            self.assertIn("test error", manager.snapshot()["jobs"][0]["log"])

    def test_only_one_run_at_a_time_and_cancel_reaches_owned_runner(self):
        with tempfile.TemporaryDirectory() as d:
            manager = JobManager(
                d, lambda *_: [sys.executable, "-c", "import time;time.sleep(20)"]
            )
            job = manager.start({})
            with self.assertRaises(RuntimeError):
                manager.start({})
            manager.cancel(job)
            self.wait(manager)
            self.assertEqual(manager.jobs[job]["status"], "stopped")
            self.assertFalse(manager.jobs[job]["plot_ready"])

    def test_reference_zero_force_is_not_reported_as_active_push(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "reference/recording"
            p.mkdir(parents=True)
            (p / "events.json").write_text(json.dumps([dict(event="push_published")]))
            phase, message, step = progress(Path(d))
            self.assertEqual(phase, "reference_recording")
            self.assertEqual(step, 0)
            self.assertIn("no-push", message)


class PanelRequestBoundaryTests(unittest.TestCase):
    def handler(
        self, host="127.0.0.1:8765", origin="http://127.0.0.1:8765", token="panel-token"
    ):
        import io
        from types import SimpleNamespace
        from unittest.mock import Mock
        from force_panel.server import make_handler

        manager = Mock()
        handler = make_handler(manager, "panel-token").__new__(
            make_handler(manager, "panel-token")
        )
        payload = b"{}"
        handler.server = SimpleNamespace(server_address=("127.0.0.1", 8765))
        handler.headers = {
            "Host": host,
            "Origin": origin,
            "X-Force-Panel-Token": token,
            "Content-Type": "application/json",
            "Content-Length": str(len(payload)),
        }
        handler.rfile = io.BytesIO(payload)
        handler.path = "/api/run"
        handler.respond = Mock()
        return handler, manager

    def test_foreign_origin_wrong_token_and_host_are_rejected_before_start(self):
        for kwargs in (
            {"origin": "http://example.com"},
            {"token": "wrong"},
            {"host": "example.com:8765"},
        ):
            handler, manager = self.handler(**kwargs)
            handler.do_POST()
            self.assertEqual(handler.respond.call_args.args[0], 403)
            manager.start.assert_not_called()

    def test_valid_local_request_calls_only_the_experiment_manager(self):
        handler, manager = self.handler()
        manager.start.return_value = "job"
        handler.do_POST()
        manager.start.assert_called_once_with({})
        self.assertEqual(handler.respond.call_args.args[0], 202)

    def test_arbitrary_file_paths_are_not_served(self):
        handler, manager = self.handler()
        handler.path = "/../../etc/passwd"
        handler.do_GET()
        self.assertEqual(handler.respond.call_args.args[0], 404)


class P54PanelTests(unittest.TestCase):
    def test_lowered_shift_has_fixed_depth_no_push_and_signed_direction(self):
        payload = dict(
            mode="lowered_shift",
            force_n=0,
            balance_enabled=False,
            lowering_mm=40,
            shift_side="right",
        )
        cmd = ExperimentRequest.parse(payload).command(ROOT, Path("/tmp/lowered-shift"))
        self.assertEqual(cmd[cmd.index("--lowered-shift") + 1], "-0.005")
        larger = ExperimentRequest.parse(dict(payload, shift_mm=10)).command(
            ROOT, Path("/tmp/lowered-shift10")
        )
        self.assertEqual(larger[larger.index("--lowered-shift") + 1], "-0.01")
        next_step = ExperimentRequest.parse(
            dict(payload, shift_mm=20, shift_side="left")
        ).command(ROOT, Path("/tmp/lowered-shift20"))
        self.assertEqual(next_step[next_step.index("--lowered-shift") + 1], "0.02")
        for side, value in [("left", "0.04"), ("right", "-0.04")]:
            next_step = ExperimentRequest.parse(
                dict(payload, shift_mm=40, shift_side=side)
            ).command(ROOT, Path("/tmp/lowered-shift30"))
            self.assertEqual(next_step[next_step.index("--lowered-shift") + 1], value)
        self.assertNotIn("--pulse-force-y", cmd)
        for update in [
            dict(lowering_mm=30),
            dict(force_n=1),
            dict(shift_side="forward"),
            dict(balance_enabled=True),
            dict(shift_mm=True),
            dict(shift_mm=50),
            dict(shift_mm=float("nan")),
        ]:
            with self.assertRaises(ValueError):
                ExperimentRequest.parse(dict(payload, **update))

    def test_larger_transfer_requires_its_own_measured_verification(self):
        with tempfile.TemporaryDirectory() as d:
            m = JobManager(d)
            # Historical requests omit shift_mm and must remain 5 mm evidence.
            m.jobs["old"] = dict(
                request=dict(mode="lowered_shift", shift_side="left"),
                result=dict(passed=True),
                status="completed",
            )
            find = lambda mm: next(
                s
                for s in m.roadmap()
                if s.get("mode") == "lowered_shift"
                and s.get("shift_side") == "left"
                and s.get("shift_mm") == mm
            )
            self.assertEqual(find(5)["status"], "verified")
            self.assertEqual(find(10)["status"], "ready to test")
            m.jobs["new"] = dict(
                request=dict(mode="lowered_shift", shift_side="left", shift_mm=10),
                result=dict(passed=False),
                status="completed",
            )
            self.assertEqual(find(10)["status"], "ready to test")
            m.jobs["new"]["result"]["passed"] = True
            self.assertEqual(find(10)["status"], "verified")
            self.assertEqual(find(20)["status"], "ready to test")
            m.jobs["next"] = dict(
                request=dict(mode="lowered_shift", shift_side="left", shift_mm=20),
                result=dict(passed=False),
                status="completed",
            )
            self.assertEqual(find(20)["status"], "ready to test")
            m.jobs["next"]["result"]["passed"] = True
            self.assertEqual(find(20)["status"], "verified")
            self.assertEqual(find(30)["status"], "ready to test")
            m.jobs["thirty"] = dict(
                request=dict(mode="lowered_shift", shift_side="left", shift_mm=30),
                result=dict(passed=True),
                status="completed",
            )
            self.assertEqual(find(30)["status"], "verified")
            self.assertEqual(find(40)["status"], "ready to test")
            m.jobs["forty"] = dict(
                request=dict(mode="lowered_shift", shift_side="left", shift_mm=40),
                result=dict(passed=True),
                status="completed",
            )
            self.assertEqual(find(40)["status"], "verified")

    def test_lowering_milestones_use_no_push_runner(self):
        payload = dict(
            mode="lowering",
            force_n=0,
            balance_enabled=False,
            lowering_mm=20,
            gazebo_gui=True,
        )
        config = ExperimentRequest.parse(payload)
        cmd = config.command(ROOT, Path("/tmp/lowering"))
        self.assertEqual(cmd[cmd.index("--lowering") + 1], "0.02")
        self.assertIn("--gui", cmd)
        self.assertNotIn("--pulse-force-y", cmd)
        self.assertNotIn("--load-feedback", cmd)
        fourth = ExperimentRequest.parse(dict(payload, lowering_mm=40)).command(
            ROOT, Path("/tmp/lowering40")
        )
        self.assertEqual(fourth[fourth.index("--lowering") + 1], "0.04")
        for update in [
            dict(lowering_mm=60),
            dict(lowering_mm=True),
            dict(force_n=1),
            dict(balance_enabled=True),
            dict(gazebo_gui="yes"),
        ]:
            with self.assertRaises(ValueError):
                ExperimentRequest.parse(dict(payload, **update))

    def test_roadmap_requires_a_passing_saved_run(self):
        with tempfile.TemporaryDirectory() as d:
            m = JobManager(d)
            self.assertEqual(m.roadmap()[2]["status"], "next experiment")
            m.jobs["fake"] = dict(
                request=dict(mode="lowering", lowering_mm=20),
                result=dict(passed=False),
                status="completed",
            )
            self.assertEqual(m.roadmap()[2]["status"], "next experiment")
            m.jobs["fake"]["result"]["passed"] = True
            self.assertEqual(m.roadmap()[2]["status"], "verified")

    def test_new_mode_selects_real_p54_runner_and_independent_force_direction(self):
        r = ExperimentRequest.parse(
            dict(
                mode="load_transfer",
                direction="right",
                loaded_side="left",
                force_n=1.0,
                duration_s=0.2,
            )
        )
        cmd = r.command(ROOT, Path("/tmp/p54-panel-test"))
        self.assertIn("robotis_p54", cmd)
        self.assertIn("--load-feedback", cmd)
        self.assertEqual(cmd[cmd.index("--direction") + 1], "left")
        self.assertEqual(cmd[cmd.index("--pulse-force-y") + 1], "-1.0")
        ref = ExperimentRequest.parse(
            dict(
                mode="load_transfer",
                direction="left",
                force_n=0.0,
                balance_enabled=False,
            )
        )
        self.assertNotIn("--load-feedback", ref.command(ROOT, Path("/tmp/reference")))
        for update in [
            dict(force_n=2),
            dict(duration_s=0.3),
            dict(direction="forward"),
        ]:
            payload = dict(mode="load_transfer", direction="left", force_n=1.0)
            payload.update(update)
            with self.assertRaises(ValueError):
                ExperimentRequest.parse(payload)

    def test_p54_completed_failed_checks_remain_visible_as_completed(self):
        with tempfile.TemporaryDirectory() as d:
            code = "import pathlib,json,sys;p=pathlib.Path(sys.argv[1]);p.mkdir();(p/'summary.json').write_text(json.dumps({'completed':True,'passed':False,'actuator_profile':{'name':'robotis_p54'}}));sys.exit(2)"
            m = JobManager(
                d, lambda config, output: [sys.executable, "-c", code, str(output)]
            )
            job = m.start(dict(mode="load_transfer", direction="left", force_n=1.0))
            deadline = time.monotonic() + 8
            while m.active_id is not None and time.monotonic() < deadline:
                time.sleep(0.03)
            self.assertEqual(m.jobs[job]["status"], "completed")
            self.assertFalse(m.jobs[job]["result"]["passed"])


class LoweredUnloadingPanelTests(unittest.TestCase):
    def test_target_command_and_strict_request(self):
        payload = dict(
            mode="lowered_unload",
            force_n=0,
            balance_enabled=False,
            lowering_mm=40,
            shift_side="left",
            unload_percent=20,
        )
        cmd = ExperimentRequest.parse(payload).command(
            ROOT, Path("/tmp/test-unloading")
        )
        self.assertEqual(cmd[cmd.index("--lowered-unload") + 1], "0.2")
        self.assertEqual(cmd[cmd.index("--direction") + 1], "left")
        for change in [
            dict(unload_percent=True),
            dict(unload_percent=0),
            dict(lowering_mm=20),
            dict(force_n=1),
        ]:
            with self.assertRaises(ValueError):
                ExperimentRequest.parse(dict(payload, **change))

    def test_later_target_requires_matching_support_side(self):
        payload = dict(
            mode="lowered_unload",
            force_n=0,
            balance_enabled=False,
            lowering_mm=40,
            shift_side="left",
            unload_percent=10,
        )
        with tempfile.TemporaryDirectory() as d:
            m = JobManager(d)
            with self.assertRaisesRegex(ValueError, "20%"):
                m.start(payload)
            m.jobs["previous"] = dict(
                status="completed",
                result=dict(passed=True),
                request=dict(
                    mode="lowered_unload", unload_percent=20, shift_side="right"
                ),
            )
            with self.assertRaisesRegex(ValueError, "20%"):
                m.start(payload)
            from unittest.mock import patch

            m.jobs["previous"]["request"]["shift_side"] = "left"
            stage = next(
                r
                for r in m.roadmap()
                if r.get("mode") == "lowered_unload"
                and r.get("unload_percent") == 10
                and r.get("shift_side") == "left"
            )
            self.assertEqual(stage["status"], "ready to test")
            with patch("force_panel.jobs.threading.Thread") as worker:
                job = m.start(payload)
                worker.return_value.start.assert_called_once()
                self.assertEqual(m.jobs[job]["request"]["unload_percent"], 10)


class ContactBenchPanelTests(unittest.TestCase):
    def test_diagnostic_has_no_robot_command(self):
        payload = dict(
            mode="contact_bench", force_n=0, balance_enabled=False, gazebo_gui=False
        )
        cmd = ExperimentRequest.parse(payload).command(ROOT, Path("/tmp/contact-bench"))
        self.assertEqual(Path(cmd[1]).name, "run_contact_transition_bench.py")
        self.assertEqual(cmd[2:], ["--output", "/tmp/contact-bench"])
        for change in [
            dict(force_n=1),
            dict(force_n=False),
            dict(balance_enabled=0),
            dict(balance_enabled=True),
        ]:
            with self.assertRaises(ValueError):
                ExperimentRequest.parse(dict(payload, **change))

    def test_sensor_failure_is_distinct_from_robot_recovery(self):
        self.assertIn(
            "sensor diagnostic passed",
            completion_message(dict(bench_kind="contact_transition", passed=True)),
        )
        self.assertIn(
            "did not pass",
            completion_message(
                dict(
                    bench_kind="contact_transition",
                    passed=False,
                    error="No fresh air message",
                )
            ),
        )


class ZeroLoadPanelTests(unittest.TestCase):
    def test_zero_mode_requires_sensor_and_same_side_unloading(self):
        payload = dict(
            mode="zero_load",
            force_n=0,
            balance_enabled=False,
            lowering_mm=40,
            shift_side="left",
        )
        cmd = ExperimentRequest.parse(payload).command(ROOT, Path("/tmp/zero-load"))
        self.assertIn("--zero-load", cmd)
        self.assertNotIn("--lowered-unload", cmd)
        with tempfile.TemporaryDirectory() as d:
            m = JobManager(d)
            with self.assertRaisesRegex(ValueError, "physics contact"):
                m.start(payload)
            m.jobs["sensor"] = dict(
                status="completed",
                request=dict(mode="contact_bench"),
                result=dict(passed=True, contact_source="state"),
            )
            m.jobs["unload"] = dict(
                status="completed",
                request=dict(
                    mode="lowered_unload", shift_side="right", unload_percent=5
                ),
                result=dict(passed=True),
            )
            with self.assertRaisesRegex(ValueError, "same-side"):
                m.start(payload)
            m.jobs["unload"]["request"]["shift_side"] = "left"
            from unittest.mock import patch

            with patch("force_panel.jobs.threading.Thread") as worker:
                m.start(payload)
                worker.return_value.start.assert_called_once()

    def test_roadmap_keeps_older_verification_after_twenty_runs(self):
        with tempfile.TemporaryDirectory() as d:
            store = Path(d) / "evidence/gui/.panel"
            store.mkdir(parents=True)
            for i in range(25):
                id = f"20260101-000000-{i:08x}"
                r = dict(
                    id=id,
                    status="completed",
                    request=dict(mode="lowering", lowering_mm=20 if i == 0 else 10),
                    result=dict(passed=True),
                )
                (store / (id + ".json")).write_text(json.dumps(r))
            m = JobManager(d)
            self.assertEqual(len(m.jobs), 25)
            self.assertEqual(m.roadmap()[2]["status"], "verified")
