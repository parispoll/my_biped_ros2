"""Regression checks for test lateral shift. See the folder README for its role."""

from pathlib import Path
import unittest
import xacro
from my_robot_description.motion.lateral_shift import (
    LateralPlanner,
    ShiftProgram,
    TOTAL_TIME,
)
from my_robot_description.model.roll_stance import (
    JOINT_NAMES,
    MOTOR_CAPS,
    standing_plan,
)
from my_robot_description.control.roll_posture import LATERAL_KI
from my_robot_description.model.roll_geometry import link_transforms

ROOT = Path(__file__).resolve().parents[1]


class LateralShiftTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.xml = xacro.process_file(
            str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
        ).toxml()
        cls.planner = LateralPlanner(cls.xml)
        cls.program = ShiftProgram(cls.xml)

    def test_zero_shift_reproduces_verified_stance(self):
        p = self.planner.pose(0)
        old = standing_plan(self.xml)
        for key in ["target_rad", "feedforward_nm"]:
            for a, b in zip(p[key], old[key]):
                self.assertAlmostEqual(a, b, places=12)

    def test_fixed_flat_feet_and_load_budgets_in_both_directions(self):
        for d in [-0.01, -0.005, 0, 0.005, 0.01]:
            p = self.planner.pose(d)
            self.assertLess(p["max_fixed_foot_geometry_error"], 1e-12)
            self.assertTrue(
                all(abs(v) < cap for v, cap in zip(p["feedforward_nm"], MOTOR_CAPS))
            )
            self.assertAlmostEqual(
                sum(p["predicted_force_n"].values()), 14.42 * 9.81, places=9
            )
            if d:
                self.assertGreater(p["predicted_com_y_m"] * d, 0)

    def test_mirrored_com_and_predicted_loads(self):
        a = self.planner.pose(0.01)
        b = self.planner.pose(-0.01)
        self.assertAlmostEqual(
            a["predicted_com_y_m"], -b["predicted_com_y_m"], places=12
        )
        self.assertAlmostEqual(
            a["predicted_force_n"]["left"], b["predicted_force_n"]["right"], places=12
        )

    def test_program_timing_and_return(self):
        p = self.program
        for t, phase in [
            (0, "baseline"),
            (3, "move_out"),
            (7, "hold"),
            (13, "move_back"),
            (17, "recovery"),
            (25, "done"),
        ]:
            self.assertEqual(p.sample(t)["phase"], phase)
        self.assertEqual(p.sample(25)["pelvis_y_m"], 0)
        self.assertEqual(p.sample(9)["pelvis_y_m"], 0.01)
        self.assertAlmostEqual(p.sample(5)["pelvis_y_m"], 0.005)
        self.assertEqual(TOTAL_TIME, 25)

    def test_interpolated_trajectory_keeps_feet_within_micron(self):
        for t in [3.01, 3.7, 4.4, 5.9, 6.99, 14.1, 16.9]:
            point = self.program.sample(t)
            frames = link_transforms(
                self.xml, dict(zip(JOINT_NAMES, point["target_rad"]))
            )
            for s in ["left", "right"]:
                old = self.planner.feet[f"{s}_foot"]
                new = frames[f"{s}_foot"]
                for i in range(3):
                    self.assertLess(
                        abs(
                            new[i][3]
                            + (point["pelvis_y_m"] if i == 1 else 0)
                            - old[i][3]
                        ),
                        1e-6,
                    )

    def test_invalid_shifts_and_times(self):
        for d in [0, 0.02, float("nan")]:
            with self.assertRaises(ValueError):
                ShiftProgram(self.xml, d)
        for t in [-1, float("nan")]:
            with self.assertRaises(ValueError):
                self.program.sample(t)


class LateralAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from my_robot_description.model.roll_geometry import model_com

        cls.xml = xacro.process_file(
            str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
        ).toxml()
        p = ShiftProgram(cls.xml)
        cls.rows = []
        cls.com = []
        base = p.planner.base
        for i in range(501):
            t = i * 0.05
            point = p.sample(t)
            q = point["target_rad"]
            ff = point["feedforward_nm"]
            cls.rows.append(
                dict(
                    sim_time=t,
                    active=True,
                    ready=True,
                    shift=point,
                    integral_gains=LATERAL_KI,
                    target_rad=q,
                    position=q,
                    velocity=[0.0] * 10,
                    error=[0.0] * 10,
                    feedforward=ff,
                    integral=[0.0] * 10,
                    command=ff.copy(),
                    requested=ff.copy(),
                    shift_contacts={
                        s: dict(ground=True, age_s=0.001) for s in ["left", "right"]
                    },
                )
            )
            mass, c = model_com(cls.xml, dict(zip(JOINT_NAMES, q)))
            c[1] += point["pelvis_y_m"]
            c[2] += base["expected_pelvis_height_m"]
            cls.com.append(
                dict(
                    sim_time=t,
                    mass_kg=mass,
                    com=c,
                    feet={
                        s: [
                            [p.planner.foot_x + x, y + dy, 0.0]
                            for x, dy in [
                                (-0.1, -0.06),
                                (0.1, -0.06),
                                (0.1, 0.06),
                                (-0.1, 0.06),
                            ]
                        ]
                        for s, y in [("left", 0.085), ("right", -0.085)]
                    },
                )
            )

    def test_accepts_model_consistent_com_shift_but_does_not_claim_forces_measured(
        self,
    ):
        from my_robot_description.validation.lateral_audit import audit_shift

        report = audit_shift(self.rows, self.com, self.xml, 0.01)
        self.assertTrue(report["passed"])
        self.assertFalse(report["foot_forces_measured"])
        self.assertAlmostEqual(
            report["measured_com_shift_relative_feet_m"],
            report["predicted_com_shift_m"],
            places=10,
        )

    def test_rejects_opposite_motion_and_corrupt_motor_command(self):
        import copy
        from my_robot_description.validation.lateral_audit import audit_shift

        observations = copy.deepcopy(self.com)
        for r in observations:
            r["com"][1] *= -1
        self.assertFalse(audit_shift(self.rows, observations, self.xml, 0.01)["passed"])
        rows = copy.deepcopy(self.rows)
        rows[200]["command"][0] += 1
        self.assertFalse(audit_shift(rows, self.com, self.xml, 0.01)["passed"])


class LateralGainTests(unittest.TestCase):
    def test_shift_gains_are_explicit_and_default_standing_gains_unchanged(self):
        from my_robot_description.control.roll_posture import RollPosture, KI

        xml = xacro.process_file(
            str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
        ).toxml()
        plan = standing_plan(xml)
        ordinary = RollPosture(plan)
        shift = RollPosture(plan, integral_gains=LATERAL_KI)
        q = [v - 0.001 for v in plan["target_rad"]]
        ordinary.update(q, [0.0] * 10, 0.01)
        shift.update(q, [0.0] * 10, 0.01)
        self.assertEqual(ordinary.ki, KI)
        self.assertEqual(KI[0], 20.0)
        self.assertAlmostEqual(ordinary.integral[0], 0.0002)
        self.assertAlmostEqual(shift.integral[0], 0.00005)
        for gains in [[1.0] * 6, [-1.0] * 10]:
            with self.assertRaises(ValueError):
                RollPosture(plan, integral_gains=gains)


class LateralSensorAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rows = []
        for i in range(501):
            t = i * 0.05
            cls.rows.append(
                dict(
                    sim_time=t,
                    pose=dict(
                        sim_time=t,
                        position=[0.0, 0.01 if 7 <= t < 13 else 0.0, 0.55449],
                        tilt_radians=0.0,
                    ),
                    imu=dict(sim_time=t),
                    contacts={
                        s: dict(sim_time=t, ground=True) for s in ["left", "right"]
                    },
                )
            )
        cls.imu = [
            dict(
                sim_time=i * 0.01,
                tilt_radians=0.0,
                angular_speed=0.0,
                acceleration=[0.0, 0.0, 9.81],
            )
            for i in range(2501)
        ]

    def test_sensor_coverage_and_pelvis_shift_are_checked(self):
        from my_robot_description.validation.lateral_audit import audit_sensors

        r = audit_sensors(self.rows, self.imu, 0, 25, 0.55449, 0.01)
        self.assertTrue(r["passed"])
        self.assertAlmostEqual(r["measured_pelvis_shift_m"], 0.01)

    def test_stale_cached_sample_and_imu_spike_are_not_hidden_by_means(self):
        import copy
        from my_robot_description.validation.lateral_audit import audit_sensors

        rows = copy.deepcopy(self.rows)
        rows[200]["imu"]["sim_time"] -= 0.06
        self.assertFalse(audit_sensors(rows, self.imu, 0, 25, 0.55449, 0.01)["passed"])
        imu = copy.deepcopy(self.imu)
        imu[1500]["acceleration"][1] = 0.26
        self.assertFalse(audit_sensors(self.rows, imu, 0, 25, 0.55449, 0.01)["passed"])

    def test_height_reference_uses_baseline_for_fresh_pre_window_pose(self):
        import copy
        from my_robot_description.validation.lateral_audit import audit_sensors

        class Reference:
            def sample(self, t):
                assert t >= 0
                return dict(pelvis_height_m=0.55449)

        rows = copy.deepcopy(self.rows)
        rows[0]["pose"]["sim_time"] = -0.005
        self.assertTrue(
            audit_sensors(
                rows, self.imu, 0, 25, 0.55449, 0.01, height_program=Reference()
            )["passed"]
        )
        rows[0]["pose"]["sim_time"] = -0.2
        self.assertFalse(
            audit_sensors(
                rows, self.imu, 0, 25, 0.55449, 0.01, height_program=Reference()
            )["passed"]
        )
