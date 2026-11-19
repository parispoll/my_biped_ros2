"""Detect dropped inputs, hidden modes, limit contact and invalid torque accounting."""

import unittest
from my_robot_description.validation.physics_audit import (
    audit_joint,
    audit_explicit_joint,
    split_step_balance,
    final_getter_window,
    stationary_physics_window,
)


class PhysicsAuditTests(unittest.TestCase):
    def test_static_gravity_comparison_requires_a_stationary_window(self):
        rows = [dict(q_rad=0.1, v_rad_s=0.0), dict(q_rad=0.1, v_rad_s=0.0)]
        self.assertTrue(stationary_physics_window([rows]))
        rows[-1]["v_rad_s"] = 0.0018
        self.assertFalse(stationary_physics_window([rows]))
        rows[-1].update(v_rad_s=0.0, q_rad=0.1001)
        self.assertFalse(stationary_physics_window([rows]))
        rows[-1]["q_rad"] = float("nan")
        with self.assertRaises(ValueError):
            stationary_physics_window([rows])

    def explicit_fixture(self):
        p, d, v = [], [], []
        for i in range(30):
            p.append(
                dict(
                    sim_time=i * 0.01,
                    input_last_nm=0.15,
                    ty_world=0.2,
                    external_fx_last_n=0.0,
                )
            )
            d.append(
                dict(
                    dt_s=0.001,
                    fd_command_nm=0.15,
                    fd_force_nm=0.15,
                    implicit_damping_nm=0.0,
                    damping_nm_s_rad=0.0,
                    constraint_impulse_nm_s=0.00005,
                    transmitted_ty_nm=0.2,
                    actuator_type=0,
                    friction_nm=0.2,
                )
            )
            v.append(
                dict(
                    sim_time=i * 0.01,
                    motor_nm=0.25,
                    torque=-0.1,
                    total_nm=0.15,
                    max_abs_motor_in_block_nm=0.25,
                )
            )
        return p, d, v

    def test_explicit_path_accounts_for_motor_plus_passive_torque(self):
        p, d, v = self.explicit_fixture()
        self.assertTrue(audit_explicit_joint(p, d, v, 0)["passed"])
        for r in p:
            r["input_last_nm"] = 0.25
        self.assertFalse(audit_explicit_joint(p, d, v, 0)["passed"])
        p, d, v = self.explicit_fixture()
        v[-1]["motor_nm"] = float("nan")
        with self.assertRaises(ValueError):
            audit_explicit_joint(p, d, v, 0)

    def test_explicit_path_rejects_hidden_backend_damping_or_overlimit_motor(self):
        p, d, v = self.explicit_fixture()
        d[0]["damping_nm_s_rad"] = 2.5
        self.assertFalse(audit_explicit_joint(p, d, v, 0)["passed"])
        p, d, v = self.explicit_fixture()
        v[0]["max_abs_motor_in_block_nm"] = 6.1
        self.assertFalse(audit_explicit_joint(p, d, v, 0)["passed"])
        p, d, v = self.explicit_fixture()
        v.pop()
        with self.assertRaisesRegex(ValueError, "matching"):
            audit_explicit_joint(p, d, v, 0)

    def test_getter_window_covers_one_second_at_supported_steps(self):
        for dt, expected in ((0.001, 101), (0.0005, 201), (0.00025, 401)):
            rows = [dict(sequence=i, dt_s=dt) for i in range(0, 4010, 10)]
            window = final_getter_window(rows)
            self.assertEqual(len(window), expected)
            self.assertAlmostEqual(
                (window[-1]["sequence"] - window[0]["sequence"]) * dt, 1.0
            )

    def test_getter_window_rejects_short_recording(self):
        with self.assertRaisesRegex(ValueError, "coverage"):
            final_getter_window(
                [dict(sequence=i, dt_s=0.0005) for i in range(0, 1010, 10)]
            )

    def test_getter_window_rejects_variable_step(self):
        with self.assertRaisesRegex(ValueError, "Variable"):
            final_getter_window(
                [dict(sequence=0, dt_s=0.001), dict(sequence=10, dt_s=0.0005)]
            )

    def test_getter_window_rejects_gaps(self):
        rows = [dict(sequence=i, dt_s=0.001) for i in range(0, 2010, 10) if i != 1500]
        with self.assertRaisesRegex(ValueError, "Missing"):
            final_getter_window(rows)

    def fixture(self):
        d = dict(
            dt_s=0.001,
            fd_command_nm=0.6286402,
            fd_force_nm=0.6286402,
            implicit_damping_nm=-0.5167648,
            constraint_impulse_nm_s=0.0001081235,
            transmitted_ty_nm=0.2199989,
            fd_velocity_rad_s=0.0,
            fd_acceleration_rad_s2=206.70592,
            v_rad_s=0.0,
            q_rad=-0.186,
            actuator_type=0,
            friction_nm=0.2,
            damping_nm_s_rad=2.5,
            force_nm=0.0,
        )
        p = dict(
            input_mean_nm=0.6286402,
            input_min_nm=0.6286402,
            input_max_nm=0.6286402,
            q_rad=-0.186,
            v_rad_s=0.0,
            lower_rad=-0.8,
            upper_rad=0.8,
            ty_world=0.2199989,
            external_fx_last_n=0.0,
        )
        c = dict(command=[0.6286402] * 6, position=[-0.186] * 6)
        return (
            [p.copy() for _ in range(30)],
            [d.copy() for _ in range(30)],
            [c.copy() for _ in range(30)],
        )

    def test_split_step_closes_with_nonzero_damping_at_zero_final_speed(self):
        p, d, c = self.fixture()
        result = audit_joint(p, d, c, 0)
        self.assertTrue(result["passed"])
        self.assertAlmostEqual(result["max_split_step_closure_error_nm"], 0)
        self.assertGreater(abs(result["implicit_damping_nm"]), 0.5)
        self.assertEqual(result["max_final_speed_rad_s"], 0)

    def test_invalid_timestep_or_force_is_rejected(self):
        _, d, _ = self.fixture()
        for key, value in [("dt_s", 0), ("dt_s", -1), ("fd_force_nm", float("nan"))]:
            row = d[0].copy()
            row[key] = value
            with self.assertRaises(ValueError):
                split_step_balance(row)

    def test_velocity_servo_mode_cannot_pass_as_force_mode(self):
        p, d, c = self.fixture()
        d[-1]["actuator_type"] = 2
        self.assertFalse(audit_joint(p, d, c, 0)["passed"])

    def test_dropped_physics_command_is_detected(self):
        p, d, c = self.fixture()
        for r in p:
            r["input_mean_nm"] = 0
        self.assertFalse(audit_joint(p, d, c, 0)["passed"])

    def test_pulsed_input_with_same_average_does_not_pass(self):
        p, d, c = self.fixture()
        for r in p:
            r.update(input_min_nm=0.0, input_max_nm=2 * 0.6286402)
        self.assertFalse(audit_joint(p, d, c, 0)["passed"])

    def test_constraint_above_nominal_coulomb_bound_is_detected(self):
        p, d, c = self.fixture()
        d[-1]["constraint_impulse_nm_s"] = 0.0003
        self.assertFalse(audit_joint(p, d, c, 0)["constraint_within_coulomb_bound"])

    def test_mismatched_snapshot_windows_are_rejected(self):
        p, d, c = self.fixture()
        for r in p:
            r["q_rad"] += 0.01
        with self.assertRaises(ValueError):
            audit_joint(p, d, c, 0)

    def test_pose_close_to_travel_stop_is_not_accepted(self):
        p, d, c = self.fixture()
        for r in p:
            r["lower_rad"] = -0.187
        self.assertFalse(audit_joint(p, d, c, 0)["passed"])
