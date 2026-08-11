#!/usr/bin/env python3

from pathlib import Path
import tempfile
import textwrap
import unittest

from run_experiment import (
    ExperimentError,
    case_command,
    load_config,
    lqr_fingerprint,
    write_turn_envelope_report,
)
from plot_trajectory import load_paths, write_svg


class ExperimentConfigTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        (self.root / "model.xml").write_text("<mujoco/>\n", encoding="ascii")
        schedule = self.root / "schedule"
        schedule.mkdir()
        (schedule / "current_model_schedule.h").write_text(
            "// test\n", encoding="ascii")
        (schedule / "current_model_schedule.json").write_text(
            "{}\n", encoding="ascii")
        (self.root / "parameters.json").write_text("{}\n", encoding="ascii")

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def write_config(self, contents: str) -> Path:
        path = self.root / "experiment.toml"
        path.write_text(textwrap.dedent(contents), encoding="ascii")
        return path

    def existing_config(self, extra: str = "") -> str:
        return f"""
            version = 1
            name = "test-experiment"

            [paths]
            model = "model.xml"
            output_root = "results"

            [build]
            root = "build"

            [controller]

            [lqr]
            mode = "existing"
            schedule_dir = "schedule"

            [[case]]
            name = "forward-test"
            axis = "forward"
            target = 2.0
            command_rate = 1.0
            {extra}
        """

    def test_existing_config_defaults_and_relative_paths(self) -> None:
        config = load_config(self.write_config(self.existing_config()))
        self.assertEqual(config.paths.model, self.root / "model.xml")
        self.assertEqual(config.build.configuration, "Release")
        self.assertEqual(config.controller.leg_length, 0.18)
        self.assertEqual(config.controller.trace_stride, 10)
        self.assertEqual(
            config.controller.yaw_acceleration_feedforward_scale, 0.9)
        self.assertFalse(config.analysis.forward_linear)
        self.assertEqual(config.cases[0].target_hold_seconds, 3.0)
        self.assertEqual(config.cases[0].standing_seconds, 2.0)
        self.assertEqual(config.lqr.schedule_dir, self.root / "schedule")

    def test_forward_analysis_uses_drive_policy(self) -> None:
        contents = self.existing_config().replace(
            "[[case]]",
            "[analysis]\nforward_linear = true\n\n[[case]]")
        config = load_config(self.write_config(contents))
        self.assertTrue(config.analysis.forward_linear)

    def test_forward_analysis_requires_forward_case(self) -> None:
        contents = self.existing_config().replace(
            "[[case]]",
            "[analysis]\nforward_linear = true\n\n[[case]]").replace(
                'axis = "forward"', 'axis = "heading"')
        with self.assertRaisesRegex(ExperimentError, "forward case"):
            load_config(self.write_config(contents))

    def test_legacy_yaw_axis_is_rejected(self) -> None:
        contents = self.existing_config().replace(
            'axis = "forward"', 'axis = "yaw"')
        with self.assertRaisesRegex(ExperimentError, "forward, heading, or turn"):
            load_config(self.write_config(contents))

    def test_turn_sweep_expands_cartesian_product(self) -> None:
        contents = self.existing_config().split("[[case]]", 1)[0] + """
            [[turn_sweep]]
            name = "coarse-turn"
            forward_velocities = [1, 1.5, 2, 2.5, 3]
            yaw_rates = [-4.71238898038469, -3.14159265358979,
                         -1.5707963267949, 1.5707963267949,
                         3.14159265358979, 4.71238898038469]
            forward_rate = 5
            yaw_rate = 10
            standing_seconds = 8
            target_hold_seconds = 2
            stop_settle_seconds = 3
        """
        config = load_config(self.write_config(contents))
        self.assertEqual(len(config.cases), 30)
        self.assertTrue(all(case.kind == "turn" for case in config.cases))
        self.assertEqual(config.cases[0].forward_target, 1.0)
        self.assertEqual(config.cases[-1].yaw_target, 4.71238898038469)
        command = case_command(
            self.root / "benchmark", config, config.cases[0],
            self.root / "output")
        self.assertIn("--forward-target", command)
        self.assertIn("--yaw-target", command)
        self.assertNotIn("--axis", command)

    def test_turn_report_is_dependency_free_svg(self) -> None:
        summary = self.root / "summary.csv"
        summary.write_text(
            "case,kind,forward_target,yaw_target,valid,response_pass,"
            "contact_free,unsaturated,actual_forward_mean,actual_yaw_mean,"
            "lateral_acceleration_mean,issue\n"
            "turn,turn,2,3.14,1,1,0,1,1.9,3.0,5.7,none\n",
            encoding="ascii")
        output = self.root / "turn.svg"
        write_turn_envelope_report(summary, output)
        document = output.read_text(encoding="ascii")
        self.assertIn("Steady-turn envelope", document)
        self.assertIn("contact event", document)

    def test_trajectory_report_compares_reference_and_actual(self) -> None:
        trace = self.root / "trace.csv"
        trace.write_text(
            "phase,simulation_time,axle_x,axle_y,psi,ref_ds,ref_dpsi\n"
            "figure_eight_straight_one,0.0,1.0,2.0,0.0,1.0,0.0\n"
            "figure_eight_straight_one,0.1,1.1,2.0,0.0,1.0,0.0\n",
            encoding="ascii")
        planned, actual = load_paths(trace)
        self.assertAlmostEqual(planned[-1][0], 0.1)
        self.assertAlmostEqual(actual[-1][0], 0.1)
        output = self.root / "trajectory.svg"
        write_svg(output, planned, actual)
        self.assertIn(
            "actual axle path", output.read_text(encoding="ascii"))

    def test_generate_config_reads_full_lqr_candidate(self) -> None:
        config = load_config(self.write_config("""
            version = 1
            name = "generated-test"

            [paths]
            model = "model.xml"
            output_root = "results"

            [build]
            root = "build"
            configuration = "Debug"

            [controller]

            [lqr]
            mode = "generate"

            [lqr.generate]
            model_parameters = "parameters.json"
            q_diagonal = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
            r_diagonal = [1, 2, 3, 4]
            leg_angle_difference_weight = 20
            leg_angular_velocity_difference_weight = 8

            [[case]]
            name = "heading-test"
            axis = "heading"
            target = -3.0
            command_rate = 2.0
        """))
        self.assertEqual(config.build.configuration, "Debug")
        self.assertEqual(config.lqr.generate.q_diagonal[-1], 10.0)

    def test_lqr_weight_change_changes_fingerprint(self) -> None:
        first = load_config(self.write_config("""
            version = 1
            name = "generated-test"
            [paths]
            model = "model.xml"
            output_root = "results"
            [build]
            root = "build"
            [controller]
            [lqr]
            mode = "generate"
            [lqr.generate]
            model_parameters = "parameters.json"
            q_diagonal = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
            r_diagonal = [1, 2, 3, 4]
            leg_angle_difference_weight = 20
            leg_angular_velocity_difference_weight = 8
            [[case]]
            name = "forward-test"
            axis = "forward"
            target = 1
            command_rate = 1
        """))
        first_fingerprint = lqr_fingerprint(first)
        second_path = self.root / "second.toml"
        second_path.write_text(
            (self.root / "experiment.toml").read_text(encoding="ascii")
            .replace(
                "q_diagonal = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]",
                "q_diagonal = [2, 2, 3, 4, 5, 6, 7, 8, 9, 10]"),
            encoding="ascii")
        second = load_config(second_path)
        self.assertNotEqual(first_fingerprint, lqr_fingerprint(second))

    def test_duplicate_case_is_rejected(self) -> None:
        contents = self.existing_config() + """
            [[case]]
            name = "forward-test"
            axis = "forward"
            target = -2.0
            command_rate = 1.0
        """
        with self.assertRaisesRegex(ExperimentError, "duplicate case"):
            load_config(self.write_config(contents))

    def test_feedback_override_is_rejected(self) -> None:
        contents = self.existing_config().replace(
            "[controller]", "[controller]\nposition_feedback = false")
        with self.assertRaisesRegex(ExperimentError, "unknown controller keys"):
            load_config(self.write_config(contents))

    def test_unknown_key_is_rejected(self) -> None:
        contents = self.existing_config().replace(
            "root = \"build\"", "root = \"build\"\nunknown = 1")
        with self.assertRaisesRegex(ExperimentError, "unknown build keys"):
            load_config(self.write_config(contents))

    def test_repository_formal_lqr_validation(self) -> None:
        config_path = (
            Path(__file__).resolve().parent /
            "formal_lqr_validation.toml")
        config = load_config(config_path)
        self.assertEqual(config.controller.trace_stride, 1)
        self.assertTrue(config.analysis.forward_linear)
        self.assertEqual(
            [(case.target, case.command_rate) for case in config.cases],
            [(3.0, 5.0), (-3.0, 5.0)])


if __name__ == "__main__":
    unittest.main()
