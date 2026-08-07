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
)


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
            position_feedback = false

            [lqr]
            mode = "existing"
            schedule_dir = "schedule"

            [analysis]
            forward_linear = true

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
        self.assertTrue(config.controller.yaw_position_feedback)
        self.assertEqual(config.cases[0].target_hold_seconds, 3.0)
        self.assertEqual(config.cases[0].standing_seconds, 2.0)
        self.assertEqual(config.lqr.schedule_dir, self.root / "schedule")

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
            position_feedback = false
            yaw_position_feedback = false

            [lqr]
            mode = "generate"

            [lqr.generate]
            model_parameters = "parameters.json"
            q_diagonal = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
            r_diagonal = [1, 2, 3, 4]
            leg_angle_difference_weight = 20
            leg_angular_velocity_difference_weight = 8
            yaw_inertia_source = "assembly"

            [[case]]
            name = "yaw-test"
            axis = "yaw"
            target = -3.0
            command_rate = 2.0
        """))
        self.assertEqual(config.build.configuration, "Debug")
        self.assertFalse(config.controller.yaw_position_feedback)
        self.assertEqual(config.lqr.generate.q_diagonal[-1], 10.0)
        self.assertEqual(config.lqr.generate.yaw_inertia_source, "assembly")
        command = case_command(
            self.root / "benchmark", config, config.cases[0],
            self.root / "case-output")
        self.assertIn(
            ["--yaw-position-feedback", "off"],
            [command[index:index + 2] for index in range(len(command) - 1)])

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

    def test_forward_analysis_requires_position_feedback_off(self) -> None:
        contents = self.existing_config().replace(
            "position_feedback = false", "position_feedback = true")
        with self.assertRaisesRegex(ExperimentError, "position_feedback"):
            load_config(self.write_config(contents))

    def test_unknown_key_is_rejected(self) -> None:
        contents = self.existing_config().replace(
            "root = \"build\"", "root = \"build\"\nunknown = 1")
        with self.assertRaisesRegex(ExperimentError, "unknown build keys"):
            load_config(self.write_config(contents))


if __name__ == "__main__":
    unittest.main()
