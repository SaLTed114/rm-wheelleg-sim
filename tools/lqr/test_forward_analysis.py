#!/usr/bin/env python3

from pathlib import Path
import sys
import unittest

import numpy as np


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIRECTORY))

from forward_response import (  # noqa: E402
    axle_midpoint_velocity,
    directed_response_metrics,
    load_model,
)
from forward_weight_sweep import candidate_gain  # noqa: E402


SCHEDULE = SCRIPT_DIRECTORY / "generated" / "current_model_schedule.json"


class ForwardAnalysisTest(unittest.TestCase):
    def test_current_schedule_loads(self) -> None:
        matrix_a, matrix_b, gain, timestep = load_model(SCHEDULE, 0.18)
        self.assertEqual(matrix_a.shape, (10, 10))
        self.assertEqual(matrix_b.shape, (10, 4))
        self.assertEqual(gain.shape, (4, 10))
        self.assertEqual(timestep, 0.001)
        self.assertTrue(np.isfinite(matrix_a).all())

    def test_axle_truth_averages_wheel_center_sites(self) -> None:
        row = {
            "wheel_center_velocity_l": "1.5",
            "wheel_center_velocity_r": "2.5",
        }
        self.assertEqual(axle_midpoint_velocity(row), 2.0)
        with self.assertRaisesRegex(ValueError, "wheel-center"):
            axle_midpoint_velocity({})

    def test_directed_response_handles_reverse_and_relative_position(self) -> None:
        metrics = directed_response_metrics(
            [0.0, 0.1, 0.2, 0.3],
            [0.0, -1.0, -2.0, -2.0],
            [0.0, -0.4, -1.2, -1.9],
            [0.0, -0.3, -1.1, -1.8],
            [4.0, 4.01, 3.95, 3.80],
            [8.0, 8.02, 7.97, 7.82],
        )
        self.assertEqual(metrics["target_velocity"], -2.0)
        self.assertAlmostEqual(metrics["actual_t10"], 0.1)
        self.assertAlmostEqual(metrics["actual_t50"], 0.2)
        self.assertAlmostEqual(metrics["actual_t90"], 0.3)
        self.assertAlmostEqual(metrics["actual_reverse_displacement"], 0.01)
        self.assertAlmostEqual(
            metrics["predicted_reverse_displacement"], 0.02)

    def test_ds_and_wheel_weights_change_candidate_gain(self) -> None:
        import json

        document = json.loads(SCHEDULE.read_text(encoding="ascii"))
        matrix_a, matrix_b, baseline, _ = load_model(SCHEDULE, 0.18)
        candidate, maximum_eigenvalue = candidate_gain(
            document, matrix_a, matrix_b,
            ds_weight=120.0,
            wheel_weight=2.4,
            theta_weight=300.0,
            dtheta_weight=60.0,
        )
        self.assertFalse(np.allclose(candidate, baseline))
        self.assertLess(maximum_eigenvalue, 1.0)


if __name__ == "__main__":
    unittest.main()
