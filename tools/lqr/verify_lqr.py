#!/usr/bin/env python3
"""Verify the Python LQR generator against the real-vehicle coefficients."""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
import re

import numpy as np

from lqr_generator import (
    STATE_NAMES,
    build_state_cost_matrix,
    compute_gain_samples,
    fit_gain_schedule,
    legacy_leg_parameters,
    legacy_parameters,
    legacy_settings,
    verify_yaw_inertia_identity,
)


DEFAULT_REFERENCE = Path(
    "references/rm2026cb-balance-chassis/Tasks/balance_chassis/"
    "bc_lqr_schedule.c")
GOLDEN_TOLERANCE = 1.0e-6


def verify_leg_mode_costs() -> None:
    settings = replace(
        legacy_settings(),
        leg_angle_difference_weight=120.0,
        leg_angular_velocity_difference_weight=3.2)
    matrix_q = build_state_cost_matrix(settings)

    for left_name, right_name, common, difference in (
        ("theta_l", "theta_r", 30.0, 120.0),
        ("dtheta_l", "dtheta_r", 0.8, 3.2),
    ):
        left = STATE_NAMES.index(left_name)
        right = STATE_NAMES.index(right_name)
        common_mode = np.zeros(len(STATE_NAMES))
        difference_mode = np.zeros(len(STATE_NAMES))
        common_mode[left] = common_mode[right] = 1.0 / np.sqrt(2.0)
        difference_mode[left] = 1.0 / np.sqrt(2.0)
        difference_mode[right] = -1.0 / np.sqrt(2.0)
        if not np.isclose(common_mode @ matrix_q @ common_mode, common):
            raise AssertionError(f"{left_name} common-mode cost is incorrect")
        if not np.isclose(
            difference_mode @ matrix_q @ difference_mode, difference
        ):
            raise AssertionError(
                f"{left_name} difference-mode cost is incorrect")


def parse_reference(path: Path) -> tuple[np.ndarray, float, float]:
    source = path.read_text(encoding="utf-8")
    midpoint_match = re.search(
        r"K_LMID\s*=\s*([-+\d.eE]+)f", source)
    scale_match = re.search(
        r"K_LSCA\s*=\s*([-+\d.eE]+)f", source)
    if midpoint_match is None or scale_match is None:
        raise ValueError(f"could not find length normalization in {path}")

    start = source.index("Kcoef[")
    block = source[start:source.index("};", start)]
    rows = re.findall(r"\{([^{}]*\d[^{}]*)\}", block)
    coefficient_rows = []
    for row in rows:
        values = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", row)
        if len(values) >= 2:
            coefficient_rows.append([float(value) for value in values])
    coefficient = np.asarray(coefficient_rows, dtype=float)
    if coefficient.shape[0] != 40:
        raise ValueError(
            f"expected 40 coefficient rows in {path}, got {coefficient.shape[0]}")
    return (
        coefficient.reshape(4, 10, coefficient.shape[1]),
        float(midpoint_match.group(1)),
        float(scale_match.group(1)),
    )


def verify_legacy(reference: Path) -> dict[str, float]:
    if not verify_yaw_inertia_identity():
        raise AssertionError("yaw inertia identity did not reduce to -I_z*D2psi")
    verify_leg_mode_costs()

    expected, midpoint, scale = parse_reference(reference)
    samples = compute_gain_samples(
        legacy_parameters(), legacy_settings(), legacy_leg_parameters)
    schedule = fit_gain_schedule(samples, expected.shape[2] - 1)
    difference = np.abs(schedule.coefficient - expected)
    maximum_difference = float(np.max(difference))
    maximum_index = tuple(int(value) for value in np.unravel_index(
        int(np.argmax(difference)), difference.shape))

    if abs(schedule.length_midpoint - midpoint) > 1.0e-12:
        raise AssertionError("legacy K_LMID does not match the firmware")
    if abs(schedule.length_scale - scale) > 1.0e-12:
        raise AssertionError("legacy K_LSCA does not match the firmware")
    if maximum_difference > GOLDEN_TOLERANCE:
        raise AssertionError(
            f"legacy coefficient mismatch {maximum_difference:.6e} "
            f"at K{maximum_index}")
    if samples.minimum_controllability_rank != 10:
        raise AssertionError("legacy model is not controllable at every sample")
    if samples.maximum_eigenvalue >= 1.0:
        raise AssertionError("legacy raw LQR gains are not stable")
    if samples.maximum_are_residual > 1.0e-8:
        raise AssertionError("legacy Riccati residual exceeds tolerance")

    return {
        "maximum_coefficient_difference": maximum_difference,
        "maximum_fit_error": schedule.maximum_fit_error,
        "maximum_raw_eigenvalue": samples.maximum_eigenvalue,
        "maximum_are_residual": samples.maximum_are_residual,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--reference", type=Path, default=DEFAULT_REFERENCE,
        help="firmware bc_lqr_schedule.c used as the golden reference")
    arguments = parser.parse_args()

    metrics = verify_legacy(arguments.reference)
    print("Yaw inertia identity: -I_z * D2psi")
    print(
        "Legacy maximum coefficient difference: "
        f"{metrics['maximum_coefficient_difference']:.6e}")
    print(f"Legacy maximum K fit error: {metrics['maximum_fit_error']:.6e}")
    print(
        "Legacy maximum raw closed-loop |eigenvalue|: "
        f"{metrics['maximum_raw_eigenvalue']:.9f}")
    print(
        "Legacy maximum Riccati residual: "
        f"{metrics['maximum_are_residual']:.6e}")
    print("Legacy golden verification passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
