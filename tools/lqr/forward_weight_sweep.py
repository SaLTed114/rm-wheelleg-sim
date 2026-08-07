#!/usr/bin/env python3
"""Screen body pitch LQR weights with the fixed-length linear model."""

from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path

import numpy as np

from forward_response import (
    COMPARISON_PHASES,
    DS_INDEX,
    MOTION_PHASES,
    S_INDEX,
    THETA_B_INDEX,
    TRIM_AVERAGE_SECONDS,
    load_model,
    s_feedback_disabled,
    trace_vector,
)
from lqr_generator import (
    LqrSettings,
    build_state_cost_matrix,
    solve_lqr,
)


DEFAULT_SCHEDULE = Path("tools/lqr/generated/current_model_schedule.json")


def root_mean_square(values: list[float]) -> float:
    array = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(np.square(array))))


def candidate_gain(
    document: dict[str, object],
    matrix_a: np.ndarray,
    matrix_b: np.ndarray,
    theta_weight: float,
    dtheta_weight: float,
) -> tuple[np.ndarray, float]:
    controller = document["controller"]
    q_diagonal = list(controller["q_diagonal"])
    q_diagonal[THETA_B_INDEX] = theta_weight
    q_diagonal[THETA_B_INDEX + 1] = dtheta_weight
    settings = LqrSettings(
        length_min=0.0,
        length_max=0.0,
        sample_count=1,
        timestep=float(controller["timestep"]),
        q_diagonal=tuple(float(value) for value in q_diagonal),
        r_diagonal=tuple(float(value) for value in controller["r_diagonal"]),
        leg_angle_difference_weight=float(
            controller["leg_angle_cost"]["difference"]),
        leg_angular_velocity_difference_weight=float(
            controller["leg_angular_velocity_cost"]["difference"]),
    )
    matrix_q = build_state_cost_matrix(settings) * settings.timestep
    matrix_r = np.diag(settings.r_diagonal) * settings.timestep
    gain, _, eigenvalues, _ = solve_lqr(
        matrix_a, matrix_b, matrix_q, matrix_r)
    return gain, float(np.max(np.abs(eigenvalues)))


def simulate_case(
    name: str,
    case_rows: list[dict[str, str]],
    matrix_a: np.ndarray,
    matrix_b: np.ndarray,
    gain: np.ndarray,
    timestep: float,
) -> dict[str, float | str]:
    rows = [row for row in case_rows if row["phase"] in COMPARISON_PHASES]
    if not rows:
        raise ValueError(f"case {name} has no forward-motion samples")
    for row in rows:
        s_feedback_disabled(row)

    start_time = float(rows[0]["simulation_time"])
    trim_rows = [
        row for row in case_rows
        if row["phase"] == "standing"
        and float(row["simulation_time"]) >= start_time - TRIM_AVERAGE_SECONDS
    ]
    if not trim_rows:
        raise ValueError(f"case {name} has no standing trim samples")

    trim_state = np.mean([trace_vector(row) for row in trim_rows], axis=0)
    trim_reference = np.mean(
        [trace_vector(row, "ref_") for row in trim_rows], axis=0)
    state = trace_vector(rows[0]) - trim_state
    previous_reference = trace_vector(rows[0], "ref_") - trim_reference
    previous_time = start_time
    phase_values = {
        "motion": {"pitch": [], "ds_error": []},
        "stop": {"pitch": [], "ds_error": []},
    }
    common_wheel_values: list[float] = []

    for index, row in enumerate(rows):
        time = float(row["simulation_time"])
        reference = trace_vector(row, "ref_") - trim_reference
        if index != 0:
            interval_steps = max(1, int(round((time - previous_time) / timestep)))
            for interval_step in range(interval_steps):
                alpha = (interval_step + 1) / interval_steps
                interpolated_reference = (
                    previous_reference + alpha * (reference - previous_reference))
                if s_feedback_disabled(row):
                    interpolated_reference[S_INDEX] = state[S_INDEX]
                control = gain @ (interpolated_reference - state)
                state = matrix_a @ state + matrix_b @ control

        effective_reference = reference.copy()
        if s_feedback_disabled(row):
            effective_reference[S_INDEX] = state[S_INDEX]
        control = gain @ (effective_reference - state)
        phase = "motion" if row["phase"] in MOTION_PHASES else "stop"
        phase_values[phase]["pitch"].append(
            trim_state[THETA_B_INDEX] + state[THETA_B_INDEX])
        phase_values[phase]["ds_error"].append(
            state[DS_INDEX] - effective_reference[DS_INDEX])
        common_wheel_values.append(0.5 * (control[0] + control[1]))
        previous_time = time
        previous_reference = reference

    return {
        "case": name,
        "motion_peak_pitch_deg": np.degrees(max(
            abs(value) for value in phase_values["motion"]["pitch"])),
        "stop_peak_pitch_deg": np.degrees(max(
            abs(value) for value in phase_values["stop"]["pitch"])),
        "motion_ds_rms_error": root_mean_square(
            phase_values["motion"]["ds_error"]),
        "stop_ds_rms_error": root_mean_square(
            phase_values["stop"]["ds_error"]),
        "peak_common_wheel": max(abs(value) for value in common_wheel_values),
        "final_ds_error": phase_values["stop"]["ds_error"][-1],
        "final_pitch_deg": np.degrees(phase_values["stop"]["pitch"][-1]),
    }


def parse_weights(values: list[float], name: str) -> list[float]:
    if any(value <= 0.0 for value in values):
        raise ValueError(f"{name} weights must be positive")
    return sorted(set(values))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--schedule", type=Path, default=DEFAULT_SCHEDULE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mujoco-trace", type=Path, nargs="+", required=True)
    parser.add_argument("--theta-weights", type=float, nargs="+", required=True)
    parser.add_argument("--dtheta-weights", type=float, nargs="+", required=True)
    parser.add_argument("--leg-length", type=float, default=0.18)
    arguments = parser.parse_args()

    if arguments.leg_length <= 0.0:
        parser.error("leg length must be positive")
    try:
        theta_weights = parse_weights(arguments.theta_weights, "theta")
        dtheta_weights = parse_weights(arguments.dtheta_weights, "dtheta")
    except ValueError as error:
        parser.error(str(error))

    document = json.loads(arguments.schedule.read_text(encoding="ascii"))
    matrix_a, matrix_b, _, timestep = load_model(
        arguments.schedule, arguments.leg_length)
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for trace_path in arguments.mujoco_trace:
        with trace_path.open(newline="", encoding="ascii") as source:
            for row in csv.DictReader(source):
                grouped[row["case"]].append(row)
    if not grouped:
        raise ValueError("no cases found in the supplied traces")

    rows: list[dict[str, float | str]] = []
    rankings: list[dict[str, float]] = []
    for theta_weight in theta_weights:
        for dtheta_weight in dtheta_weights:
            gain, maximum_eigenvalue = candidate_gain(
                document, matrix_a, matrix_b, theta_weight, dtheta_weight)
            candidate_rows = []
            for name, case_rows in grouped.items():
                metrics = simulate_case(
                    name, case_rows, matrix_a, matrix_b, gain, timestep)
                row = {
                    "theta_weight": theta_weight,
                    "dtheta_weight": dtheta_weight,
                    "maximum_eigenvalue": maximum_eigenvalue,
                    **metrics,
                }
                rows.append(row)
                candidate_rows.append(row)
            rankings.append({
                "theta_weight": theta_weight,
                "dtheta_weight": dtheta_weight,
                "maximum_eigenvalue": maximum_eigenvalue,
                "max_motion_peak_pitch_deg": max(float(
                    row["motion_peak_pitch_deg"]) for row in candidate_rows),
                "max_stop_peak_pitch_deg": max(float(
                    row["stop_peak_pitch_deg"]) for row in candidate_rows),
                "max_motion_ds_rms_error": max(float(
                    row["motion_ds_rms_error"]) for row in candidate_rows),
                "max_stop_ds_rms_error": max(float(
                    row["stop_ds_rms_error"]) for row in candidate_rows),
                "max_peak_common_wheel": max(float(
                    row["peak_common_wheel"]) for row in candidate_rows),
            })

    rankings.sort(key=lambda row: (
        row["max_stop_peak_pitch_deg"], row["max_motion_peak_pitch_deg"]))
    arguments.output.mkdir(parents=True, exist_ok=True)
    with (arguments.output / "weight_sweep.csv").open(
        "w", newline="", encoding="ascii",
    ) as output:
        writer = csv.DictWriter(output, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    with (arguments.output / "ranking.csv").open(
        "w", newline="", encoding="ascii",
    ) as output:
        writer = csv.DictWriter(output, fieldnames=rankings[0].keys())
        writer.writeheader()
        writer.writerows(rankings)

    print("theta dtheta  motion  stop  ds_motion ds_stop wheel")
    for row in rankings:
        print(
            f"{row['theta_weight']:5.0f} {row['dtheta_weight']:6.0f} "
            f"{row['max_motion_peak_pitch_deg']:7.3f} "
            f"{row['max_stop_peak_pitch_deg']:6.3f} "
            f"{row['max_motion_ds_rms_error']:9.4f} "
            f"{row['max_stop_ds_rms_error']:7.4f} "
            f"{row['max_peak_common_wheel']:5.3f}")
    print(f"Wrote {arguments.output / 'ranking.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
