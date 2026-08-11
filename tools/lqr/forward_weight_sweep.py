#!/usr/bin/env python3
"""Screen forward LQR weights with the fixed-length linear model."""

from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import itertools
import json
from pathlib import Path

import numpy as np

from forward_response import (
    COMPARISON_PHASES,
    DS_INDEX,
    DTHETA_L_INDEX,
    DTHETA_R_INDEX,
    MOTION_PHASES,
    S_INDEX,
    THETA_B_INDEX,
    THETA_L_INDEX,
    THETA_R_INDEX,
    TRIM_AVERAGE_SECONDS,
    directed_response_metrics,
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
    ds_weight: float,
    wheel_weight: float,
    theta_weight: float,
    dtheta_weight: float,
) -> tuple[np.ndarray, float]:
    controller = document["controller"]
    q_diagonal = list(controller["q_diagonal"])
    q_diagonal[DS_INDEX] = ds_weight
    q_diagonal[THETA_B_INDEX] = theta_weight
    q_diagonal[THETA_B_INDEX + 1] = dtheta_weight
    r_diagonal = list(controller["r_diagonal"])
    r_diagonal[0] = wheel_weight
    r_diagonal[1] = wheel_weight
    settings = LqrSettings(
        length_min=0.0,
        length_max=0.0,
        sample_count=1,
        timestep=float(controller["timestep"]),
        q_diagonal=tuple(float(value) for value in q_diagonal),
        r_diagonal=tuple(float(value) for value in r_diagonal),
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
    wheel_torque_values: list[float] = []
    leg_torque_values: list[float] = []
    common_leg_values: list[float] = []
    common_leg_rate_values: list[float] = []
    response_times: list[float] = []
    response_references: list[float] = []
    response_ds: list[float] = []
    response_s: list[float] = []

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
        wheel_torque_values.extend(abs(value) for value in control[:2])
        leg_torque_values.extend(abs(value) for value in control[2:])
        common_leg_values.append(0.5 * (
            trim_state[THETA_L_INDEX] + trim_state[THETA_R_INDEX] +
            state[THETA_L_INDEX] + state[THETA_R_INDEX]))
        common_leg_rate_values.append(0.5 * (
            state[DTHETA_L_INDEX] + state[DTHETA_R_INDEX]))
        if row["phase"] in MOTION_PHASES:
            response_times.append(time - start_time)
            response_references.append(reference[DS_INDEX])
            response_ds.append(state[DS_INDEX])
            response_s.append(state[S_INDEX])
        previous_time = time
        previous_reference = reference

    response = directed_response_metrics(
        response_times, response_references,
        response_ds, response_ds, response_s, response_s)
    direction = float(np.copysign(1.0, response["target_velocity"]))
    target_magnitude = abs(response["target_velocity"])
    directed_ds = [direction * value for value in response_ds]
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
        "peak_wheel_torque": max(wheel_torque_values),
        "peak_leg_torque": max(leg_torque_values),
        "peak_common_leg_deg": np.degrees(max(
            abs(value) for value in common_leg_values)),
        "peak_common_leg_rate": max(
            abs(value) for value in common_leg_rate_values),
        "t10": response["predicted_t10"],
        "t50": response["predicted_t50"],
        "t90": response["predicted_t90"],
        "reverse_displacement": response["predicted_reverse_displacement"],
        "velocity_overshoot": max(
            0.0, max(directed_ds) - target_magnitude),
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
    parser.add_argument("--ds-weights", type=float, nargs="+")
    parser.add_argument("--wheel-weights", type=float, nargs="+")
    parser.add_argument("--theta-weights", type=float, nargs="+")
    parser.add_argument("--dtheta-weights", type=float, nargs="+")
    parser.add_argument("--pitch-limit-deg", type=float, default=2.5)
    parser.add_argument("--t90-limit", type=float, default=1.0)
    parser.add_argument("--velocity-overshoot-limit", type=float, default=0.3)
    parser.add_argument("--leg-length", type=float, default=0.18)
    arguments = parser.parse_args()

    if arguments.leg_length <= 0.0:
        parser.error("leg length must be positive")
    try:
        controller = json.loads(
            arguments.schedule.read_text(encoding="ascii"))["controller"]
        ds_weights = parse_weights(
            arguments.ds_weights or [controller["q_diagonal"][DS_INDEX]],
            "ds")
        wheel_weights = parse_weights(
            arguments.wheel_weights or [controller["r_diagonal"][0]],
            "wheel")
        theta_weights = parse_weights(
            arguments.theta_weights or [
                controller["q_diagonal"][THETA_B_INDEX]],
            "theta")
        dtheta_weights = parse_weights(
            arguments.dtheta_weights or [
                controller["q_diagonal"][THETA_B_INDEX + 1]],
            "dtheta")
    except ValueError as error:
        parser.error(str(error))
    if arguments.pitch_limit_deg <= 0.0 or arguments.t90_limit <= 0.0 or (
        arguments.velocity_overshoot_limit < 0.0
    ):
        parser.error("screening limits must be positive")

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
    candidates = itertools.product(
        ds_weights, wheel_weights, theta_weights, dtheta_weights)
    for ds_weight, wheel_weight, theta_weight, dtheta_weight in candidates:
        gain, maximum_eigenvalue = candidate_gain(
            document, matrix_a, matrix_b,
            ds_weight, wheel_weight, theta_weight, dtheta_weight)
        candidate_rows = []
        for name, case_rows in grouped.items():
            metrics = simulate_case(
                name, case_rows, matrix_a, matrix_b, gain, timestep)
            row = {
                "ds_weight": ds_weight,
                "wheel_weight": wheel_weight,
                "theta_weight": theta_weight,
                "dtheta_weight": dtheta_weight,
                "maximum_eigenvalue": maximum_eigenvalue,
                **metrics,
            }
            rows.append(row)
            candidate_rows.append(row)
        worst_t90 = max(float(row["t90"]) for row in candidate_rows)
        maximum_pitch = max(
            max(float(row["motion_peak_pitch_deg"]),
                float(row["stop_peak_pitch_deg"]))
            for row in candidate_rows)
        maximum_overshoot = max(
            float(row["velocity_overshoot"])
            for row in candidate_rows)
        maximum_wheel = max(float(
            row["peak_wheel_torque"]) for row in candidate_rows)
        maximum_leg = max(float(
            row["peak_leg_torque"]) for row in candidate_rows)
        passed = (
            maximum_eigenvalue < 1.0 and
            np.isfinite(worst_t90) and
            worst_t90 <= arguments.t90_limit and
            maximum_pitch <= arguments.pitch_limit_deg and
            maximum_overshoot <= arguments.velocity_overshoot_limit and
            maximum_wheel <= 6.32 and maximum_leg <= 40.0)
        rankings.append({
            "ds_weight": ds_weight,
            "wheel_weight": wheel_weight,
            "theta_weight": theta_weight,
            "dtheta_weight": dtheta_weight,
            "maximum_eigenvalue": maximum_eigenvalue,
            "passed": int(passed),
            "worst_t90": worst_t90,
            "max_reverse_displacement": max(float(
                row["reverse_displacement"]) for row in candidate_rows),
            "max_velocity_overshoot": maximum_overshoot,
            "max_peak_common_leg_deg": max(float(
                row["peak_common_leg_deg"]) for row in candidate_rows),
            "max_peak_common_leg_rate": max(float(
                row["peak_common_leg_rate"]) for row in candidate_rows),
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
            "max_peak_wheel_torque": maximum_wheel,
            "max_peak_leg_torque": maximum_leg,
        })

    rankings.sort(key=lambda row: (
        -row["passed"], row["worst_t90"],
        row["max_reverse_displacement"], row["max_peak_wheel_torque"]))
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

    print(
        " ds  wheel theta dtheta pass  t90 reverse pitch leg_angle "
        "wheel_torque leg_torque")
    for row in rankings:
        print(
            f"{row['ds_weight']:3.0f} {row['wheel_weight']:6.2f} "
            f"{row['theta_weight']:5.0f} {row['dtheta_weight']:6.0f} "
            f"{row['passed']:4d} {row['worst_t90']:5.3f} "
            f"{row['max_reverse_displacement']:7.4f} "
            f"{max(row['max_motion_peak_pitch_deg'], row['max_stop_peak_pitch_deg']):5.2f} "
            f"{row['max_peak_common_leg_deg']:9.2f} "
            f"{row['max_peak_wheel_torque']:11.3f} "
            f"{row['max_peak_leg_torque']:10.3f}")
    print(f"Wrote {arguments.output / 'ranking.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
