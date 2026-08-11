#!/usr/bin/env python3
"""Compare trim-aligned MuJoCo forward motion with the generated A/B/K."""

from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path

import numpy as np

from build_current_model import make_leg_provider, make_physical_parameters
from lqr_generator import (
    GainSchedule,
    INPUT_NAMES,
    STATE_NAMES,
    build_state_matrix_functions,
    discretize,
    evaluate_gain,
    evaluate_state_matrices,
)


DEFAULT_SCHEDULE = Path("tools/lqr/generated/current_model_schedule.json")
TRIM_AVERAGE_SECONDS = 1.0
PITCH_ERROR_THRESHOLD = np.radians(1.0)
MOTION_PHASES = ("target_ramp", "target_hold")
STOP_PHASES = ("yaw_stop_ramp", "forward_stop_ramp", "stop_settle")
COMPARISON_PHASES = MOTION_PHASES + STOP_PHASES

S_INDEX = STATE_NAMES.index("s")
DS_INDEX = STATE_NAMES.index("ds")
THETA_L_INDEX = STATE_NAMES.index("theta_l")
THETA_R_INDEX = STATE_NAMES.index("theta_r")
DTHETA_L_INDEX = STATE_NAMES.index("dtheta_l")
DTHETA_R_INDEX = STATE_NAMES.index("dtheta_r")
THETA_B_INDEX = STATE_NAMES.index("theta_b")


def load_model(
    schedule_path: Path,
    leg_length: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    document = json.loads(schedule_path.read_text(encoding="ascii"))
    controller = document["controller"]
    timestep = float(controller["timestep"])
    physical = make_physical_parameters(document["model"])
    leg = make_leg_provider(document["model"])(leg_length)

    functions = build_state_matrix_functions(physical)
    continuous_a, continuous_b = evaluate_state_matrices(
        functions, leg_length, leg)
    discrete_a, discrete_b = discretize(
        continuous_a, continuous_b, timestep)

    stored = document["schedule"]
    schedule = GainSchedule(
        coefficient=np.asarray(stored["coefficients"], dtype=float),
        polynomial_order=int(stored["polynomial_order"]),
        length_midpoint=float(stored["length_midpoint"]),
        length_scale=float(stored["length_scale"]),
        maximum_fit_error=0.0,
        maximum_dense_eigenvalue=0.0,
    )
    return discrete_a, discrete_b, evaluate_gain(
        schedule, leg_length), timestep


def trace_vector(row: dict[str, str], prefix: str = "") -> np.ndarray:
    return np.asarray([float(row[f"{prefix}{name}"]) for name in STATE_NAMES])


def s_feedback_disabled(row: dict[str, str]) -> bool:
    forward = row.get("forward")
    if forward:
        if forward not in ("idle", "hold", "velocity"):
            raise ValueError(f"unknown forward mode: {forward}")
        return forward == "velocity"

    drive = row.get("drive")
    if drive:
        if drive not in (
                "idle", "parked", "driving", "hold", "drive", "spin"):
            raise ValueError(f"unknown drive state: {drive}")
        return drive in ("driving", "drive")

    legacy = row.get("position_feedback_enabled")
    if legacy == "0":
        return True
    if legacy == "1":
        raise ValueError(
            "legacy trace must be recorded with position feedback off")
    raise ValueError(
        "trace must contain forward, drive, or position_feedback_enabled")


def root_mean_square(values: list[float]) -> float:
    array = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(np.square(array))))


def axle_midpoint_velocity(row: dict[str, str]) -> float:
    """Return MuJoCo truth at the point represented by the DS state."""
    try:
        left = float(row["wheel_center_velocity_l"])
        right = float(row["wheel_center_velocity_r"])
    except KeyError as error:
        raise ValueError(
            "trace must contain both wheel-center velocity columns"
        ) from error
    return 0.5 * (left + right)


def threshold_time(
    times: list[float], values: list[float], threshold: float,
) -> float:
    for time, value in zip(times, values):
        if value >= threshold:
            return time
    return float("nan")


def directed_response_metrics(
    times: list[float], reference: list[float], actual: list[float],
    predicted: list[float], actual_displacement: list[float],
    predicted_displacement: list[float],
) -> dict[str, float]:
    target_index = int(np.argmax(np.abs(reference)))
    target = reference[target_index]
    if target == 0.0:
        raise ValueError("forward trace has no non-zero velocity reference")
    direction = float(np.copysign(1.0, target))
    magnitude = abs(target)
    directed_actual = [direction * value for value in actual]
    directed_predicted = [direction * value for value in predicted]
    actual_s_origin = actual_displacement[0]
    predicted_s_origin = predicted_displacement[0]
    directed_actual_s = [
        direction * (value - actual_s_origin)
        for value in actual_displacement]
    directed_predicted_s = [
        direction * (value - predicted_s_origin)
        for value in predicted_displacement]
    result: dict[str, float] = {
        "target_velocity": target,
        "actual_reverse_displacement": max(
            0.0, -min(directed_actual_s)),
        "predicted_reverse_displacement": max(
            0.0, -min(directed_predicted_s)),
    }
    for percentage in (10, 50, 90):
        threshold = magnitude * percentage / 100.0
        result[f"actual_t{percentage}"] = threshold_time(
            times, directed_actual, threshold)
        result[f"predicted_t{percentage}"] = threshold_time(
            times, directed_predicted, threshold)
    return result


def compare_case(
    name: str,
    case_rows: list[dict[str, str]],
    matrix_a: np.ndarray,
    matrix_b: np.ndarray,
    gain: np.ndarray,
    timestep: float,
) -> tuple[dict[str, float | str], list[list[float | str]]]:
    rows = [
        row for row in case_rows
        if row["phase"] in COMPARISON_PHASES
    ]
    if not rows:
        raise ValueError(f"case {name} has no forward-motion samples")
    for row in rows:
        s_feedback_disabled(row)

    start_time = float(rows[0]["simulation_time"])
    trim_rows = [
        row for row in case_rows
        if row["phase"] == "standing" and
        float(row["simulation_time"]) >=
            start_time - TRIM_AVERAGE_SECONDS
    ]
    if not trim_rows:
        raise ValueError(f"case {name} has no standing trim samples")

    trim_state = np.mean(
        [trace_vector(row) for row in trim_rows], axis=0)
    trim_reference = np.mean(
        [trace_vector(row, "ref_") for row in trim_rows], axis=0)
    trim_wheel = np.mean([
        0.5 * (float(row["raw_wheel_l"]) + float(row["raw_wheel_r"]))
        for row in trim_rows
    ])

    state = trace_vector(rows[0]) - trim_state
    previous_reference = trace_vector(rows[0], "ref_") - trim_reference
    previous_time = start_time
    comparison_rows: list[list[float | str]] = []
    ds_errors: list[float] = []
    pitch_errors: list[float] = []
    common_leg_errors: list[float] = []
    common_leg_rate_errors: list[float] = []
    common_wheel_errors: list[float] = []
    reconstruction_errors: list[float] = []
    actual_pitch_values: list[float] = []
    predicted_pitch_values: list[float] = []
    actual_wheel_values: list[float] = []
    predicted_wheel_values: list[float] = []
    actual_common_leg_values: list[float] = []
    predicted_common_leg_values: list[float] = []
    actual_common_leg_rate_values: list[float] = []
    predicted_common_leg_rate_values: list[float] = []
    observer_errors: list[float] = []
    response_times: list[float] = []
    response_references: list[float] = []
    response_actual: list[float] = []
    response_predicted: list[float] = []
    response_actual_displacement: list[float] = []
    response_predicted_displacement: list[float] = []
    actual_displacement = 0.0
    previous_axle_truth = axle_midpoint_velocity(rows[0])
    phase_metrics: dict[str, dict[str, list[float]]] = {
        label: {
            "ds_errors": [],
            "pitch_errors": [],
            "actual_pitch": [],
            "predicted_pitch": [],
        }
        for label in ("motion", "stop")
    }
    first_pitch_error = float("nan")
    contact_steps = 0
    other_contact_steps = 0

    for index, row in enumerate(rows):
        time = float(row["simulation_time"])
        reference = trace_vector(row, "ref_") - trim_reference
        if index != 0:
            interval_steps = max(
                1, int(round((time - previous_time) / timestep)))
            for interval_step in range(interval_steps):
                alpha = (interval_step + 1) / interval_steps
                interpolated_reference = (
                    previous_reference +
                    alpha * (reference - previous_reference))
                if s_feedback_disabled(row):
                    interpolated_reference[S_INDEX] = state[S_INDEX]
                control = gain @ (interpolated_reference - state)
                state = matrix_a @ state + matrix_b @ control

        effective_reference = reference.copy()
        if s_feedback_disabled(row):
            effective_reference[S_INDEX] = state[S_INDEX]
        predicted_control = gain @ (effective_reference - state)
        actual = trace_vector(row)
        predicted = trim_state + state
        axle_truth = axle_midpoint_velocity(row)
        actual_common_leg = 0.5 * (
            actual[THETA_L_INDEX] + actual[THETA_R_INDEX])
        predicted_common_leg = 0.5 * (
            predicted[THETA_L_INDEX] + predicted[THETA_R_INDEX])
        actual_common_leg_rate = 0.5 * (
            actual[DTHETA_L_INDEX] + actual[DTHETA_R_INDEX])
        predicted_common_leg_rate = 0.5 * (
            predicted[DTHETA_L_INDEX] + predicted[DTHETA_R_INDEX])
        actual_common_wheel = 0.5 * (
            float(row["raw_wheel_l"]) + float(row["raw_wheel_r"]))
        actual_common_wheel_delta = actual_common_wheel - trim_wheel
        predicted_common_wheel_delta = 0.5 * (
            predicted_control[0] + predicted_control[1])
        actual_delta = actual - trim_state
        actual_effective_reference = reference.copy()
        if s_feedback_disabled(row):
            actual_effective_reference[S_INDEX] = actual_delta[S_INDEX]
        reconstructed_control = gain @ (
            actual_effective_reference - actual_delta)
        reconstructed_common_wheel_delta = 0.5 * (
            reconstructed_control[0] + reconstructed_control[1])

        ds_error = axle_truth - predicted[DS_INDEX]
        observer_error = actual[DS_INDEX] - axle_truth
        pitch_error = actual[THETA_B_INDEX] - predicted[THETA_B_INDEX]
        common_leg_error = actual_common_leg - predicted_common_leg
        common_leg_rate_error = (
            actual_common_leg_rate - predicted_common_leg_rate)
        common_wheel_error = (
            actual_common_wheel_delta - predicted_common_wheel_delta)
        reconstruction_error = (
            actual_common_wheel_delta - reconstructed_common_wheel_delta)
        relative_time = time - start_time

        ds_errors.append(ds_error)
        observer_errors.append(observer_error)
        pitch_errors.append(pitch_error)
        common_leg_errors.append(common_leg_error)
        common_leg_rate_errors.append(common_leg_rate_error)
        common_wheel_errors.append(common_wheel_error)
        reconstruction_errors.append(reconstruction_error)
        actual_pitch_values.append(actual[THETA_B_INDEX])
        predicted_pitch_values.append(predicted[THETA_B_INDEX])
        actual_wheel_values.append(actual_common_wheel_delta)
        predicted_wheel_values.append(predicted_common_wheel_delta)
        actual_common_leg_values.append(actual_common_leg)
        predicted_common_leg_values.append(predicted_common_leg)
        actual_common_leg_rate_values.append(actual_common_leg_rate)
        predicted_common_leg_rate_values.append(predicted_common_leg_rate)
        phase_label = "motion" if row["phase"] in MOTION_PHASES else "stop"
        metrics = phase_metrics[phase_label]
        metrics["ds_errors"].append(ds_error)
        metrics["pitch_errors"].append(pitch_error)
        metrics["actual_pitch"].append(actual[THETA_B_INDEX])
        metrics["predicted_pitch"].append(predicted[THETA_B_INDEX])
        contact_steps += int(
            row["contact_wheel_l"] == "1" and
            row["contact_wheel_r"] == "1")
        other_contact_steps += int(row["other_contact"] == "1")
        if not np.isfinite(first_pitch_error) and abs(
            pitch_error
        ) > PITCH_ERROR_THRESHOLD:
            first_pitch_error = relative_time

        if row["phase"] in MOTION_PHASES:
            if response_times:
                interval = relative_time - response_times[-1]
                actual_displacement += 0.5 * (
                    previous_axle_truth + axle_truth) * interval
            response_times.append(relative_time)
            response_references.append(
                trim_reference[DS_INDEX] + reference[DS_INDEX])
            response_actual.append(axle_truth)
            response_predicted.append(predicted[DS_INDEX])
            response_actual_displacement.append(actual_displacement)
            response_predicted_displacement.append(
                predicted[S_INDEX] - trim_state[S_INDEX])
            previous_axle_truth = axle_truth

        comparison_rows.append([
            name,
            relative_time,
            row["phase"],
            axle_truth,
            actual[DS_INDEX],
            predicted[DS_INDEX],
            np.degrees(actual[THETA_B_INDEX]),
            np.degrees(predicted[THETA_B_INDEX]),
            np.degrees(actual_common_leg),
            np.degrees(predicted_common_leg),
            actual_common_wheel_delta,
            predicted_common_wheel_delta,
            reconstructed_common_wheel_delta,
            int(row["contact_wheel_l"]),
            int(row["contact_wheel_r"]),
            int(row["other_contact"]),
        ])
        previous_time = time
        previous_reference = reference

    response_metrics = directed_response_metrics(
        response_times, response_references,
        response_actual, response_predicted,
        response_actual_displacement,
        response_predicted_displacement)
    actual_peak_pitch = np.degrees(
        max(abs(value) for value in actual_pitch_values))
    predicted_peak_pitch = np.degrees(
        max(abs(value) for value in predicted_pitch_values))
    actual_peak_common_leg = np.degrees(max(
        abs(value) for value in actual_common_leg_values))
    predicted_peak_common_leg = np.degrees(max(
        abs(value) for value in predicted_common_leg_values))
    actual_peak_common_wheel = max(
        abs(value) for value in actual_wheel_values)
    predicted_peak_common_wheel = max(
        abs(value) for value in predicted_wheel_values)
    t90_error = abs(
        response_metrics["actual_t90"] - response_metrics["predicted_t90"])
    pitch_peak_error = abs(actual_peak_pitch - predicted_peak_pitch)
    common_leg_peak_error = abs(
        actual_peak_common_leg - predicted_peak_common_leg)
    common_wheel_peak_error = abs(
        actual_peak_common_wheel - predicted_peak_common_wheel)

    summary: dict[str, float | str] = {
        "case": name,
        "trim_ds": trim_state[DS_INDEX],
        "trim_pitch_deg": np.degrees(trim_state[THETA_B_INDEX]),
        "trim_common_leg_deg": np.degrees(0.5 * (
            trim_state[THETA_L_INDEX] + trim_state[THETA_R_INDEX])),
        "ds_rms_error": root_mean_square(ds_errors),
        "ds_max_error": max(abs(value) for value in ds_errors),
        "observer_ds_rms_error": root_mean_square(observer_errors),
        "observer_ds_max_error": max(abs(value) for value in observer_errors),
        "pitch_rms_error_deg": np.degrees(root_mean_square(pitch_errors)),
        "pitch_max_error_deg": np.degrees(
            max(abs(value) for value in pitch_errors)),
        "first_pitch_error_time": first_pitch_error,
        "actual_peak_pitch_deg": actual_peak_pitch,
        "predicted_peak_pitch_deg": predicted_peak_pitch,
        "pitch_peak_error_deg": pitch_peak_error,
        "common_leg_rms_error_deg": np.degrees(
            root_mean_square(common_leg_errors)),
        "common_leg_max_error_deg": np.degrees(
            max(abs(value) for value in common_leg_errors)),
        "common_leg_rate_rms_error": root_mean_square(
            common_leg_rate_errors),
        "common_leg_rate_max_error": max(
            abs(value) for value in common_leg_rate_errors),
        "actual_peak_common_leg_deg": actual_peak_common_leg,
        "predicted_peak_common_leg_deg": predicted_peak_common_leg,
        "common_leg_peak_error_deg": common_leg_peak_error,
        "actual_peak_common_leg_rate": max(
            abs(value) for value in actual_common_leg_rate_values),
        "predicted_peak_common_leg_rate": max(
            abs(value) for value in predicted_common_leg_rate_values),
        "common_wheel_rms_error": root_mean_square(common_wheel_errors),
        "common_wheel_max_error": max(
            abs(value) for value in common_wheel_errors),
        "controller_reconstruction_rms_error": root_mean_square(
            reconstruction_errors),
        "controller_reconstruction_max_error": max(
            abs(value) for value in reconstruction_errors),
        "actual_peak_common_wheel": actual_peak_common_wheel,
        "predicted_peak_common_wheel": predicted_peak_common_wheel,
        "common_wheel_peak_error": common_wheel_peak_error,
        "dual_wheel_contact_ratio": contact_steps / len(rows),
        "other_contact_steps": other_contact_steps,
        **response_metrics,
        "t90_error": t90_error,
    }
    for label, metrics in phase_metrics.items():
        summary[f"{label}_ds_rms_error"] = root_mean_square(
            metrics["ds_errors"])
        summary[f"{label}_pitch_rms_error_deg"] = np.degrees(
            root_mean_square(metrics["pitch_errors"]))
        summary[f"{label}_actual_peak_pitch_deg"] = np.degrees(
            max(abs(value) for value in metrics["actual_pitch"]))
        summary[f"{label}_predicted_peak_pitch_deg"] = np.degrees(
            max(abs(value) for value in metrics["predicted_pitch"]))
    return summary, comparison_rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--schedule", type=Path, default=DEFAULT_SCHEDULE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mujoco-trace", type=Path, nargs="+", required=True)
    parser.add_argument("--leg-length", type=float, default=0.18)
    arguments = parser.parse_args()

    if arguments.leg_length <= 0.0:
        parser.error("leg length must be positive")

    matrix_a, matrix_b, gain, timestep = load_model(
        arguments.schedule, arguments.leg_length)
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for trace_path in arguments.mujoco_trace:
        with trace_path.open(newline="", encoding="ascii") as source:
            for row in csv.DictReader(source):
                grouped[row["case"]].append(row)

    summaries: list[dict[str, float | str]] = []
    comparisons: list[list[float | str]] = []
    for name, rows in grouped.items():
        summary, comparison = compare_case(
            name, rows, matrix_a, matrix_b, gain, timestep)
        summaries.append(summary)
        comparisons.extend(comparison)

    if not summaries:
        raise ValueError("no cases found in the supplied traces")

    arguments.output.mkdir(parents=True, exist_ok=True)
    summary_path = arguments.output / "comparison_summary.csv"
    with summary_path.open("w", newline="", encoding="ascii") as output:
        writer = csv.DictWriter(output, fieldnames=summaries[0].keys())
        writer.writeheader()
        writer.writerows(summaries)

    comparison_path = arguments.output / "comparison.csv"
    with comparison_path.open("w", newline="", encoding="ascii") as output:
        writer = csv.writer(output)
        writer.writerow([
            "case", "time", "phase",
            "axle_truth_ds", "estimated_ds", "predicted_ds",
            "actual_pitch_deg", "predicted_pitch_deg",
            "actual_common_leg_deg", "predicted_common_leg_deg",
            "actual_common_wheel_delta", "predicted_common_wheel_delta",
            "reconstructed_common_wheel_delta",
            "contact_wheel_l", "contact_wheel_r", "other_contact",
        ])
        writer.writerows(comparisons)

    for summary in summaries:
        print(
            f"{summary['case']:<22} "
            f"actual/model t90={summary['actual_t90']:.3f}/"
            f"{summary['predicted_t90']:.3f}s "
            f"motion_pitch={summary['motion_actual_peak_pitch_deg']:.2f}/"
            f"{summary['motion_predicted_peak_pitch_deg']:.2f} deg "
            f"stop_pitch={summary['stop_actual_peak_pitch_deg']:.2f}/"
            f"{summary['stop_predicted_peak_pitch_deg']:.2f} deg")
    print(f"Wrote {summary_path}")
    print(f"Wrote {comparison_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
