#!/usr/bin/env python3
"""Predict fixed-length yaw responses with the generated A/B model and K."""

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
DEFAULT_ACCELERATIONS = (1.0, 2.0, 3.0, 5.0, 7.5, 10.0, 15.0)
WHEEL_TORQUE_LIMIT = 6.32
MODEL_ERROR_THRESHOLD = np.radians(2.0)
TRIM_AVERAGE_SECONDS = 1.0


def case_name(target_rate: float, acceleration: float) -> str:
    direction = "pos" if target_rate > 0.0 else "neg"
    target = f"{abs(target_rate) / np.pi:g}".replace(".", "p")
    rate = f"{acceleration:g}".replace(".", "p")
    return f"yaw_{direction}_{target}pi_a{rate}"


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


def run_case(
    matrix_a: np.ndarray,
    matrix_b: np.ndarray,
    gain: np.ndarray,
    timestep: float,
    target_rate: float,
    acceleration: float,
    hold_seconds: float,
) -> tuple[dict[str, float | str], list[list[float | str]]]:
    state = np.zeros(len(STATE_NAMES))
    reference = np.zeros(len(STATE_NAMES))
    ramp_seconds = abs(target_rate) / acceleration
    duration = ramp_seconds + hold_seconds
    steps = int(round(duration / timestep))
    rows: list[list[float | str]] = []

    maximum_common = 0.0
    maximum_difference = 0.0
    maximum_wheel = np.zeros(2)
    maximum_leg = np.zeros(2)
    first_wheel_limit = float("nan")
    name = case_name(target_rate, acceleration)

    for step in range(steps):
        time = (step + 1) * timestep
        yaw_rate = np.copysign(
            min(abs(target_rate), acceleration * time), target_rate)
        reference[3] = yaw_rate
        reference[2] += yaw_rate * timestep
        control = gain @ (reference - state)

        common = 0.5 * (state[4] + state[6])
        difference = 0.5 * (state[4] - state[6])
        maximum_common = max(maximum_common, abs(common))
        maximum_difference = max(maximum_difference, abs(difference))
        maximum_wheel = np.maximum(maximum_wheel, np.abs(control[:2]))
        maximum_leg = np.maximum(maximum_leg, np.abs(control[2:]))
        if not np.isfinite(first_wheel_limit) and np.any(
            np.abs(control[:2]) > WHEEL_TORQUE_LIMIT
        ):
            first_wheel_limit = time

        rows.append([
            name,
            time,
            "target_ramp" if time <= ramp_seconds else "target_hold",
            *state,
            *reference,
            *control,
        ])
        state = matrix_a @ state + matrix_b @ control

    summary: dict[str, float | str] = {
        "case": name,
        "acceleration": acceleration,
        "ramp_seconds": ramp_seconds,
        "peak_wheel_l": maximum_wheel[0],
        "peak_wheel_r": maximum_wheel[1],
        "first_wheel_limit_time": first_wheel_limit,
        "peak_leg_torque_l": maximum_leg[0],
        "peak_leg_torque_r": maximum_leg[1],
        "peak_leg_common_deg": np.degrees(maximum_common),
        "peak_leg_difference_deg": np.degrees(maximum_difference),
        "final_yaw_rate_error": state[3] - target_rate,
    }
    return summary, rows


def trace_vector(row: dict[str, str], prefix: str = "") -> np.ndarray:
    return np.asarray([float(row[f"{prefix}{name}"]) for name in STATE_NAMES])


def optional_float(row: dict[str, str], name: str) -> float:
    value = row.get(name)
    return 0.0 if value is None or value == "" else float(value)


def compare_mujoco_trace(
    trace_paths: list[Path],
    matrix_a: np.ndarray,
    matrix_b: np.ndarray,
    gain: np.ndarray,
    timestep: float,
) -> tuple[list[dict[str, float | str]], list[list[float | str]]]:
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for trace_path in trace_paths:
        with trace_path.open(newline="", encoding="ascii") as source:
            for row in csv.DictReader(source):
                grouped[row["case"]].append(row)

    summaries: list[dict[str, float | str]] = []
    comparisons: list[list[float | str]] = []
    for name, case_rows in grouped.items():
        rows = [
            row for row in case_rows
            if row["phase"] in ("target_ramp", "target_hold")
        ]
        if not rows:
            continue

        start_time = float(rows[0]["simulation_time"])
        trim_rows = [
            row for row in case_rows
            if row["phase"] == "standing" and
            float(row["simulation_time"]) >=
                start_time - TRIM_AVERAGE_SECONDS
        ]
        if trim_rows:
            trim_state = np.mean(
                [trace_vector(row) for row in trim_rows], axis=0)
            trim_reference = np.mean(
                [trace_vector(row, "ref_") for row in trim_rows], axis=0)
        else:
            trim_state = trace_vector(rows[0])
            trim_reference = trace_vector(rows[0], "ref_")

        state = trace_vector(rows[0]) - trim_state
        previous_reference = (
            trace_vector(rows[0], "ref_") - trim_reference)
        previous_time = start_time
        first_lift = float("nan")
        first_other = float("nan")
        first_wheel_limit = float("nan")
        first_common_error = float("nan")
        first_difference_error = float("nan")
        maximum_difference_error = 0.0
        maximum_common_error = 0.0

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
                    control = gain @ (interpolated_reference - state)
                    state = matrix_a @ state + matrix_b @ control

            actual = trace_vector(row)
            predicted = trim_state + state
            predicted_control = gain @ (
                trim_reference + reference - predicted)
            actual_common = 0.5 * (actual[4] + actual[6])
            actual_difference = 0.5 * (actual[4] - actual[6])
            predicted_common = 0.5 * (predicted[4] + predicted[6])
            predicted_difference = 0.5 * (predicted[4] - predicted[6])
            common_error = actual_common - predicted_common
            difference_error = actual_difference - predicted_difference
            relative_time = time - start_time
            maximum_common_error = max(
                maximum_common_error, abs(common_error))
            maximum_difference_error = max(
                maximum_difference_error, abs(difference_error))

            contact_l = int(row["contact_wheel_l"])
            contact_r = int(row["contact_wheel_r"])
            other_contact = int(row["other_contact"])
            raw_wheel_l = float(row["raw_wheel_l"])
            raw_wheel_r = float(row["raw_wheel_r"])
            if not np.isfinite(first_lift) and not (
                contact_l and contact_r
            ):
                first_lift = relative_time
            if not np.isfinite(first_other) and other_contact:
                first_other = relative_time
            if not np.isfinite(first_wheel_limit) and max(
                abs(raw_wheel_l), abs(raw_wheel_r)
            ) > WHEEL_TORQUE_LIMIT:
                first_wheel_limit = relative_time
            if not np.isfinite(first_common_error) and abs(
                common_error
            ) > MODEL_ERROR_THRESHOLD:
                first_common_error = relative_time
            if not np.isfinite(first_difference_error) and abs(
                difference_error
            ) > MODEL_ERROR_THRESHOLD:
                first_difference_error = relative_time

            comparisons.append([
                name,
                relative_time,
                row["phase"],
                actual_common,
                predicted_common,
                actual_difference,
                predicted_difference,
                raw_wheel_l,
                raw_wheel_r,
                predicted_control[0],
                predicted_control[1],
                contact_l,
                contact_r,
                other_contact,
                optional_float(row, "normal_force_l"),
                optional_float(row, "normal_force_r"),
            ])
            previous_time = time
            previous_reference = reference

        summaries.append({
            "case": name,
            "first_common_error_time": first_common_error,
            "first_difference_error_time": first_difference_error,
            "first_lift_time": first_lift,
            "first_wheel_limit_time": first_wheel_limit,
            "first_other_contact_time": first_other,
            "maximum_common_error_deg": np.degrees(maximum_common_error),
            "maximum_difference_error_deg": np.degrees(
                maximum_difference_error),
        })

    return summaries, comparisons


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--schedule", type=Path, default=DEFAULT_SCHEDULE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mujoco-trace", type=Path, nargs="+")
    parser.add_argument("--leg-length", type=float, default=0.18)
    parser.add_argument("--target-rate", type=float, default=2.0 * np.pi)
    parser.add_argument("--hold-seconds", type=float, default=3.0)
    parser.add_argument(
        "--accelerations", type=float, nargs="+",
        default=DEFAULT_ACCELERATIONS)
    arguments = parser.parse_args()

    if arguments.leg_length <= 0.0 or arguments.hold_seconds < 0.0:
        parser.error("leg length and hold duration must be valid")
    if arguments.target_rate == 0.0 or any(
        value <= 0.0 for value in arguments.accelerations
    ):
        parser.error("target rate and accelerations must be non-zero")

    matrix_a, matrix_b, gain, timestep = load_model(
        arguments.schedule, arguments.leg_length)
    arguments.output.mkdir(parents=True, exist_ok=True)

    summaries = []
    traces = []
    for acceleration in arguments.accelerations:
        summary, rows = run_case(
            matrix_a, matrix_b, gain, timestep,
            arguments.target_rate, acceleration,
            arguments.hold_seconds)
        summaries.append(summary)
        traces.extend(rows)

    summary_path = arguments.output / "summary.csv"
    with summary_path.open("w", newline="", encoding="ascii") as output:
        writer = csv.DictWriter(output, fieldnames=summaries[0].keys())
        writer.writeheader()
        writer.writerows(summaries)

    trace_path = arguments.output / "trace.csv"
    with trace_path.open("w", newline="", encoding="ascii") as output:
        writer = csv.writer(output)
        writer.writerow([
            "case", "time", "phase",
            *STATE_NAMES,
            *(f"ref_{name}" for name in STATE_NAMES),
            *INPUT_NAMES,
        ])
        writer.writerows(traces)

    if arguments.mujoco_trace is not None:
        comparison_summaries, comparisons = compare_mujoco_trace(
            arguments.mujoco_trace,
            matrix_a, matrix_b, gain, timestep)
        comparison_summary_path = arguments.output / "comparison_summary.csv"
        with comparison_summary_path.open(
            "w", newline="", encoding="ascii"
        ) as output:
            writer = csv.DictWriter(
                output, fieldnames=comparison_summaries[0].keys())
            writer.writeheader()
            writer.writerows(comparison_summaries)

        comparison_path = arguments.output / "comparison.csv"
        with comparison_path.open(
            "w", newline="", encoding="ascii"
        ) as output:
            writer = csv.writer(output)
            writer.writerow([
                "case", "time", "phase",
                "actual_common", "predicted_common",
                "actual_difference", "predicted_difference",
                "actual_wheel_l", "actual_wheel_r",
                "predicted_wheel_l", "predicted_wheel_r",
                "contact_wheel_l", "contact_wheel_r", "other_contact",
                "normal_force_l", "normal_force_r",
            ])
            writer.writerows(comparisons)

        for summary in comparison_summaries:
            print(
                f"{summary['case']:<22} "
                f"common_error={summary['first_common_error_time']} "
                f"difference_error={summary['first_difference_error_time']} "
                f"lift={summary['first_lift_time']} "
                f"limit={summary['first_wheel_limit_time']}")
        print(f"Wrote {comparison_summary_path}")
        print(f"Wrote {comparison_path}")

    for summary in summaries:
        print(
            f"{summary['case']:<18} "
            f"wheel={summary['peak_wheel_l']:.3f} "
            f"diff={summary['peak_leg_difference_deg']:.2f} deg "
            f"limit_at={summary['first_wheel_limit_time']}")
    print(f"Wrote {summary_path}")
    print(f"Wrote {trace_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
