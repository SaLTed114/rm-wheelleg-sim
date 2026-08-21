#!/usr/bin/env python3
"""Calibrate MuJoCo leg-adapter joint offsets near the standing pose."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
LQR_TOOLS = REPOSITORY_ROOT / "tools" / "lqr"
sys.path.insert(0, str(LQR_TOOLS))

from model_parameters import ModelParameterExtractor, SIDE_SPECS  # noqa: E402


DEFAULT_MODEL = (
    REPOSITORY_ROOT / "models" / "MJCF" /
    "Fudan-2026RoboMaster-Balance.xml"
)
DEFAULT_JSON = (
    REPOSITORY_ROOT / "tools" / "calibration" / "generated" /
    "leg_adapter_calibration.json"
)
DEFAULT_HEADER = (
    REPOSITORY_ROOT / "src" / "sim" / "generated" /
    "mujoco_leg_calibration.hpp"
)

SIDE_NAMES = ("left", "right")
JOINT_NAMES = ("front", "rear")
# Fudan's joint axes use the same sign on the front/rear pair.  These are the
# fitted standing-pose offsets used as the new model baseline.
JOINT_SCALES = ((+1.0, +1.0), (-1.0, -1.0))
LEGACY_OFFSETS = (
    (-1.700323600945978, -0.000896018156600),
    (-1.700323600945066, -0.000896018155538),
)

LOCAL_LENGTHS = np.arange(0.18, 0.2100001, 0.005)
SAFE_LENGTHS = np.linspace(0.186, 0.39, 42)
LENGTH_ERROR_SCALE = 0.001
ANGLE_ERROR_SCALE = np.deg2rad(0.25)

MAXIMUM_LOCAL_LENGTH_RMS = 0.00125
MAXIMUM_LOCAL_LENGTH_ERROR = 0.002
MAXIMUM_LOCAL_ANGLE_ERROR = np.deg2rad(0.5)
MAXIMUM_LOCAL_SIDE_DIFFERENCE = 0.001
MAXIMUM_OFFSET_CHANGE = 0.15
REGRESSION_EPSILON = 1.0e-10


@dataclass(frozen=True)
class PoseSamples:
    lengths: np.ndarray
    raw_joint_positions: np.ndarray
    model_joint_positions: np.ndarray
    measured_lengths: np.ndarray
    measured_angles: np.ndarray
    closure_errors: np.ndarray
    target_errors: np.ndarray
    joint_margins: np.ndarray


def wrap_angle(value: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(value), np.cos(value))


def calculate_virtual_leg(
    raw_joint_positions: np.ndarray,
    scales: tuple[float, float],
    offsets: np.ndarray,
    hip_link_length: float,
    wheel_link_length: float,
) -> tuple[np.ndarray, np.ndarray]:
    phi_front = scales[0] * raw_joint_positions[:, 0] + offsets[0]
    phi_rear = scales[1] * raw_joint_positions[:, 1] + offsets[1]
    delta = 0.5 * (phi_front - phi_rear)
    radicand = (
        wheel_link_length**2
        - hip_link_length**2 * np.sin(delta) ** 2
    )
    if np.any(radicand < -1.0e-12):
        raise RuntimeError("calibrated joint pose is outside virtual-leg geometry")
    length = (
        hip_link_length * np.cos(delta)
        + np.sqrt(np.maximum(radicand, 0.0))
    )
    angle = 0.5 * (phi_front + phi_rear)
    return length, angle


def sample_side(
    extractor: ModelParameterExtractor,
    side_index: int,
    lengths: np.ndarray,
) -> PoseSamples:
    spec = SIDE_SPECS[side_index]
    solutions = extractor.scan_side(spec, lengths, 0.34)
    addresses = extractor._side_addresses(spec)
    front_index = 0
    rear_index = next(
        index for index, name in enumerate(spec.joint_names)
        if name.endswith("_rear_joint")
    )
    raw_positions = []
    model_positions = []
    measured_lengths = []
    measured_angles = []

    for solution in solutions:
        extractor.data.qpos[addresses["qpos"]] = solution.joint_position
        mujoco.mj_forward(extractor.model, extractor.data)
        hip = extractor.data.site_xpos[addresses["hip"]]
        wheel = extractor.data.site_xpos[addresses["wheel"]]
        displacement = wheel - hip
        raw_positions.append((
            solution.joint_position[front_index],
            solution.joint_position[rear_index],
        ))
        model_positions.append(solution.joint_position.copy())
        measured_lengths.append(np.hypot(displacement[0], displacement[2]))
        measured_angles.append(np.arctan2(displacement[2], -displacement[0]))

    return PoseSamples(
        lengths=np.asarray(lengths, dtype=float),
        raw_joint_positions=np.asarray(raw_positions, dtype=float),
        model_joint_positions=np.asarray(model_positions, dtype=float),
        measured_lengths=np.asarray(measured_lengths, dtype=float),
        measured_angles=np.asarray(measured_angles, dtype=float),
        closure_errors=np.asarray(
            [solution.closure_error for solution in solutions], dtype=float),
        target_errors=np.asarray(
            [solution.target_error for solution in solutions], dtype=float),
        joint_margins=np.asarray(
            [solution.joint_margin for solution in solutions], dtype=float),
    )


def error_arrays(
    samples: PoseSamples,
    side_index: int,
    offsets: np.ndarray,
    hip_link_length: float,
    wheel_link_length: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    calculated_length, calculated_angle = calculate_virtual_leg(
        samples.raw_joint_positions,
        JOINT_SCALES[side_index],
        offsets,
        hip_link_length,
        wheel_link_length,
    )
    return (
        calculated_length - samples.measured_lengths,
        wrap_angle(calculated_angle - samples.measured_angles),
        calculated_length,
    )


def metrics(length_error: np.ndarray, angle_error: np.ndarray) -> dict[str, float]:
    return {
        "length_mean_m": float(np.mean(length_error)),
        "length_rms_m": float(np.sqrt(np.mean(length_error**2))),
        "length_maximum_absolute_m": float(np.max(np.abs(length_error))),
        "angle_mean_rad": float(np.mean(angle_error)),
        "angle_rms_rad": float(np.sqrt(np.mean(angle_error**2))),
        "angle_maximum_absolute_rad": float(np.max(np.abs(angle_error))),
    }


def fit_offsets(
    samples: PoseSamples,
    side_index: int,
    hip_link_length: float,
    wheel_link_length: float,
) -> np.ndarray:
    def residual(offsets: np.ndarray) -> np.ndarray:
        length_error, angle_error, _ = error_arrays(
            samples,
            side_index,
            offsets,
            hip_link_length,
            wheel_link_length,
        )
        return np.concatenate((
            length_error / LENGTH_ERROR_SCALE,
            angle_error / ANGLE_ERROR_SCALE,
        ))

    result = least_squares(
        residual,
        np.asarray(LEGACY_OFFSETS[side_index], dtype=float),
        xtol=1.0e-14,
        ftol=1.0e-14,
        gtol=1.0e-14,
        max_nfev=2000,
    )
    if not result.success:
        raise RuntimeError(
            f"{SIDE_NAMES[side_index]} offset fit failed: {result.message}")
    return result.x


def sample_records(samples: PoseSamples) -> list[dict[str, object]]:
    return [
        {
            "requested_length_m": float(samples.lengths[index]),
            "raw_joint_position_rad": [
                float(value) for value in samples.raw_joint_positions[index]
            ],
            "model_joint_position_rad": [
                float(value) for value in samples.model_joint_positions[index]
            ],
            "measured_length_m": float(samples.measured_lengths[index]),
            "measured_angle_rad": float(samples.measured_angles[index]),
            "closure_error_m": float(samples.closure_errors[index]),
            "target_error_m": float(samples.target_errors[index]),
            "joint_margin_rad": float(samples.joint_margins[index]),
        }
        for index in range(len(samples.lengths))
    ]


def build_result(
    model_path: Path,
    hip_link_length: float,
    wheel_link_length: float,
) -> tuple[dict[str, object], list[str]]:
    extractor = ModelParameterExtractor(model_path)
    local_samples = tuple(
        sample_side(extractor, side, LOCAL_LENGTHS)
        for side in range(len(SIDE_NAMES))
    )
    safe_samples = tuple(
        sample_side(extractor, side, SAFE_LENGTHS)
        for side in range(len(SIDE_NAMES))
    )
    fitted_offsets = tuple(
        fit_offsets(
            local_samples[side], side,
            hip_link_length, wheel_link_length)
        for side in range(len(SIDE_NAMES))
    )

    failures = []
    side_results = {}
    local_calculated_lengths = []
    for side, side_name in enumerate(SIDE_NAMES):
        legacy = np.asarray(LEGACY_OFFSETS[side], dtype=float)
        candidate = fitted_offsets[side]
        local_before = error_arrays(
            local_samples[side], side, legacy,
            hip_link_length, wheel_link_length)
        local_after = error_arrays(
            local_samples[side], side, candidate,
            hip_link_length, wheel_link_length)
        safe_before = error_arrays(
            safe_samples[side], side, legacy,
            hip_link_length, wheel_link_length)
        safe_after = error_arrays(
            safe_samples[side], side, candidate,
            hip_link_length, wheel_link_length)
        local_before_metrics = metrics(local_before[0], local_before[1])
        local_after_metrics = metrics(local_after[0], local_after[1])
        safe_before_metrics = metrics(safe_before[0], safe_before[1])
        safe_after_metrics = metrics(safe_after[0], safe_after[1])
        local_calculated_lengths.append(local_after[2])

        if local_after_metrics["length_rms_m"] > MAXIMUM_LOCAL_LENGTH_RMS:
            failures.append(f"{side_name} local length RMS exceeds limit")
        if (local_after_metrics["length_maximum_absolute_m"] >
                MAXIMUM_LOCAL_LENGTH_ERROR):
            failures.append(f"{side_name} local maximum length error exceeds limit")
        if (local_after_metrics["angle_maximum_absolute_rad"] >
                MAXIMUM_LOCAL_ANGLE_ERROR):
            failures.append(f"{side_name} local maximum angle error exceeds limit")
        if np.max(np.abs(candidate - legacy)) > MAXIMUM_OFFSET_CHANGE:
            failures.append(f"{side_name} offset change exceeds limit")

        for metric_name in (
            "length_rms_m",
            "length_maximum_absolute_m",
            "angle_rms_rad",
            "angle_maximum_absolute_rad",
        ):
            if (safe_after_metrics[metric_name] >
                    safe_before_metrics[metric_name] + REGRESSION_EPSILON):
                failures.append(
                    f"{side_name} safe-range {metric_name} regressed")

        side_results[side_name] = {
            "joint_scales": list(JOINT_SCALES[side]),
            "legacy_offsets_rad": [float(value) for value in legacy],
            "fitted_offsets_rad": [float(value) for value in candidate],
            "offset_changes_rad": [
                float(value) for value in candidate - legacy
            ],
            "local_before": local_before_metrics,
            "local_after": local_after_metrics,
            "safe_before": safe_before_metrics,
            "safe_after": safe_after_metrics,
            "local_samples": sample_records(local_samples[side]),
        }

    maximum_side_difference = float(np.max(np.abs(
        local_calculated_lengths[0] - local_calculated_lengths[1]
    )))
    if maximum_side_difference > MAXIMUM_LOCAL_SIDE_DIFFERENCE:
        failures.append("local left/right virtual length difference exceeds limit")

    result = {
        "schema_version": 1,
        "model": {
            "path": str(model_path.relative_to(REPOSITORY_ROOT)).replace("\\", "/"),
            "sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
            "mujoco_version": mujoco.__version__,
        },
        "geometry": {
            "hip_link_length_m": hip_link_length,
            "wheel_link_length_m": wheel_link_length,
        },
        "fit": {
            "local_length_range_m": [
                float(LOCAL_LENGTHS[0]), float(LOCAL_LENGTHS[-1])],
            "local_length_step_m": 0.005,
            "safe_diagnostic_range_m": [
                float(SAFE_LENGTHS[0]), float(SAFE_LENGTHS[-1])],
            "length_error_scale_m": LENGTH_ERROR_SCALE,
            "angle_error_scale_rad": ANGLE_ERROR_SCALE,
            "maximum_local_side_difference_m": maximum_side_difference,
        },
        "limits": {
            "local_length_rms_m": MAXIMUM_LOCAL_LENGTH_RMS,
            "local_length_maximum_absolute_m": MAXIMUM_LOCAL_LENGTH_ERROR,
            "local_angle_maximum_absolute_rad": MAXIMUM_LOCAL_ANGLE_ERROR,
            "local_side_difference_m": MAXIMUM_LOCAL_SIDE_DIFFERENCE,
            "offset_change_rad": MAXIMUM_OFFSET_CHANGE,
            "safe_range_must_not_regress": True,
        },
        "sides": side_results,
        "accepted": not failures,
        "failures": failures,
    }
    return result, failures


def emit_header(result: dict[str, object]) -> str:
    sides = result["sides"]
    local_samples = [sides[name]["local_samples"] for name in SIDE_NAMES]
    lines = [
        "#ifndef BALANCE_SIM_GENERATED_MUJOCO_LEG_CALIBRATION_HPP",
        "#define BALANCE_SIM_GENERATED_MUJOCO_LEG_CALIBRATION_HPP",
        "",
        "#include <array>",
        "#include <cstddef>",
        "",
        "namespace balance::sim::calibration {",
        "",
        "// Generated by tools/calibration/calibrate_leg_adapter.py.",
        f'inline constexpr char kModelSha256[] = "{result["model"]["sha256"]}";',
        "inline constexpr std::array<std::array<double, 2>, 2> kJointScales{{",
    ]
    for values in JOINT_SCALES:
        lines.append(f"    {{{{{values[0]:+.1f}, {values[1]:+.1f}}}}},")
    lines.extend((
        "}};",
        "inline constexpr std::array<std::array<double, 2>, 2> kJointOffsets{{",
    ))
    for name in SIDE_NAMES:
        values = sides[name]["fitted_offsets_rad"]
        lines.append(f"    {{{{{values[0]:.15f}, {values[1]:.15f}}}}},")
    lines.extend((
        "}};",
        f"inline constexpr std::size_t kStandingSampleCount = {len(LOCAL_LENGTHS)};",
        f"inline constexpr std::size_t kModelLegJointCount = {len(SIDE_SPECS[0].joint_names)};",
        "inline constexpr std::array<",
        "    std::array<std::array<double, 2>, kStandingSampleCount>, 2>",
        "    kStandingRawJointPositions{{",
    ))
    for side_samples in local_samples:
        lines.append("        {{")
        for sample in side_samples:
            values = sample["raw_joint_position_rad"]
            lines.append(
                f"            {{{{{values[0]:.15f}, {values[1]:.15f}}}}},")
        lines.append("        }},")
    lines.extend((
        "    }};",
        "inline constexpr std::array<",
        "    std::array<std::array<double, kModelLegJointCount>,",
        "        kStandingSampleCount>, 2>",
        "    kStandingModelJointPositions{{",
    ))
    for side_samples in local_samples:
        lines.append("        {{")
        for sample in side_samples:
            values = sample["model_joint_position_rad"]
            formatted = ", ".join(f"{value:.15f}" for value in values)
            lines.append(f"            {{{{{formatted}}}}},")
        lines.append("        }},")
    lines.extend((
        "    }};",
        "",
        "} // namespace balance::sim::calibration",
        "",
        "#endif",
        "",
    ))
    return "\n".join(lines)


def emit_json(result: dict[str, object]) -> str:
    return json.dumps(result, indent=2, ensure_ascii=True) + "\n"


def print_summary(result: dict[str, object]) -> None:
    for name in SIDE_NAMES:
        side = result["sides"][name]
        local = side["local_after"]
        safe = side["safe_after"]
        print(
            f"{name}: offsets={side['fitted_offsets_rad']}, "
            f"local length rms/max="
            f"{1000.0 * local['length_rms_m']:.3f}/"
            f"{1000.0 * local['length_maximum_absolute_m']:.3f} mm, "
            f"local angle max="
            f"{np.rad2deg(local['angle_maximum_absolute_rad']):.3f} deg, "
            f"safe length rms/max="
            f"{1000.0 * safe['length_rms_m']:.3f}/"
            f"{1000.0 * safe['length_maximum_absolute_m']:.3f} mm"
        )
    print(
        "local maximum side difference: "
        f"{1000.0 * result['fit']['maximum_local_side_difference_m']:.3f} mm")
    print("calibration accepted" if result["accepted"] else "calibration rejected")
    for failure in result["failures"]:
        print(f"  - {failure}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--json", type=Path, default=DEFAULT_JSON)
    parser.add_argument("--header", type=Path, default=DEFAULT_HEADER)
    parser.add_argument("--hip-link-length", type=float, default=0.175)
    parser.add_argument("--wheel-link-length", type=float, default=0.208)
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--write", action="store_true")
    action.add_argument("--check", action="store_true")
    arguments = parser.parse_args()

    model_path = arguments.model.resolve()
    result, failures = build_result(
        model_path,
        arguments.hip_link_length,
        arguments.wheel_link_length,
    )
    json_text = emit_json(result)
    header_text = emit_header(result)
    print_summary(result)
    if failures:
        return 1

    if arguments.write:
        arguments.json.parent.mkdir(parents=True, exist_ok=True)
        arguments.header.parent.mkdir(parents=True, exist_ok=True)
        arguments.json.write_text(json_text, encoding="ascii")
        arguments.header.write_text(header_text, encoding="ascii")
        print(f"wrote {arguments.json}")
        print(f"wrote {arguments.header}")
    elif arguments.check:
        stale = []
        if not arguments.json.is_file() or arguments.json.read_text(
                encoding="ascii") != json_text:
            stale.append(str(arguments.json))
        if not arguments.header.is_file() or arguments.header.read_text(
                encoding="ascii") != header_text:
            stale.append(str(arguments.header))
        if stale:
            print("stale calibration artifacts:")
            for path in stale:
                print(f"  - {path}")
            return 1
        print("calibration artifacts are current")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
