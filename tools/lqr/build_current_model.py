#!/usr/bin/env python3
"""Extract the current MuJoCo model and generate its LQR gain schedule."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import PchipInterpolator

from lqr_generator import (
    GainSchedule,
    INPUT_NAMES,
    LegParameters,
    LqrSettings,
    PhysicalParameters,
    STATE_NAMES,
    build_state_matrix_functions,
    build_state_cost_matrix,
    compute_gain_samples,
    evaluate_gain,
    evaluate_state_matrices,
    fit_gain_schedule,
    validate_fitted_schedule,
)
from model_parameters import ModelParameterExtractor
from verify_lqr import verify_legacy


DEFAULT_MODEL = Path("models/MJCF/Fudan-2026RoboMaster-Balance.xml")
DEFAULT_OUTPUT = Path("tools/lqr/generated")
Q_DIAGONAL = (90, 260, 40, 15, 60, 1, 60, 1, 900, 120)
R_DIAGONAL = (1.6, 1.6, 0.7, 0.7)
LEG_ANGLE_DIFFERENCE_WEIGHT = 960.0
LEG_ANGULAR_VELOCITY_DIFFERENCE_WEIGHT = 16.0
CONTROLLER_LENGTH_MINIMUM = 0.16
MODEL_NOMINAL_LENGTH = 0.18


def make_leg_provider(
    extracted: dict[str, object],
):
    interpolators = {}
    for side in ("left", "right"):
        samples = extracted["legs"][side]
        lengths = np.asarray([sample["length"] for sample in samples])
        interpolators[side] = {
            "com": PchipInterpolator(
                lengths, [sample["com_distance"] for sample in samples],
                extrapolate=True),
            "inertia": PchipInterpolator(
                lengths, [sample["inertia"] for sample in samples],
                extrapolate=True),
        }

    def provider(length: float) -> LegParameters:
        return LegParameters(
            left_com_distance=float(interpolators["left"]["com"](length)),
            right_com_distance=float(interpolators["right"]["com"](length)),
            left_inertia=float(interpolators["left"]["inertia"](length)),
            right_inertia=float(interpolators["right"]["inertia"](length)),
        )

    return provider


def make_physical_parameters(
    extracted: dict[str, object],
) -> PhysicalParameters:
    rigid = extracted["rigid"]
    return PhysicalParameters(
        wheel_radius=rigid["wheel_radius"],
        half_track=rigid["half_track"],
        body_com_height=rigid["body_com_height"],
        wheel_mass=rigid["wheel_mass"],
        leg_mass=extracted["leg_mass"],
        body_mass=rigid["body_mass"],
        gravity=9.81,
        wheel_inertia=rigid["wheel_inertia"],
        body_pitch_inertia=rigid["body_pitch_inertia"],
        body_yaw_inertia=rigid["assembly_yaw_inertia_reference"],
    )


def build_yaw_acceleration_feedforward_schedule(
    physical: PhysicalParameters,
    settings: LqrSettings,
    leg_provider,
    gain_schedule: GainSchedule,
) -> tuple[np.ndarray, float, float]:
    """Fit the actuator request for one rad/s^2 of pure yaw acceleration."""
    functions = build_state_matrix_functions(physical)
    lengths = np.linspace(
        settings.length_min, settings.length_max, settings.sample_count)
    samples = np.zeros((len(INPUT_NAMES), len(lengths)))
    target_derivative = np.zeros(len(STATE_NAMES))
    target_derivative[STATE_NAMES.index("dpsi")] = 1.0
    maximum_inverse_residual = 0.0

    for index, length in enumerate(lengths):
        _, matrix_b = evaluate_state_matrices(
            functions, float(length), leg_provider(float(length)))
        feedforward, _, _, _ = np.linalg.lstsq(
            matrix_b, target_derivative, rcond=None)
        samples[:, index] = feedforward
        maximum_inverse_residual = max(
            maximum_inverse_residual,
            float(np.max(np.abs(
                matrix_b @ feedforward - target_derivative))))

    normalized = (
        lengths - gain_schedule.length_midpoint
    ) / gain_schedule.length_scale
    coefficient = np.zeros((len(INPUT_NAMES), gain_schedule.polynomial_order + 1))
    for input_index in range(len(INPUT_NAMES)):
        coefficient[input_index] = np.polyfit(
            normalized, samples[input_index],
            gain_schedule.polynomial_order)
    coefficient = coefficient.astype(np.float32)

    fitted = np.asarray([
        np.polyval(coefficient[input_index], normalized)
        for input_index in range(len(INPUT_NAMES))
    ])
    maximum_fit_error = float(np.max(np.abs(fitted - samples)))
    return coefficient, maximum_inverse_residual, maximum_fit_error


def quantize_schedule(
    schedule: GainSchedule,
    samples,
) -> GainSchedule:
    schedule.coefficient = np.asarray(
        schedule.coefficient, dtype=np.float32).astype(float)
    maximum_error = 0.0
    for index, length in enumerate(samples.lengths):
        error = np.max(np.abs(
            evaluate_gain(schedule, float(length)) - samples.gains[:, :, index]))
        maximum_error = max(maximum_error, float(error))
    schedule.maximum_fit_error = maximum_error
    return schedule


def maximum_float_horner_error(schedule: GainSchedule) -> float:
    maximum_error = 0.0
    for length in np.linspace(
        schedule.length_midpoint - schedule.length_scale,
        schedule.length_midpoint + schedule.length_scale,
        201,
    ):
        normalized = np.float32(
            (length - schedule.length_midpoint) / schedule.length_scale)
        float_gain = np.zeros((len(INPUT_NAMES), len(STATE_NAMES)), dtype=np.float32)
        for input_index in range(len(INPUT_NAMES)):
            for state_index in range(len(STATE_NAMES)):
                values = schedule.coefficient[input_index, state_index]
                result = np.float32(values[0])
                for value in values[1:]:
                    result = np.float32(
                        result * normalized + np.float32(value))
                float_gain[input_index, state_index] = result
        error = np.max(np.abs(
            float_gain.astype(float) - evaluate_gain(schedule, float(length))))
        maximum_error = max(maximum_error, float(error))
    return maximum_error


def select_schedule(
    physical: PhysicalParameters,
    settings: LqrSettings,
    leg_provider,
    samples,
) -> GainSchedule:
    for order in range(3, 7):
        schedule = quantize_schedule(
            fit_gain_schedule(samples, order), samples)
        maximum_eigenvalue = validate_fitted_schedule(
            physical, settings, leg_provider, schedule)
        if maximum_eigenvalue < 1.0:
            return schedule
    raise RuntimeError(
        "no polynomial order from 3 through 6 stabilizes the dense length sweep")


def physical_to_dict(physical: PhysicalParameters) -> dict[str, float]:
    return {
        "wheel_radius": physical.wheel_radius,
        "half_track": physical.half_track,
        "body_com_height": physical.body_com_height,
        "wheel_mass": physical.wheel_mass,
        "leg_mass": physical.leg_mass,
        "body_mass": physical.body_mass,
        "gravity": physical.gravity,
        "wheel_inertia": physical.wheel_inertia,
        "body_pitch_inertia": physical.body_pitch_inertia,
        "body_yaw_inertia": physical.body_yaw_inertia,
    }


def build_result(
    model_path: Path,
    nominal_length: float = MODEL_NOMINAL_LENGTH,
    q_diagonal: tuple[float, ...] = Q_DIAGONAL,
    r_diagonal: tuple[float, ...] = R_DIAGONAL,
    extracted: dict[str, object] | None = None,
    leg_angle_difference_weight: float | None = (
        LEG_ANGLE_DIFFERENCE_WEIGHT),
    leg_angular_velocity_difference_weight: float | None = (
        LEG_ANGULAR_VELOCITY_DIFFERENCE_WEIGHT),
) -> dict[str, object]:
    legacy_metrics = verify_legacy(
        Path("references/rm2026cb-balance-chassis/Tasks/balance_chassis/"
             "bc_lqr_schedule.c"))
    if extracted is None:
        extracted = ModelParameterExtractor(model_path).extract(
            sample_count=31, nominal_length=nominal_length)
    leg_provider = make_leg_provider(extracted)
    extracted_minimum, length_maximum = extracted["safe_length_range"]
    if CONTROLLER_LENGTH_MINIMUM < extracted["scan"]["minimum"]:
        raise RuntimeError(
            "controller LQR minimum is below the model extraction scan")
    settings = LqrSettings(
        length_min=CONTROLLER_LENGTH_MINIMUM,
        length_max=length_maximum,
        sample_count=31,
        timestep=0.001,
        q_diagonal=q_diagonal,
        r_diagonal=r_diagonal,
        leg_angle_difference_weight=leg_angle_difference_weight,
        leg_angular_velocity_difference_weight=(
            leg_angular_velocity_difference_weight),
    )

    physical = make_physical_parameters(extracted)
    samples = compute_gain_samples(physical, settings, leg_provider)
    if samples.minimum_controllability_rank != len(STATE_NAMES):
        raise RuntimeError(
            "current reduced model is not controllable at every length sample")
    if samples.maximum_are_residual > 1.0e-8:
        raise RuntimeError("current model Riccati residual exceeds tolerance")
    if samples.maximum_eigenvalue >= 1.0:
        raise RuntimeError("current raw LQR gains are not stable")
    schedule = select_schedule(
        physical, settings, leg_provider, samples)
    (
        yaw_acceleration_feedforward,
        yaw_feedforward_inverse_residual,
        yaw_feedforward_fit_error,
    ) = build_yaw_acceleration_feedforward_schedule(
        physical, settings, leg_provider, schedule)
    horner_error = maximum_float_horner_error(schedule)
    if horner_error > 1.0e-5:
        raise RuntimeError("float Horner evaluation exceeds tolerance")

    matrix_q = build_state_cost_matrix(settings)
    left_angle = STATE_NAMES.index("theta_l")
    right_angle = STATE_NAMES.index("theta_r")
    left_angular_velocity = STATE_NAMES.index("dtheta_l")
    right_angular_velocity = STATE_NAMES.index("dtheta_r")

    return {
        "schema_version": 2,
        "model": extracted,
        "controller": {
            "state_order": list(STATE_NAMES),
            "input_order": list(INPUT_NAMES),
            "timestep": settings.timestep,
            "q_diagonal": list(settings.q_diagonal),
            "q_matrix_diagonal": np.diag(matrix_q).tolist(),
            "q_matrix": matrix_q.tolist(),
            "leg_angle_cost": {
                "common": settings.q_diagonal[left_angle],
                "difference": settings.leg_angle_difference_weight,
                "coupling": matrix_q[left_angle, right_angle],
            },
            "leg_angular_velocity_cost": {
                "common": settings.q_diagonal[left_angular_velocity],
                "difference": (
                    settings.leg_angular_velocity_difference_weight),
                "coupling": matrix_q[
                    left_angular_velocity, right_angular_velocity],
            },
            "r_diagonal": list(settings.r_diagonal),
            "sample_count": settings.sample_count,
            "length_range": [settings.length_min, settings.length_max],
            "model_parameter_range": [extracted_minimum, length_maximum],
            "parameter_extrapolation_range": (
                [settings.length_min, extracted_minimum]
                if settings.length_min < extracted_minimum else None),
            "physical_parameters": physical_to_dict(physical),
        },
        "schedule": {
            "polynomial_order": schedule.polynomial_order,
            "length_midpoint": schedule.length_midpoint,
            "length_scale": schedule.length_scale,
            "coefficients": schedule.coefficient.tolist(),
            "yaw_acceleration_feedforward_coefficients": (
                yaw_acceleration_feedforward.tolist()),
        },
        "validation": {
            "legacy": legacy_metrics,
            "current": {
                "minimum_controllability_rank": (
                    samples.minimum_controllability_rank),
                "maximum_raw_eigenvalue": samples.maximum_eigenvalue,
                "maximum_fitted_eigenvalue": (
                    schedule.maximum_dense_eigenvalue),
                "maximum_are_residual": samples.maximum_are_residual,
                "maximum_fit_error": schedule.maximum_fit_error,
                "maximum_float_horner_error": horner_error,
                "maximum_yaw_feedforward_inverse_residual": (
                    yaw_feedforward_inverse_residual),
                "maximum_yaw_feedforward_fit_error": (
                    yaw_feedforward_fit_error),
            },
        },
    }


def emit_header(result: dict[str, object], path: Path) -> None:
    schedule = result["schedule"]
    coefficient = np.asarray(schedule["coefficients"])
    yaw_feedforward = np.asarray(
        schedule["yaw_acceleration_feedforward_coefficients"])
    lines = [
        "#ifndef BALANCE_GENERATED_CURRENT_MODEL_LQR_H",
        "#define BALANCE_GENERATED_CURRENT_MODEL_LQR_H",
        "",
        "/* Generated by tools/lqr/build_current_model.py. */",
        f"#define BC_LQR_GENERATED_INPUT_COUNT {coefficient.shape[0]}",
        f"#define BC_LQR_GENERATED_STATE_COUNT {coefficient.shape[1]}",
        f"#define BC_LQR_GENERATED_COEFFICIENT_COUNT {coefficient.shape[2]}",
        "",
        "static const float bc_lqr_generated_length_midpoint = "
        f"{schedule['length_midpoint']:.9f}F;",
        "static const float bc_lqr_generated_length_scale = "
        f"{schedule['length_scale']:.9f}F;",
        "",
        "static const float bc_lqr_generated_coefficients",
        "    [BC_LQR_GENERATED_INPUT_COUNT]",
        "    [BC_LQR_GENERATED_STATE_COUNT]",
        "    [BC_LQR_GENERATED_COEFFICIENT_COUNT] = {",
    ]
    for input_index in range(coefficient.shape[0]):
        lines.append("    {")
        for state_index in range(coefficient.shape[1]):
            values = ", ".join(
                f"{value: .9f}F"
                for value in coefficient[input_index, state_index])
            comma = "," if state_index + 1 < coefficient.shape[1] else ""
            lines.append(f"        {{{values}}}{comma}")
        comma = "," if input_index + 1 < coefficient.shape[0] else ""
        lines.append(f"    }}{comma}")
    lines.extend(("};", ""))
    lines.extend((
        "static const float "
        "bc_lqr_generated_yaw_acceleration_feedforward_coefficients",
        "    [BC_LQR_GENERATED_INPUT_COUNT]",
        "    [BC_LQR_GENERATED_COEFFICIENT_COUNT] = {",
    ))
    for input_index in range(yaw_feedforward.shape[0]):
        values = ", ".join(
            f"{value: .9f}F" for value in yaw_feedforward[input_index])
        comma = "," if input_index + 1 < yaw_feedforward.shape[0] else ""
        lines.append(f"    {{{values}}}{comma}")
    lines.extend(("};", "", "#endif", ""))
    path.write_text("\n".join(lines), encoding="ascii")


def emit_report(result: dict[str, object], path: Path) -> None:
    model = result["model"]
    rigid = model["rigid"]
    current = result["validation"]["current"]
    legacy = result["validation"]["legacy"]
    controller = result["controller"]
    schedule = result["schedule"]
    left_fit = model["fit_diagnostics"]["left"]
    right_fit = model["fit_diagnostics"]["right"]
    extrapolation_range = controller["parameter_extrapolation_range"]
    model_name = Path(model["model_path"]).stem
    nominal_length = model["scan"]["nominal_length"]
    extrapolation_line = (
        "- 下段模型参数拟合延伸范围："
        f"`{extrapolation_range[0]:.3f}` 至 "
        f"`{extrapolation_range[1]:.3f} m`。"
        if extrapolation_range is not None else
        "- 控制范围全部落在模型直接参数范围内，无需下段拟合外推。")
    text = f"""# LQR 参数生成与验证归档

本文由 `tools/lqr/build_current_model.py` 自动生成。

## 旧实车参数 Golden Test

- 固件系数最大误差：`{legacy['maximum_coefficient_difference']:.6e}`。
- 原始增益闭环特征值模最大值：`{legacy['maximum_raw_eigenvalue']:.9f}`。
- 离散 Riccati 方程最大残差：`{legacy['maximum_are_residual']:.6e}`。
- 结论：Python 生成器能够数值复现当前实车固件中的参数表。

## 偏航惯量结论

将 `equ7` 代入 `equ5` 后，所有偏航惯量项严格化简为
`-I_z * D2psi`。原始推导也通过
`I * D2psi = R_l * (-N_f,l + N_f,r)` 将 `I` 定义为完整的机体偏航惯量，
方程中没有遗留的二分之一系数。

- `base_link` 实际 Izz：`{rigid['body_yaw_inertia_actual']:.9f} kg*m^2`。
- 提取器 nominal 姿态下的整机诊断 Izz：
  `{rigid['assembly_yaw_inertia_diagnostic']:.9f} kg*m^2`。
- 整机 Izz 参考腿长：
  `{rigid['assembly_yaw_inertia_reference_length']:.3f} m`，LQR 建模值：
  `{controller['physical_parameters']['body_yaw_inertia']:.9f} kg*m^2`。
- 在 `{rigid['assembly_yaw_inertia_range_lengths'][0]:.3f}` 至
  `{rigid['assembly_yaw_inertia_range_lengths'][1]:.3f} m` 腿长采样中的整机 Izz
  范围：`{rigid['assembly_yaw_inertia_range'][0]:.9f}` 至
  `{rigid['assembly_yaw_inertia_range'][1]:.9f} kg*m^2`。

生成器固定使用参考腿长处聚合得到的整机惯量，不提供 base-link/assembly
来源开关，也不进行额外倍率换算。

## 当前模型参数

- 控制器 LQR 调度腿长范围：`{controller['length_range'][0]:.3f}` 至
  `{controller['length_range'][1]:.3f} m`。
- 模型可直接提取参数的机械范围：`{controller['model_parameter_range'][0]:.3f}` 至
  `{controller['model_parameter_range'][1]:.3f} m`。
{extrapolation_line}
- 控制周期：`{1000.0 * controller['timestep']:.1f} ms`。
- 机体质量：`{rigid['body_mass']:.6f} kg`。
- 机体俯仰惯量：`{rigid['body_pitch_inertia']:.9f} kg*m^2`。
- 机体偏航惯量：`{rigid['body_yaw_inertia_actual']:.9f} kg*m^2`。
- 单轮质量：`{rigid['wheel_mass']:.6f} kg`；半径：
  `{rigid['wheel_radius']:.9f} m`；轴向惯量：
  `{rigid['wheel_inertia']:.9f} kg*m^2`。
- 半轮距：`{rigid['half_track']:.9f} m`；单腿质量：
  `{model['leg_mass']:.6f} kg`。
- 降阶前后质量分配误差：`{model['mass_partition_error']:.3e} kg`。
- 机体质心相对虚拟髋关节的高度为
  `{rigid['body_com_height']:.9f} m`，前向偏移为
  `{rigid['body_com_forward_offset']:.9f} m`。

当前生成 plant 在参数扫描中的最大闭合误差为
`{1000.0 * model['scan']['maximum_closure_error']:.6f} mm`。可行性判断仍按
每侧名义姿态固有闭合误差加 `0.5 mm` 设置上限，以兼容后续保留固定装配偏差的
闭链模型。

实际腿部质心明显偏离髋关节到轮轴的连线：

- 物理左腿垂直于虚拟腿轴的最大偏移：
  `{1000.0 * left_fit['maximum_com_perpendicular_offset']:.3f} mm`。
- 物理右腿垂直于虚拟腿轴的最大偏移：
  `{1000.0 * right_fit['maximum_com_perpendicular_offset']:.3f} mm`。

降阶惯量使用 `I_leg = I_about_hip - m_leg * L_b^2` 保留该偏置造成的
惯量。轮子没有参与腿部聚合，因为原动力学方程已经单独包含轮子质量和惯量。

## 当前增益调度验证

- 状态顺序：`{', '.join(controller['state_order'])}`。
- Q 基础对角权重：`{controller['q_diagonal']}`。
- Q 实际矩阵对角线：`{controller['q_matrix_diagonal']}`。
- 共同腿角权重：`{controller['leg_angle_cost']['common']}`；差分腿角权重：
  `{controller['leg_angle_cost']['difference']}`；左右腿角交叉项：
  `{controller['leg_angle_cost']['coupling']}`。
- 共同腿角速度权重：`{controller['leg_angular_velocity_cost']['common']}`；
  差分腿角速度权重：
  `{controller['leg_angular_velocity_cost']['difference']}`；交叉项：
  `{controller['leg_angular_velocity_cost']['coupling']}`。
- 输入顺序：`{', '.join(controller['input_order'])}`。
- R 对角线：`{controller['r_diagonal']}`。
- 多项式阶数：`{schedule['polynomial_order']}`。
- 所有采样点的可控矩阵秩：`{current['minimum_controllability_rank']}`。
- 原始增益闭环特征值模最大值：
  `{current['maximum_raw_eigenvalue']:.9f}`。
- 拟合增益在 201 个腿长点上的闭环特征值模最大值：
  `{current['maximum_fitted_eigenvalue']:.9f}`。
- 离散 Riccati 方程最大残差：`{current['maximum_are_residual']:.6e}`。
- 增益拟合最大误差：`{current['maximum_fit_error']:.6e}`。
- float Horner 求值最大误差：
  `{current['maximum_float_horner_error']:.6e}`。

当前模型 `{model_name}` 以 `{nominal_length:.3f} m` 为名义工作点，增益覆盖
生成器报告的完整控制腿长范围；
旧实车权重仅保留为生成器 golden test。调度按照 1 ms 仿真控制周期生成，
不能直接用于现有 3 ms 实车控制循环。
"""
    path.write_text(text, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument(
        "--nominal-length", type=float, default=MODEL_NOMINAL_LENGTH)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--report", type=Path,
        default=Path("docs/archive/validation/lqr-validation.md"))
    parser.add_argument(
        "--reuse-model-parameters", type=Path,
        help="reuse the model section from a prior generated JSON")
    parser.add_argument(
        "--q-diagonal", type=float, nargs=len(Q_DIAGONAL),
        default=Q_DIAGONAL)
    parser.add_argument(
        "--r-diagonal", type=float, nargs=len(R_DIAGONAL),
        default=R_DIAGONAL)
    parser.add_argument(
        "--leg-angle-difference-weight", type=float,
        default=LEG_ANGLE_DIFFERENCE_WEIGHT,
        help="eigenweight of the differential leg-angle mode")
    parser.add_argument(
        "--leg-angular-velocity-difference-weight", type=float,
        default=LEG_ANGULAR_VELOCITY_DIFFERENCE_WEIGHT,
        help="eigenweight of the differential leg-angular-velocity mode")
    arguments = parser.parse_args()

    extracted = None
    if arguments.reuse_model_parameters is not None:
        cached = json.loads(
            arguments.reuse_model_parameters.read_text(encoding="ascii"))
        extracted = cached["model"] if "model" in cached else cached
    result = build_result(
        model_path=arguments.model,
        nominal_length=arguments.nominal_length,
        q_diagonal=tuple(arguments.q_diagonal),
        r_diagonal=tuple(arguments.r_diagonal),
        extracted=extracted,
        leg_angle_difference_weight=(
            arguments.leg_angle_difference_weight),
        leg_angular_velocity_difference_weight=(
            arguments.leg_angular_velocity_difference_weight))
    arguments.output.mkdir(parents=True, exist_ok=True)
    json_path = arguments.output / "current_model_schedule.json"
    header_path = arguments.output / "current_model_schedule.h"
    json_path.write_text(
        json.dumps(result, indent=2, ensure_ascii=True) + "\n",
        encoding="ascii")
    emit_header(result, header_path)
    emit_report(result, arguments.report)

    validation = result["validation"]["current"]
    print(
        "Controller LQR leg-length range: "
        f"{result['controller']['length_range'][0]:.3f} .. "
        f"{result['controller']['length_range'][1]:.3f} m")
    print(
        "Direct model-parameter range: "
        f"{result['controller']['model_parameter_range'][0]:.3f} .. "
        f"{result['controller']['model_parameter_range'][1]:.3f} m")
    print(
        "Generated polynomial order: "
        f"{result['schedule']['polynomial_order']}")
    print(
        "Assembly yaw inertia: "
        f"{result['controller']['physical_parameters']['body_yaw_inertia']:.9f} "
        "kg*m^2")
    print(
        "Dense fitted closed-loop max |eigenvalue|: "
        f"{validation['maximum_fitted_eigenvalue']:.9f}")
    print(f"Wrote {json_path}")
    print(f"Wrote {header_path}")
    print(f"Wrote {arguments.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
