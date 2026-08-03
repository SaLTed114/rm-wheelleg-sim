#!/usr/bin/env python3
"""Extract the current MuJoCo model and generate its LQR gain schedule."""

from __future__ import annotations

import argparse
from dataclasses import replace
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
    compute_gain_samples,
    evaluate_gain,
    fit_gain_schedule,
    validate_fitted_schedule,
)
from model_parameters import ModelParameterExtractor
from verify_lqr import verify_legacy


DEFAULT_MODEL = Path("models/MJCF/COD-2026RoboMaster-Balance.xml")
DEFAULT_OUTPUT = Path("tools/lqr/generated")
Q_DIAGONAL = (90, 60, 40, 15, 240, 4, 240, 4, 300, 60)
R_DIAGONAL = (3.2, 3.2, 0.7, 0.7)
CONTROLLER_LENGTH_MINIMUM = 0.16
YAW_INERTIA_SOURCES = ("base-link", "assembly")


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
    extracted: dict[str, object], yaw_scale: float,
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
        body_yaw_inertia_actual=rigid["body_yaw_inertia_actual"],
        body_yaw_inertia_scale=yaw_scale,
    )


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
        "body_yaw_inertia_actual": physical.body_yaw_inertia_actual,
        "body_yaw_inertia_scale": physical.body_yaw_inertia_scale,
        "body_yaw_inertia_model": physical.body_yaw_inertia_model,
    }


def yaw_inertia_scale(
    extracted: dict[str, object], source: str,
) -> float:
    rigid = extracted["rigid"]
    if source == "base-link": return 1.0
    if source == "assembly":
        return (
            rigid["assembly_yaw_inertia_diagnostic"] /
            rigid["body_yaw_inertia_actual"])
    raise ValueError(f"unknown yaw inertia source: {source}")


def build_result(
    model_path: Path,
    q_diagonal: tuple[float, ...] = Q_DIAGONAL,
    r_diagonal: tuple[float, ...] = R_DIAGONAL,
    extracted: dict[str, object] | None = None,
    yaw_inertia_source: str = "base-link",
) -> dict[str, object]:
    legacy_metrics = verify_legacy(
        Path("references/rm2026cb-balance-chassis/Tasks/balance_chassis/"
             "bc_lqr_schedule.c"))
    if extracted is None:
        extracted = ModelParameterExtractor(model_path).extract(sample_count=31)
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
    )

    selected_scale = yaw_inertia_scale(extracted, yaw_inertia_source)
    physical = make_physical_parameters(extracted, selected_scale)
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
    horner_error = maximum_float_horner_error(schedule)
    if horner_error > 1.0e-5:
        raise RuntimeError("float Horner evaluation exceeds tolerance")

    comparison_source = (
        "2x-base-link" if yaw_inertia_source == "base-link" else "base-link")
    comparison_scale = 2.0 if yaw_inertia_source == "base-link" else 1.0
    comparison = replace(
        physical, body_yaw_inertia_scale=comparison_scale)
    comparison_samples = compute_gain_samples(
        comparison, settings, leg_provider)
    comparison_difference = float(np.max(np.abs(
        samples.gains - comparison_samples.gains)))

    return {
        "schema_version": 1,
        "model": extracted,
        "controller": {
            "state_order": list(STATE_NAMES),
            "input_order": list(INPUT_NAMES),
            "timestep": settings.timestep,
            "q_diagonal": list(settings.q_diagonal),
            "r_diagonal": list(settings.r_diagonal),
            "sample_count": settings.sample_count,
            "length_range": [settings.length_min, settings.length_max],
            "model_parameter_range": [extracted_minimum, length_maximum],
            "parameter_extrapolation_range": [
                settings.length_min, extracted_minimum],
            "physical_parameters": physical_to_dict(physical),
        },
        "schedule": {
            "polynomial_order": schedule.polynomial_order,
            "length_midpoint": schedule.length_midpoint,
            "length_scale": schedule.length_scale,
            "coefficients": schedule.coefficient.tolist(),
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
            },
            "yaw_inertia_sensitivity": {
                "selected_source": yaw_inertia_source,
                "selected_scale": selected_scale,
                "selected_model_inertia": physical.body_yaw_inertia_model,
                "comparison_source": comparison_source,
                "comparison_scale": comparison_scale,
                "comparison_model_inertia": (
                    comparison.body_yaw_inertia_model),
                "comparison_maximum_raw_eigenvalue": (
                    comparison_samples.maximum_eigenvalue),
                "maximum_raw_gain_difference": comparison_difference,
            },
        },
    }


def emit_header(result: dict[str, object], path: Path) -> None:
    schedule = result["schedule"]
    coefficient = np.asarray(schedule["coefficients"])
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
    lines.extend(("};", "", "#endif", ""))
    path.write_text("\n".join(lines), encoding="ascii")


def emit_report(result: dict[str, object], path: Path) -> None:
    model = result["model"]
    rigid = model["rigid"]
    current = result["validation"]["current"]
    legacy = result["validation"]["legacy"]
    sensitivity = result["validation"]["yaw_inertia_sensitivity"]
    controller = result["controller"]
    schedule = result["schedule"]
    left_fit = model["fit_diagnostics"]["left"]
    right_fit = model["fit_diagnostics"]["right"]
    if sensitivity["selected_source"] == "base-link":
        yaw_conclusion = (
            "正式调度采用 `base_link` 实际惯量。两倍惯量只保留为敏感性对照，"
            "不能在其他位置再次乘二。")
    else:
        yaw_conclusion = (
            "本调度采用整机偏航惯量，仅用于诊断控制器与 plant 的惯量匹配，"
            "不能替代正式的 `base-link` 调度。")

    text = f"""# LQR 参数生成与验证

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
- 选定来源：`{sensitivity['selected_source']}`，倍率：
  `{sensitivity['selected_scale']:.9f}x`，建模值：
  `{sensitivity['selected_model_inertia']:.9f} kg*m^2`。
- 对照来源：`{sensitivity['comparison_source']}`，倍率：
  `{sensitivity['comparison_scale']:.9f}x`，对应
  `{sensitivity['comparison_model_inertia']:.9f} kg*m^2`。
- 两种情况的原始增益最大差：
  `{sensitivity['maximum_raw_gain_difference']:.9f}`。

{yaw_conclusion}

## 当前模型参数

- 控制器 LQR 调度腿长范围：`{controller['length_range'][0]:.3f}` 至
  `{controller['length_range'][1]:.3f} m`。
- 模型可直接提取参数的机械范围：`{controller['model_parameter_range'][0]:.3f}` 至
  `{controller['model_parameter_range'][1]:.3f} m`。
- 下段模型参数拟合延伸范围：`{controller['parameter_extrapolation_range'][0]:.3f}` 至
  `{controller['parameter_extrapolation_range'][1]:.3f} m`。
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

源 XML 中物理右腿的闭链连接点存在约 1.95 mm 的固定横向错位。因此可行性
判断采用每侧固有闭合误差加 0.5 mm，而不是统一使用 0.5 mm 绝对阈值；采样
得到的最大闭合误差为
`{1000.0 * model['scan']['maximum_closure_error']:.6f} mm`。

实际腿部质心明显偏离髋关节到轮轴的连线：

- 物理左腿垂直于虚拟腿轴的最大偏移：
  `{1000.0 * left_fit['maximum_com_perpendicular_offset']:.3f} mm`。
- 物理右腿垂直于虚拟腿轴的最大偏移：
  `{1000.0 * right_fit['maximum_com_perpendicular_offset']:.3f} mm`。

降阶惯量使用 `I_leg = I_about_hip - m_leg * L_b^2` 保留该偏置造成的
惯量。轮子没有参与腿部聚合，因为原动力学方程已经单独包含轮子质量和惯量。

## 当前增益调度验证

- 状态顺序：`{', '.join(controller['state_order'])}`。
- Q 对角线：`{controller['q_diagonal']}`。
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

当前 Q/R 由 MuJoCo 正反向加速与原地旋转扫描选定，重点覆盖 `0.16 m` 与
`0.18 m` 腿长；旧实车权重仅保留为生成器 golden test。调度按照 1 ms
仿真控制周期生成，不能直接用于现有 3 ms 实车控制循环。
"""
    path.write_text(text, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--report", type=Path,
        default=Path("docs/notes/lqr-validation.md"))
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
        "--yaw-inertia-source", choices=YAW_INERTIA_SOURCES,
        default="base-link",
        help="yaw inertia used by the reduced controller model")
    arguments = parser.parse_args()

    extracted = None
    if arguments.reuse_model_parameters is not None:
        cached = json.loads(
            arguments.reuse_model_parameters.read_text(encoding="ascii"))
        extracted = cached["model"] if "model" in cached else cached
    result = build_result(
        arguments.model,
        tuple(arguments.q_diagonal),
        tuple(arguments.r_diagonal),
        extracted,
        arguments.yaw_inertia_source)
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
    sensitivity = result["validation"]["yaw_inertia_sensitivity"]
    print(
        "Yaw inertia source: "
        f"{sensitivity['selected_source']} "
        f"({sensitivity['selected_model_inertia']:.9f} kg*m^2)")
    print(
        "Dense fitted closed-loop max |eigenvalue|: "
        f"{validation['maximum_fitted_eigenvalue']:.9f}")
    print(f"Wrote {json_path}")
    print(f"Wrote {header_path}")
    print(f"Wrote {arguments.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
