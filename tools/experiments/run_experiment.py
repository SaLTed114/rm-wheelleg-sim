#!/usr/bin/env python3
"""Build and run reproducible balance-controller experiments from TOML."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
from typing import Any

try:
    import tomllib
except ModuleNotFoundError as error:
    raise SystemExit("run_experiment.py requires Python 3.11 or newer") from error


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
STATE_COUNT = 10
INPUT_COUNT = 4
FORWARD_OBSERVATIONS = {
    "wheel-odometry", "base-truth", "contact-gated",
}
SAFE_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*\Z")


@dataclass(frozen=True)
class PathsConfig:
    model: Path
    output_root: Path


@dataclass(frozen=True)
class BuildConfig:
    root: Path
    configuration: str
    generator: str | None
    cmake_args: tuple[str, ...]


@dataclass(frozen=True)
class ControllerConfig:
    leg_length: float
    position_feedback: bool
    velocity_feedback: bool
    forward_observation: str
    roll_restrained: bool
    trace_stride: int


@dataclass(frozen=True)
class LqrGenerateConfig:
    model_parameters: Path | None
    q_diagonal: tuple[float, ...]
    r_diagonal: tuple[float, ...]
    leg_angle_difference_weight: float
    leg_angular_velocity_difference_weight: float
    yaw_inertia_source: str


@dataclass(frozen=True)
class LqrConfig:
    mode: str
    schedule_dir: Path | None
    generate: LqrGenerateConfig | None


@dataclass(frozen=True)
class AnalysisConfig:
    forward_linear: bool


@dataclass(frozen=True)
class CaseConfig:
    name: str
    axis: str
    target: float
    command_rate: float
    target_hold_seconds: float
    stop_settle_seconds: float
    standing_seconds: float


@dataclass(frozen=True)
class ExperimentConfig:
    version: int
    name: str
    source_path: Path
    paths: PathsConfig
    build: BuildConfig
    controller: ControllerConfig
    lqr: LqrConfig
    analysis: AnalysisConfig
    cases: tuple[CaseConfig, ...]


class ExperimentError(RuntimeError):
    pass


def reject_unknown(table: dict[str, Any], allowed: set[str], label: str) -> None:
    unknown = set(table) - allowed
    if unknown:
        raise ExperimentError(
            f"unknown {label} keys: {', '.join(sorted(unknown))}")


def require_table(document: dict[str, Any], name: str) -> dict[str, Any]:
    value = document.get(name)
    if not isinstance(value, dict):
        raise ExperimentError(f"{name} must be a TOML table")
    return value


def resolve_path(config_directory: Path, value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ExperimentError(f"{label} must be a non-empty path string")
    path = Path(value)
    return (config_directory / path).resolve() if not path.is_absolute() else path


def finite_float(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ExperimentError(f"{label} must be a number")
    result = float(value)
    if not (float("-inf") < result < float("inf")):
        raise ExperimentError(f"{label} must be finite")
    return result


def positive_float(value: Any, label: str) -> float:
    result = finite_float(value, label)
    if result <= 0.0:
        raise ExperimentError(f"{label} must be positive")
    return result


def positive_vector(value: Any, length: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise ExperimentError(f"{label} must contain exactly {length} values")
    result = tuple(positive_float(item, f"{label}[{index}]")
                   for index, item in enumerate(value))
    return result


def boolean_value(table: dict[str, Any], name: str, default: bool) -> bool:
    value = table.get(name, default)
    if not isinstance(value, bool):
        raise ExperimentError(f"{name} must be true or false")
    return value


def load_config(path: Path) -> ExperimentConfig:
    source_path = path.resolve()
    try:
        with source_path.open("rb") as source:
            document = tomllib.load(source)
    except (OSError, tomllib.TOMLDecodeError) as error:
        raise ExperimentError(f"failed to read {source_path}: {error}") from error

    reject_unknown(document, {
        "version", "name", "paths", "build", "controller", "lqr",
        "analysis", "case",
    }, "top-level")
    if document.get("version") != 1:
        raise ExperimentError("version must be 1")
    name = document.get("name")
    if not isinstance(name, str) or SAFE_NAME.fullmatch(name) is None:
        raise ExperimentError(
            "name must use only letters, digits, dot, underscore, and dash")
    config_directory = source_path.parent

    paths_table = require_table(document, "paths")
    reject_unknown(paths_table, {"model", "output_root"}, "paths")
    paths = PathsConfig(
        model=resolve_path(config_directory, paths_table.get("model"),
                           "paths.model"),
        output_root=resolve_path(
            config_directory, paths_table.get("output_root"),
            "paths.output_root"),
    )

    build_table = require_table(document, "build")
    reject_unknown(build_table, {
        "root", "configuration", "generator", "cmake_args",
    }, "build")
    configuration = build_table.get("configuration", "Release")
    if not isinstance(configuration, str) or not configuration:
        raise ExperimentError("build.configuration must be a string")
    generator = build_table.get("generator")
    if generator is not None and (
        not isinstance(generator, str) or not generator
    ):
        raise ExperimentError("build.generator must be a non-empty string")
    cmake_args_value = build_table.get("cmake_args", [])
    if not isinstance(cmake_args_value, list) or not all(
        isinstance(item, str) and item for item in cmake_args_value
    ):
        raise ExperimentError("build.cmake_args must be an array of strings")
    reserved_cmake_options = (
        "-G", "-DBALANCE_LQR_SCHEDULE_DIR=", "-DCMAKE_BUILD_TYPE=",
        "-DBUILD_TESTING=", "-DBALANCE_BUILD_GUI=",
    )
    if any(item.startswith(reserved_cmake_options) for item in cmake_args_value):
        raise ExperimentError(
            "build.cmake_args contains an option managed by the runner")
    build = BuildConfig(
        root=resolve_path(config_directory, build_table.get("root"),
                          "build.root"),
        configuration=configuration,
        generator=generator,
        cmake_args=tuple(cmake_args_value),
    )

    controller_table = require_table(document, "controller")
    reject_unknown(controller_table, {
        "leg_length", "position_feedback", "velocity_feedback",
        "forward_observation", "roll_restrained", "trace_stride",
    }, "controller")
    forward_observation = controller_table.get(
        "forward_observation", "wheel-odometry")
    if forward_observation not in FORWARD_OBSERVATIONS:
        raise ExperimentError(
            "controller.forward_observation must be wheel-odometry, "
            "base-truth, or contact-gated")
    trace_stride = controller_table.get("trace_stride", 10)
    if isinstance(trace_stride, bool) or not isinstance(trace_stride, int) or (
        trace_stride <= 0
    ):
        raise ExperimentError("controller.trace_stride must be a positive integer")
    controller = ControllerConfig(
        leg_length=positive_float(
            controller_table.get("leg_length", 0.18),
            "controller.leg_length"),
        position_feedback=boolean_value(
            controller_table, "position_feedback", True),
        velocity_feedback=boolean_value(
            controller_table, "velocity_feedback", True),
        forward_observation=forward_observation,
        roll_restrained=boolean_value(
            controller_table, "roll_restrained", False),
        trace_stride=trace_stride,
    )

    lqr_table = require_table(document, "lqr")
    reject_unknown(lqr_table, {"mode", "schedule_dir", "generate"}, "lqr")
    mode = lqr_table.get("mode")
    if mode not in ("existing", "generate"):
        raise ExperimentError("lqr.mode must be existing or generate")
    schedule_dir: Path | None = None
    generate: LqrGenerateConfig | None = None
    if mode == "existing":
        if "generate" in lqr_table:
            raise ExperimentError("lqr.generate is invalid in existing mode")
        schedule_dir = resolve_path(
            config_directory, lqr_table.get("schedule_dir"),
            "lqr.schedule_dir")
    else:
        if "schedule_dir" in lqr_table:
            raise ExperimentError("lqr.schedule_dir is invalid in generate mode")
        generate_table = lqr_table.get("generate")
        if not isinstance(generate_table, dict):
            raise ExperimentError("lqr.generate must be a TOML table")
        reject_unknown(generate_table, {
            "model_parameters", "q_diagonal", "r_diagonal",
            "leg_angle_difference_weight",
            "leg_angular_velocity_difference_weight", "yaw_inertia_source",
        }, "lqr.generate")
        model_parameters_value = generate_table.get("model_parameters")
        model_parameters = None if model_parameters_value is None else resolve_path(
            config_directory, model_parameters_value,
            "lqr.generate.model_parameters")
        yaw_source = generate_table.get("yaw_inertia_source", "base-link")
        if yaw_source not in ("base-link", "assembly"):
            raise ExperimentError(
                "lqr.generate.yaw_inertia_source must be base-link or assembly")
        generate = LqrGenerateConfig(
            model_parameters=model_parameters,
            q_diagonal=positive_vector(
                generate_table.get("q_diagonal"), STATE_COUNT,
                "lqr.generate.q_diagonal"),
            r_diagonal=positive_vector(
                generate_table.get("r_diagonal"), INPUT_COUNT,
                "lqr.generate.r_diagonal"),
            leg_angle_difference_weight=positive_float(
                generate_table.get("leg_angle_difference_weight"),
                "lqr.generate.leg_angle_difference_weight"),
            leg_angular_velocity_difference_weight=positive_float(
                generate_table.get(
                    "leg_angular_velocity_difference_weight"),
                "lqr.generate.leg_angular_velocity_difference_weight"),
            yaw_inertia_source=yaw_source,
        )

    analysis_table = document.get("analysis", {})
    if not isinstance(analysis_table, dict):
        raise ExperimentError("analysis must be a TOML table")
    reject_unknown(analysis_table, {"forward_linear"}, "analysis")
    analysis = AnalysisConfig(
        forward_linear=boolean_value(
            analysis_table, "forward_linear", False))

    case_values = document.get("case")
    if not isinstance(case_values, list) or not case_values:
        raise ExperimentError("at least one [[case]] is required")
    cases: list[CaseConfig] = []
    names: set[str] = set()
    for index, case_table in enumerate(case_values):
        if not isinstance(case_table, dict):
            raise ExperimentError(f"case[{index}] must be a TOML table")
        reject_unknown(case_table, {
            "name", "axis", "target", "command_rate",
            "target_hold_seconds", "stop_settle_seconds",
            "standing_seconds",
        }, f"case[{index}]")
        case_name = case_table.get("name")
        if not isinstance(case_name, str) or (
            SAFE_NAME.fullmatch(case_name) is None
        ):
            raise ExperimentError(
                f"case[{index}].name contains unsupported characters")
        if case_name in names:
            raise ExperimentError(f"duplicate case name: {case_name}")
        names.add(case_name)
        axis = case_table.get("axis")
        if axis not in ("forward", "yaw"):
            raise ExperimentError(f"case[{index}].axis must be forward or yaw")
        target = finite_float(case_table.get("target"), f"case[{index}].target")
        if target == 0.0:
            raise ExperimentError(f"case[{index}].target must be non-zero")
        cases.append(CaseConfig(
            name=case_name,
            axis=axis,
            target=target,
            command_rate=positive_float(
                case_table.get("command_rate"),
                f"case[{index}].command_rate"),
            target_hold_seconds=positive_float(
                case_table.get("target_hold_seconds", 3.0),
                f"case[{index}].target_hold_seconds"),
            stop_settle_seconds=positive_float(
                case_table.get("stop_settle_seconds", 2.0),
                f"case[{index}].stop_settle_seconds"),
            standing_seconds=positive_float(
                case_table.get("standing_seconds", 2.0),
                f"case[{index}].standing_seconds"),
        ))

    if analysis.forward_linear:
        if controller.position_feedback:
            raise ExperimentError(
                "forward linear analysis requires position_feedback = false")
        if not any(item.axis == "forward" for item in cases):
            raise ExperimentError(
                "forward linear analysis requires at least one forward case")

    if not paths.model.is_file():
        raise ExperimentError(f"model does not exist: {paths.model}")
    if schedule_dir is not None:
        validate_schedule_directory(schedule_dir)
    if generate is not None and generate.model_parameters is not None and (
        not generate.model_parameters.is_file()
    ):
        raise ExperimentError(
            f"model parameters do not exist: {generate.model_parameters}")

    return ExperimentConfig(
        version=1,
        name=name,
        source_path=source_path,
        paths=paths,
        build=build,
        controller=controller,
        lqr=LqrConfig(mode=mode, schedule_dir=schedule_dir, generate=generate),
        analysis=analysis,
        cases=tuple(cases),
    )


def validate_schedule_directory(path: Path) -> None:
    for name in ("current_model_schedule.h", "current_model_schedule.json"):
        if not (path / name).is_file():
            raise ExperimentError(f"schedule directory is missing {name}: {path}")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_hash(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), default=str).encode()
    return hashlib.sha256(encoded).hexdigest()


def lqr_fingerprint(config: ExperimentConfig) -> str:
    if config.lqr.mode == "existing":
        assert config.lqr.schedule_dir is not None
        return canonical_hash({
            "mode": "existing",
            "header": sha256_file(
                config.lqr.schedule_dir / "current_model_schedule.h"),
            "json": sha256_file(
                config.lqr.schedule_dir / "current_model_schedule.json"),
        })

    assert config.lqr.generate is not None
    generate = config.lqr.generate
    generator_files = (
        "build_current_model.py", "lqr_generator.py", "model_parameters.py",
        "verify_lqr.py",
    )
    source_hashes = {
        name: sha256_file(REPOSITORY_ROOT / "tools" / "lqr" / name)
        for name in generator_files
    }
    parameter_source = generate.model_parameters or config.paths.model
    return canonical_hash({
        "mode": "generate",
        "generate": serializable(generate),
        "parameter_source_hash": sha256_file(parameter_source),
        "generator_sources": source_hashes,
    })


def serializable(value: Any) -> Any:
    if hasattr(value, "__dataclass_fields__"):
        return {key: serializable(item) for key, item in asdict(value).items()}
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, tuple):
        return [serializable(item) for item in value]
    if isinstance(value, dict):
        return {key: serializable(item) for key, item in value.items()}
    if isinstance(value, list):
        return [serializable(item) for item in value]
    return value


def run_logged(
    command: list[str],
    cwd: Path,
    log_path: Path,
    commands: list[list[str]],
) -> None:
    commands.append(command)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    print("+ " + subprocess.list2cmdline(command))
    completed = subprocess.run(
        command, cwd=cwd, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        check=False)
    log_path.write_text(completed.stdout, encoding="utf-8")
    if completed.stdout:
        print(completed.stdout, end="")
    if completed.returncode != 0:
        raise ExperimentError(
            f"command failed with exit code {completed.returncode}; "
            f"see {log_path}")


def prepare_schedule(
    config: ExperimentConfig,
    fingerprint: str,
    commands: list[list[str]],
) -> Path:
    if config.lqr.mode == "existing":
        assert config.lqr.schedule_dir is not None
        return config.lqr.schedule_dir

    assert config.lqr.generate is not None
    output = config.build.root / "schedules" / fingerprint[:16]
    metadata_path = output / "fingerprint.json"
    if metadata_path.is_file() and all((output / name).is_file() for name in (
        "current_model_schedule.h", "current_model_schedule.json",
    )):
        metadata = json.loads(metadata_path.read_text(encoding="ascii"))
        if metadata.get("fingerprint") == fingerprint:
            print(f"Reusing schedule {output}")
            return output

    generate = config.lqr.generate
    command = [
        sys.executable,
        str(REPOSITORY_ROOT / "tools/lqr/build_current_model.py"),
        "--model", str(config.paths.model),
        "--output", str(output),
        "--report", str(output / "report.md"),
        "--q-diagonal", *(str(value) for value in generate.q_diagonal),
        "--r-diagonal", *(str(value) for value in generate.r_diagonal),
        "--leg-angle-difference-weight",
        str(generate.leg_angle_difference_weight),
        "--leg-angular-velocity-difference-weight",
        str(generate.leg_angular_velocity_difference_weight),
        "--yaw-inertia-source", generate.yaw_inertia_source,
    ]
    if generate.model_parameters is not None:
        command.extend([
            "--reuse-model-parameters", str(generate.model_parameters),
        ])
    run_logged(
        command, REPOSITORY_ROOT, output / "generate.log", commands)
    validate_schedule_directory(output)
    metadata_path.write_text(json.dumps({
        "fingerprint": fingerprint,
        "inputs": serializable(generate),
    }, indent=2) + "\n", encoding="ascii")
    return output


def prepare_build(
    config: ExperimentConfig,
    schedule_dir: Path,
    schedule_fingerprint: str,
    run_logs: Path,
    commands: list[list[str]],
) -> tuple[Path, str]:
    build_fingerprint = canonical_hash({
        "schedule": schedule_fingerprint,
        "configuration": config.build.configuration,
        "generator": config.build.generator,
        "cmake_args": config.build.cmake_args,
    })
    build_dir = config.build.root / "cmake" / build_fingerprint[:16]
    configure = [
        "cmake", "-S", str(REPOSITORY_ROOT), "-B", str(build_dir),
        f"-DBALANCE_LQR_SCHEDULE_DIR={schedule_dir}",
        f"-DCMAKE_BUILD_TYPE={config.build.configuration}",
        "-DBUILD_TESTING=ON",
        "-DBALANCE_BUILD_GUI=OFF",
    ]
    if config.build.generator is not None:
        configure.extend(["-G", config.build.generator])
    configure.extend(config.build.cmake_args)
    run_logged(
        configure, REPOSITORY_ROOT, run_logs / "configure.log", commands)
    build = [
        "cmake", "--build", str(build_dir),
        "--config", config.build.configuration,
        "--target", "rm_balance_performance", "lqr_controller_test",
    ]
    run_logged(build, REPOSITORY_ROOT, run_logs / "build.log", commands)
    test = [
        "ctest", "--test-dir", str(build_dir),
        "--build-config", config.build.configuration,
        "--output-on-failure", "-R", "^lqr_controller$",
    ]
    run_logged(test, REPOSITORY_ROOT, run_logs / "lqr-test.log", commands)
    return build_dir, build_fingerprint


def copy_schedule_record(schedule_dir: Path, run_directory: Path) -> Path:
    record = run_directory / "schedule"
    record.mkdir()
    for name in (
        "current_model_schedule.h", "current_model_schedule.json",
        "report.md", "generate.log", "fingerprint.json",
    ):
        source = schedule_dir / name
        if source.is_file():
            shutil.copy2(source, record / name)
    return record


def find_executable(build_dir: Path, configuration: str) -> Path:
    names = ("rm_balance_performance", "rm_balance_performance.exe")
    candidates = [
        build_dir / name for name in names
    ] + [
        build_dir / configuration / name for name in names
    ]
    found = [path for path in candidates if path.is_file()]
    if len(found) != 1:
        raise ExperimentError(
            f"expected one benchmark executable under {build_dir}, found {found}")
    return found[0]


def case_command(
    executable: Path,
    config: ExperimentConfig,
    case: CaseConfig,
    output: Path,
) -> list[str]:
    command = [
        str(executable), str(config.paths.model), str(output),
        "--name", case.name,
        "--axis", case.axis,
        "--target", str(case.target),
        "--command-rate", str(case.command_rate),
        "--standing-seconds", str(case.standing_seconds),
        "--target-hold-seconds", str(case.target_hold_seconds),
        "--stop-settle-seconds", str(case.stop_settle_seconds),
        "--leg-length", str(config.controller.leg_length),
        "--trace-stride", str(config.controller.trace_stride),
    ]
    if config.controller.roll_restrained:
        command.extend(["--roll-restraint", "on"])
    if config.controller.forward_observation != "wheel-odometry":
        command.extend([
            "--forward-observation", config.controller.forward_observation,
        ])
    if not config.controller.position_feedback:
        command.extend(["--position-feedback", "off"])
    if not config.controller.velocity_feedback:
        command.extend(["--velocity-feedback", "off"])
    return command


def combine_case_summaries(case_directories: list[Path], output: Path) -> None:
    header: list[str] | None = None
    rows: list[dict[str, str]] = []
    for directory in case_directories:
        with (directory / "summary.csv").open(newline="", encoding="ascii") as source:
            reader = csv.DictReader(source)
            current_header = reader.fieldnames
            if current_header is None:
                raise ExperimentError(f"summary has no header: {directory}")
            if header is None:
                header = current_header
            elif header != current_header:
                raise ExperimentError("case summary columns do not match")
            rows.extend(reader)
    assert header is not None
    with output.open("w", newline="", encoding="ascii") as destination:
        writer = csv.DictWriter(destination, fieldnames=header)
        writer.writeheader()
        writer.writerows(rows)


def git_metadata() -> dict[str, Any]:
    def capture(*arguments: str) -> str:
        completed = subprocess.run(
            ["git", *arguments], cwd=REPOSITORY_ROOT, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False)
        return completed.stdout.strip() if completed.returncode == 0 else "unknown"

    status = capture("status", "--short")
    return {
        "head": capture("rev-parse", "HEAD"),
        "dirty": bool(status),
        "status": status.splitlines(),
    }


def resolved_document(config: ExperimentConfig) -> dict[str, Any]:
    return {
        "version": config.version,
        "name": config.name,
        "source_path": str(config.source_path),
        "paths": serializable(config.paths),
        "build": serializable(config.build),
        "controller": serializable(config.controller),
        "lqr": serializable(config.lqr),
        "analysis": serializable(config.analysis),
        "cases": serializable(config.cases),
    }


def execute(config: ExperimentConfig, dry_run: bool) -> Path | None:
    schedule_fingerprint = lqr_fingerprint(config)
    resolved = resolved_document(config)
    experiment_fingerprint = canonical_hash(resolved)
    if dry_run:
        print(json.dumps({
            "resolved": resolved,
            "schedule_fingerprint": schedule_fingerprint,
            "experiment_fingerprint": experiment_fingerprint,
        }, indent=2))
        return None

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    run_directory = (
        config.paths.output_root / config.name /
        f"{timestamp}-{experiment_fingerprint[:8]}")
    if run_directory.exists():
        raise ExperimentError(f"run directory already exists: {run_directory}")
    run_directory.mkdir(parents=True)
    shutil.copy2(config.source_path, run_directory / "experiment.toml")
    (run_directory / "resolved_config.json").write_text(
        json.dumps(resolved, indent=2) + "\n", encoding="utf-8")

    commands: list[list[str]] = []
    schedule_dir = prepare_schedule(
        config, schedule_fingerprint, commands)
    schedule_record = copy_schedule_record(schedule_dir, run_directory)
    build_dir, build_fingerprint = prepare_build(
        config, schedule_dir, schedule_fingerprint,
        run_directory / "logs", commands)
    executable = find_executable(build_dir, config.build.configuration)

    case_directories: list[Path] = []
    forward_traces: list[Path] = []
    for case in config.cases:
        output = run_directory / "cases" / case.name
        command = case_command(executable, config, case, output)
        run_logged(
            command, REPOSITORY_ROOT,
            run_directory / "logs" / f"case-{case.name}.log", commands)
        case_directories.append(output)
        if case.axis == "forward":
            forward_traces.append(output / "trace.csv")

    combine_case_summaries(
        case_directories, run_directory / "summary.csv")
    if config.analysis.forward_linear:
        analysis_output = run_directory / "analysis" / "forward-linear"
        command = [
            sys.executable,
            str(REPOSITORY_ROOT / "tools/lqr/forward_response.py"),
            "--schedule", str(schedule_dir / "current_model_schedule.json"),
            "--output", str(analysis_output),
            "--mujoco-trace", *(str(path) for path in forward_traces),
            "--leg-length", str(config.controller.leg_length),
        ]
        run_logged(
            command, REPOSITORY_ROOT,
            run_directory / "logs" / "forward-linear.log", commands)

    metadata = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "platform": platform.platform(),
        "python": sys.version,
        "git": git_metadata(),
        "model_sha256": sha256_file(config.paths.model),
        "schedule_sha256": sha256_file(
            schedule_record / "current_model_schedule.json"),
        "schedule_fingerprint": schedule_fingerprint,
        "build_fingerprint": build_fingerprint,
        "experiment_fingerprint": experiment_fingerprint,
        "schedule_directory": str(schedule_dir),
        "schedule_record": str(schedule_record),
        "build_directory": str(build_dir),
        "executable": str(executable),
        "commands": commands,
    }
    (run_directory / "metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(f"Experiment results: {run_directory}")
    return run_directory


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    arguments = parser.parse_args()
    try:
        config = load_config(arguments.config)
        execute(config, arguments.dry_run)
    except ExperimentError as error:
        print(f"run_experiment: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
