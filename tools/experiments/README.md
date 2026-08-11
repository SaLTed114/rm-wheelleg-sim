# Experiment runner

`run_experiment.py` turns one human-readable TOML file into an isolated LQR schedule, CMake build, set of MuJoCo cases, and optional linear-model comparison. Run it with the repository's `sim` environment, which provides Python 3.11 and the numerical dependencies used by the LQR generator:

```bash
conda run -n sim python tools/experiments/run_experiment.py \
  tools/experiments/lqr_schedule_generation_example.toml
```

Use `--dry-run` to validate and print the fully resolved configuration without creating build or result files. Relative paths in TOML are resolved from the TOML file's directory.

## Configuration

Each file describes one LQR candidate and one or more cases. The example covers the supported fields. `lqr.mode = "generate"` accepts the ten state weights, four input weights, and two differential leg-mode weights. Yaw inertia is always the complete-assembly value extracted at the model's configured reference leg length; it is not a selectable experiment parameter. When `model_parameters` is present, model extraction is reused while Q/R are regenerated. Omit it to extract parameters from the selected MJCF. For forward cases, `command_rate` configures the isolated controller's shared acceleration/deceleration ramp as well as the scenario timing.

`[[turn_sweep]]` expands a list of positive forward velocities and signed yaw rates into independent steady-turn cases. Each case waits for the measured forward speed and pitch state to remain ready for `0.5 s` before applying yaw; an entry that is not ready within `5 s` is recorded without evaluating the turn. The repository's initial `0.18 m` coarse envelope is:

```bash
conda run -n sim python tools/experiments/run_experiment.py \
  tools/experiments/turn_envelope_coarse.toml
```

The preset covers `1-3 m/s` and signed `0.5*pi`, `pi`, and `1.5*pi rad/s`. Turn response, contact, and saturation remain separate result dimensions: a contact event is visible but does not overwrite the tracking result.

The first trajectory derived from that envelope is the built-in `figure_eight_open_loop` case (`2 m/s`, `+/-pi rad/s`). Run it headlessly and compare the integrated reference with the actual axle path using:

```bash
build/rm_balance_performance models/MJCF/COD-2026RoboMaster-Balance.xml \
  build/performance/figure-eight --case figure_eight_open_loop --trace-stride 1
python tools/experiments/plot_trajectory.py \
  build/performance/figure-eight/trace.csv \
  build/performance/figure-eight/trajectory.svg
```

To run an existing schedule without regeneration, replace the generation tables with:

```toml
[lqr]
mode = "existing"
schedule_dir = "../lqr/generated"
```

`forward_linear = true` compares forward ramp/hold samples with the generated fixed-length `A/B/K`. It reads the `forward` mode from new traces so `S` is ignored in velocity mode and restored in hold mode. Historical traces using the old `drive` column or `position_feedback_enabled = 0` are also accepted. Give the case enough `standing_seconds` for the actual plant to settle; the example uses eight seconds.

`formal_lqr_validation.toml` is the reproducible `+/-3 m/s`, `5 m/s^2` longitudinal acceptance case for the formal schedule:

```bash
conda run -n sim python tools/experiments/run_experiment.py \
  tools/experiments/formal_lqr_validation.toml
```

Before building a full candidate schedule, `forward_weight_sweep.py` can use the same traces and fixed-length model to screen forward-state and actuator weights. Omitted weight groups retain the values stored in the schedule:

```bash
conda run -n sim python tools/lqr/forward_weight_sweep.py \
  --schedule tools/lqr/generated/current_model_schedule.json \
  --output build/experiments/forward-weight-sweep \
  --mujoco-trace path/to/case-a/trace.csv path/to/case-b/trace.csv \
  --ds-weights 60 90 120 180 \
  --wheel-weights 3.2 2.4 1.6 \
  --pitch-limit-deg 2.5 --t90-limit 1.0
```

The ranking also records common-leg motion, reverse displacement, velocity overshoot, and predicted wheel/leg torque. The sweep is only an ideal linear prefilter: its A/B/K response is useful for choosing directions and candidate spacing, not for accepting a candidate or requiring the nonlinear plant to match throughout a transient. Shortlisted weights must be accepted using the actual MuJoCo response from a generated schedule and experiment cases.

Historical Q140 and Q_DS/R-only boundary-search results are summarized in `docs/notes/performance-baseline.md`; their one-off candidate configurations are intentionally not kept as active experiment presets. New candidates should be expressed explicitly in a TOML derived from `lqr_schedule_generation_example.toml`, while formal regression always uses `formal_lqr_validation.toml`.

## Build and output

Schedules and CMake builds are cached under `build.root` by their input fingerprints. Changing only the cases reuses both. Experiment results are never written into the source tree's formal generated files or validation report. Each run contains:

- the original TOML and fully resolved JSON;
- git, model, schedule, build, and command metadata;
- one `summary.csv` and `trace.csv` directory per case;
- an aggregate summary, optional forward linear comparison, and a dependency-free `analysis/turn-envelope.svg` for turn sweeps;
- generation, configuration, build, case, and analysis logs.

The runner disables the GUI in experiment builds, so GLFW is not needed. On a machine where MuJoCo is not discoverable from the active environment, add its CMake definition to `build.cmake_args`. Windows native builds can also select a generator and local dependency paths:

```toml
[build]
root = "../../build/experiments"
configuration = "Release"
generator = "Visual Studio 17 2022"
cmake_args = [
  "-DMUJOCO_ROOT=C:/path/to/mujoco-3.9.0",
]
```
