# Experiment runner

`run_experiment.py` turns one human-readable TOML file into an isolated LQR
schedule, CMake build, set of MuJoCo cases, and optional linear-model
comparison. Run it with the repository's `sim` environment, which provides
Python 3.11 and the numerical dependencies used by the LQR generator:

```bash
conda run -n sim python tools/experiments/run_experiment.py \
  tools/experiments/forward_ab_example.toml
```

Use `--dry-run` to validate and print the fully resolved configuration without
creating build or result files. Relative paths in TOML are resolved from the
TOML file's directory.

## Configuration

Each file describes one LQR candidate and one or more cases. The example covers
the supported fields. `lqr.mode = "generate"` accepts the ten state weights,
four input weights, two differential leg-mode weights, and yaw inertia source.
When `model_parameters` is present, model extraction is reused while Q/R are
regenerated. Omit it to extract parameters from the selected MJCF.

To run an existing schedule without regeneration, replace the generation
tables with:

```toml
[lqr]
mode = "existing"
schedule_dir = "../lqr/generated"
```

`forward_linear = true` compares forward ramp/hold samples with the generated
fixed-length `A/B/K`. It requires `position_feedback = false`, uses the last
second of the standing phase as trim, and forces the predicted `S` error to
zero. Give the case enough `standing_seconds` for the actual plant to settle;
the example uses eight seconds.

## Build and output

Schedules and CMake builds are cached under `build.root` by their input
fingerprints. Changing only the cases reuses both. Experiment results are never
written into the source tree's formal generated files or validation report.
Each run contains:

- the original TOML and fully resolved JSON;
- git, model, schedule, build, and command metadata;
- one `summary.csv` and `trace.csv` directory per case;
- an aggregate summary and optional forward linear comparison;
- generation, configuration, build, case, and analysis logs.

The runner disables the GUI in experiment builds, so GLFW is not needed. On a
machine where MuJoCo is not discoverable from the active environment, add its
CMake definition to `build.cmake_args`. Windows native builds can also select a
generator and local dependency paths:

```toml
[build]
root = "../../build/experiments"
configuration = "Release"
generator = "Visual Studio 17 2022"
cmake_args = [
  "-DMUJOCO_ROOT=C:/path/to/mujoco-3.9.0",
]
```
