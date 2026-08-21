# rm-balance-sim

## Overview

`rm-balance-sim` is a MuJoCo simulation and control-development project for a wheel-legged balancing robot. It contains the validated platform-independent C11 control core, interactive visualization, automated tests, and headless benchmark tools. The preserved C++20 control implementation is dead code on the Fudan adaptation branch and is excluded from builds and tests.

The current development state and unresolved problems are recorded in [`docs/notes/project-context.md`](docs/notes/project-context.md). The historical C++20 implementation is recorded in [`docs/notes/cpp-control-core.md`](docs/notes/cpp-control-core.md). Current experiments are recorded in [`docs/notes/controller-experiment-log.md`](docs/notes/controller-experiment-log.md), with closed experiment cycles and generated LQR validation under [`docs/archive/`](docs/archive/).

The robot assets under `models/` are derived from open-source models released by the Liaoning University of Science and Technology COD RoboMaster team and the Fudan wheel-legged robot project. See [`models/README.md`](models/README.md) for attribution and licensing information.

## Build

CMake 3.22 or newer, a C/C++ compiler, and MuJoCo are required. Viewers additionally require GLFW and OpenGL; the full diagnostic GUI also requires Dear ImGui. Local dependency source trees can be placed under the ignored `third_party/` directory.

### Windows

The recommended local layout is `third_party/mujoco-3.9.0`, `third_party/glfw`, and `third_party/imgui`. `MUJOCO_ROOT` must contain `include`, `lib`, and `bin`.

```powershell
$mujocoRoot = (Resolve-Path .\third_party\mujoco-3.9.0).Path
$glfwRoot = (Resolve-Path .\third_party\glfw).Path
$imguiRoot = (Resolve-Path .\third_party\imgui).Path

cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  "-DMUJOCO_ROOT=$mujocoRoot" `
  "-DFETCHCONTENT_SOURCE_DIR_GLFW=$glfwRoot" `
  "-DIMGUI_ROOT=$imguiRoot"
cmake --build build --config Release --parallel
```

If `FETCHCONTENT_SOURCE_DIR_GLFW` is omitted, CMake downloads the pinned GLFW revision. ImGui is never downloaded automatically and must be supplied through `IMGUI_ROOT` or `third_party/imgui`.

### Ubuntu

Install the GUI development packages, then configure with the ImGui path. Pass `MUJOCO_ROOT` as well when MuJoCo is not installed under `/usr` or `/usr/local`.

```bash
sudo apt install libglfw3-dev libgl1-mesa-dev

cmake -S . -B build \
  -DMUJOCO_ROOT=/path/to/mujoco \
  -DIMGUI_ROOT=/path/to/imgui
cmake --build build -j
```

### Headless

The control tests and benchmark tools can be built without GLFW or ImGui:

```bash
cmake -S . -B build \
  -DBALANCE_BUILD_GUI=OFF \
  -DMUJOCO_ROOT=/path/to/mujoco
cmake --build build -j
```

## Run

### Diagnostic GUI

The full `rm_balance_sim` application uses the validated C control core and includes the diagnostic UI.

Windows:

```powershell
.\build\Release\rm_balance_sim.exe `
  .\models\MJCF\Fudan-2026RoboMaster-Balance.xml
```

Linux:

```bash
./build/rm_balance_sim models/MJCF/Fudan-2026RoboMaster-Balance.xml
```

Add `--keyboard` for manual control. `W/S` or Up/Down command forward motion, Shift raises the speed limit, `A/D` or Left/Right rotate the virtual gimbal, `T` starts or cancels STEP_DOCK, Space pauses, `R` or Backspace resets, Escape exits, and the mouse controls the camera when the UI is not capturing input.

Add `--trace <csv-path>` together with `--keyboard` to record control commands, controller states, estimator diagnostics, MuJoCo truth, contacts, IMU feedback, and wheel torques. The trace is flushed every 0.1 seconds.

### C++20 Historical Source

`src/control_cpp/` and `src/sim/cpp/` are retained only as historical source on this branch. CMake does not expose a C++ control library, runner, viewer, or test target, and the code is not adapted to the Fudan model.

## Tests

Run the complete test suite with:

```powershell
ctest --test-dir build -C Release --output-on-failure
```

```bash
ctest --test-dir build --output-on-failure
```

## Benchmarks

The build provides dedicated executables for performance, drop, ramp-course, ramp-jump, jump-impulse, step-docking, and trim-scan scenarios. Benchmark outputs belong under the ignored build tree; most runs write a summary and optional trace to the selected output directory.

A representative performance run is:

```bash
./build/rm_balance_performance \
  models/MJCF/Fudan-2026RoboMaster-Balance.xml \
  build/performance/baseline
```

Dedicated jump-impulse and step-docking runs use the same model/output pattern:

```bash
./build/rm_balance_jump_impulse \
  models/MJCF/Fudan-2026RoboMaster-Balance.xml \
  build/jump-impulse

./build/rm_balance_step_dock \
  models/MJCF/Fudan-2026RoboMaster-Balance.xml \
  build/step-dock
```

Registered cases can be replayed in the diagnostic GUI with `./build/rm_balance_sim <model.xml> --case <case-name>`. Case definitions, retained negative cases, parameter sweeps, and acceptance results are documented in the experiment log rather than duplicated here.

## Leg-Adapter Calibration

The MuJoCo adapter joint offsets are generated from closed-chain poses in the `0.18-0.21 m` standing range. The `sim` Conda environment must provide MuJoCo, NumPy, and SciPy.

```powershell
conda run -n sim python tools/calibration/calibrate_leg_adapter.py --check
```

Use `--write` only after an intentional MJCF or calibration change. It updates the tracked JSON record and C++ header when the standing-range accuracy and full-range non-regression gates pass.
