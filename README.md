# rm-balance-sim

## Overview

`rm-balance-sim` is a MuJoCo simulation and control-development project for a wheel-legged balancing robot. It consists of a platform-independent C11 control core and a C++17 simulation layer, with virtual-leg kinematics, gain-scheduled LQR control, an interactive Dear ImGui viewer, automated tests, and headless performance tools.

The current development state and unresolved problems are recorded in [`docs/notes/project-context.md`](docs/notes/project-context.md). LQR generation and measured performance results are recorded in [`docs/notes/lqr-validation.md`](docs/notes/lqr-validation.md) and [`docs/notes/performance-baseline.md`](docs/notes/performance-baseline.md).

The robot assets under `models/` are derived from the open-source model released by the Liaoning University of Science and Technology COD RoboMaster team. See [`models/README.md`](models/README.md) for attribution and licensing information.

## Build

CMake 3.22 or newer, a C/C++ compiler, and MuJoCo are required. The GUI additionally requires GLFW, OpenGL, and Dear ImGui. Local dependency source trees can be placed under the ignored `third_party/` directory.

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

Install the GUI development packages first:

```bash
sudo apt install libglfw3-dev libgl1-mesa-dev
```

If MuJoCo is installed under a standard system prefix such as `/usr` or `/usr/local`, only the ImGui path is needed:

```bash
cmake -S . -B build -DIMGUI_ROOT=/path/to/imgui
cmake --build build -j
```

For an extracted MuJoCo SDK, pass the directory containing its `include` and `lib` directories:

```bash
cmake -S . -B build \
  -DMUJOCO_ROOT=/path/to/mujoco \
  -DIMGUI_ROOT=/path/to/imgui
cmake --build build -j
```

### Headless build

The control tests and benchmark tools can be built without GLFW or ImGui:

```bash
cmake -S . -B build-headless \
  -DBALANCE_BUILD_GUI=OFF \
  -DMUJOCO_ROOT=/path/to/mujoco
cmake --build build-headless -j
```

## Run

### Interactive simulator

Windows:

```powershell
.\build\Release\rm_balance_sim.exe `
  .\models\MJCF\COD-2026RoboMaster-Balance.xml
```

Linux:

```bash
./build/rm_balance_sim models/MJCF/COD-2026RoboMaster-Balance.xml
```

Append `--keyboard` to drive manually. `W/S` or the up/down arrows command forward motion, Shift raises the speed limit, and `A/D` or the left/right arrows rotate the virtual gimbal. Space pauses, `R` or Backspace resets, Escape exits, and the mouse controls the camera when the UI is not capturing input.

Keyboard mode also reveals two terrain features in front of the spawn point. The right-hand lane (`y=-1.5 m`) has a triangular `15 deg` ramp onto a `200 mm`-high, `2.0 x 2.0 m` platform. The left-hand lane (`y=+1.0 m`) is a standalone triangular ramp, `860 mm` wide and `17 deg`, with a `350 mm` summit and no top platform. Both features remain buried and non-visible for benchmark cases and non-keyboard simulation.

To capture a control-step diagnostic trace while driving, add `--trace <csv-path>` together with `--keyboard`. The CSV includes keyboard commands, controller modes, `S`/`DS`, velocity-estimator innovations and gating, MuJoCo base/wheel truth, contacts, IMU feedback, and wheel torques. It is flushed every 0.1 seconds so an intermittent failure can be inspected after closing the simulator.

### Tests

Windows:

```powershell
ctest --test-dir build -C Release --output-on-failure
```

Linux:

```bash
ctest --test-dir build --output-on-failure
```

### Leg-adapter calibration

The MuJoCo adapter joint offsets are generated from closed-chain poses in the
`0.18–0.21 m` standing range. The `armsim` Conda environment must provide
MuJoCo, NumPy, and SciPy.

```powershell
conda run -n armsim python tools/calibration/calibrate_leg_adapter.py --check
```

Use `--write` after an intentional MJCF or calibration change. It updates the
tracked JSON record and C++ header only when the standing-range accuracy and
full-range non-regression gates pass.

### Performance benchmark

Windows:

```powershell
.\build\Release\rm_balance_performance.exe `
  .\models\MJCF\COD-2026RoboMaster-Balance.xml `
  .\build\performance\baseline
```

Linux:

```bash
./build/rm_balance_performance \
  models/MJCF/COD-2026RoboMaster-Balance.xml \
  build/performance/baseline
```

The benchmark writes `summary.csv` and `trace.csv` into the selected output directory under the ignored build tree.

The fixed cross figure-eight motion case is kept out of the default axis suite.
Run it explicitly with:

```bash
./build/rm_balance_performance \
  models/MJCF/COD-2026RoboMaster-Balance.xml \
  build/performance/figure-eight \
  --case figure_eight_cross --trace-stride 1
```

The same case can be replayed in the GUI with
`./build/rm_balance_sim <model.xml> --case figure_eight_cross`.
