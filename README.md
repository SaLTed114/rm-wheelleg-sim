# rm-balance-sim

## Overview

`rm-balance-sim` is a MuJoCo simulation and control-development project for a wheel-legged balancing robot. It contains the validated platform-independent C11 control core, a parallel C++20 startup-control implementation, and a C++ simulation layer, with virtual-leg kinematics, gain-scheduled LQR control, an interactive Dear ImGui viewer, automated tests, and headless performance tools.

The current development state and unresolved problems are recorded in [`docs/notes/project-context.md`](docs/notes/project-context.md). The scope and architecture of the parallel C++20 control core are recorded in [`docs/notes/cpp-control-core.md`](docs/notes/cpp-control-core.md). Current experiments are recorded in [`docs/notes/controller-experiment-log.md`](docs/notes/controller-experiment-log.md); closed experiment cycles and generated LQR validation are under [`docs/archive/`](docs/archive/).

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

Append `--keyboard` to drive manually. `W/S` or the up/down arrows command forward motion, Shift raises the speed limit from `2.0 m/s` to `2.7 m/s`, and `A/D` or the left/right arrows rotate the virtual gimbal. `T` starts or cancels the step-docking task; successful completion automatically clears its keyboard latch so the next task starts with one press. Space pauses, `R` or Backspace resets, Escape exits, and the mouse controls the camera when the UI is not capturing input.

Keyboard mode reveals three terrain lanes in front of the spawn point. The center lane starts with a `200 mm`-high, `800 mm`-long, `1.0 m`-wide platform, followed immediately by a `300 mm`-high, `2.0 m`-long platform. A full-width curb, `100 mm` long and `50 mm` high, sits on the leading edge of the second platform. The right-hand lane (`y=-1.5 m`) retains a triangular `15 deg` ramp onto a `200 mm`-high, `2.0 x 2.0 m` platform. The left-hand lane (`y=+1.0 m`) retains a standalone triangular ramp, `860 mm` wide and `17 deg`, with a `350 mm` summit and no top platform. All features remain buried and non-visible for benchmark cases and non-keyboard simulation.

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

### C++20 startup control

The parallel C++20 control core currently covers system enable/restart, the
startup sequence, and fixed-position/fixed-heading LQR balance only. It does
not implement drive commands, gimbal following, support/landing, STEP_DOCK,
jump, or spin behavior.

Its MuJoCo integration is currently headless. The existing interactive
`rm_balance_sim` executable still runs the validated C core; there is no C++
controller viewer mode yet. Run the focused C++ checks with:

```bash
ctest --test-dir build --output-on-failure \
  -R "cpp_controller_startup|cpp_control_modules|mujoco_cpp_startup"
```

The three checks cover C/C++ startup transition parity, isolated C++ modules
and output safety, and an independent MuJoCo standing run respectively.

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

The open-loop figure-eight motion case is kept out of the default axis suite.
Run it explicitly with:

```bash
./build/rm_balance_performance \
  models/MJCF/COD-2026RoboMaster-Balance.xml \
  build/performance/figure-eight \
  --case figure_eight_open_loop --trace-stride 1
```

The same case can be replayed in the GUI with
`./build/rm_balance_sim <model.xml> --case figure_eight_open_loop`.

### Jump impulse benchmark

Run the retained negative, representative jump, and stress cases with:

```bash
./build/rm_balance_jump_impulse \
  models/MJCF/COD-2026RoboMaster-Balance.xml \
  build/jump-impulse
```

Replay one force level in the GUI with, for example,
`./build/rm_balance_sim <model.xml> --case jump_impulse_f240_t90ms`.
The permanent registry contains `jump_impulse_f140`,
`jump_impulse_f240_t90ms`, and `jump_impulse_f240_t120ms`; the intermediate
force and hold-time sweep remains documented in the experiment log.

### Step docking benchmark

Run the `0.38 m` leg, `2.0 m/s` reference, `200 mm` step cases with:

```bash
./build/rm_balance_step_dock \
  models/MJCF/COD-2026RoboMaster-Balance.xml \
  build/step-dock
```

The permanent registry contains only the production `step_dock_complete` case.
It sends the ordinary operator STEP_DOCK command and exercises the complete
pure-C task state machine. MuJoCo contact truth is used only for benchmark
assertions and never drives a phase transition or modifies the controller
command. Replay it in the GUI with `--case step_dock_complete`.

In `--keyboard` mode, press `T` to start or cancel the step-docking task. The
controller extends the legs while accepting ordinary drive input, aligns the chassis
front, detects impact, disables all actuators for one control cycle, executes
the fixed transfer and hold, then recovers attitude with position and heading
feedback masked. Once stable, it captures the current position and heading and
verifies the full balance controller before completing. Successful recovery
automatically returns to normal ACTIVE control and clears the keyboard task
latch, so the next task starts with one `T` press. The control core still
requires NORMAL to be observed once before rearming, preventing a held external
STEP_DOCK command from retriggering; recovery timeout remains latched until
system reset.
