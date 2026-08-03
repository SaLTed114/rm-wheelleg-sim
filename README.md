# rm-balance-sim

Minimal MuJoCo host for the wheel-legged balance robot. The runtime is split into
a C11 control core and a C++17 simulation layer. The current controller uses
virtual-leg kinematics, Jacobian-transpose actuation, and a gain-scheduled LQR.
PD and LQR implementations are isolated as control laws beneath the control
core, distinct from the top-level controller facade.
The demo keeps the system off while the robot settles, then enables it and
sends a simulated balance-restart pulse so the controller can position the
legs and enter balance control.
The simulation talks to the C controller through one operator-command input.
Observability is provided by an on-demand, caller-owned controller snapshot;
the state machine and low-level control remain internal. `SimulationRunner`
captures its latest snapshot after construction, reset, and each controller
execution, so the GUI and tests never need to read controller internals.
Forward-velocity and yaw-rate references pass through independent linear ramps
limited to `3 m/s, 5 m/s^2` and `4*pi rad/s, 15 rad/s^2`. Roll and roll rate
are observable through the snapshot but are not yet used for compensation;
the lower `1.5*pi rad/s` moving-turn envelope is also not yet enforced.
The current 1 ms LQR schedule was retuned around `0.16 m` and `0.18 m` leg
lengths. It completes the diagnostic `+/-3 m/s` translation sweep at both
lengths; in-place yaw above `pi rad/s` remains an investigation target rather
than a validated operating range.

The robot assets under `models/` are imported from the open-source model released
by the Liaoning University of Science and Technology COD RoboMaster team. See
`models/README.md` for upstream attribution, revision, and licensing notes.

## Linux build with the official MuJoCo SDK

Download and extract the official Linux release, then point `MUJOCO_ROOT` at
the directory containing `include` and `lib`:

```bash
cmake -S . -B build \
  -DMUJOCO_ROOT=/path/to/mujoco-3.9.0
cmake --build build -j
ctest --test-dir build --output-on-failure
```

CMake also accepts a MuJoCo Python package directory as a fallback when an
official SDK is not available.

Run the interactive simulator:

```bash
./build/rm_balance_sim \
  models/MJCF/COD-2026RoboMaster-Balance.xml
```

The simulation runs in real time until the window is closed. Use the left mouse
button to rotate, right mouse button to pan, middle button or wheel to zoom,
Space to pause, and Backspace or `R` to reset.

The interactive simulator first keeps `SYSTEM_OFF` for two seconds while the
robot falls and settles. It then enables the system and emits one simulated
balance-restart event. `LEG_POSITIONING` moves both virtual legs toward
`0.18 m / -pi/2`; once stable, `ENGAGING` enables the current-model LQR and
`54 N` axial support per leg. No mocap support or pose teleport is used. The
simulator then repeats standing, `+/-0.25 m/s` travel, and `+/-1.57 rad/s` yaw
phases. It prints the current phase and ten-element state vector every 0.5 s.
The chassis, leg links, and wheels collide with the infinite ground plane,
while self-collision remains disabled. A 40 by 40 m checkerboard and gradient
skybox provide the visible environment.

## Windows build with Visual Studio

Download and extract the official MuJoCo Windows release, then configure with
the Visual Studio generator. `MUJOCO_ROOT` must contain `include`, `lib`, and
`bin`; in particular, a native build needs both `lib/mujoco.lib` and
`bin/mujoco.dll`. A MuJoCo Python wheel that only contains headers and the DLL
is not sufficient for linking the C++ targets.

The recommended local layout is `third_party/mujoco-3.9.0`. The directory is
ignored by Git. If GLFW source is also available at `third_party/glfw`, pass it
to FetchContent explicitly so configuration does not require a Git clone:

```powershell
$mujocoRoot = (Resolve-Path .\third_party\mujoco-3.9.0).Path
$glfwRoot = (Resolve-Path .\third_party\glfw).Path

cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  "-DMUJOCO_ROOT=$mujocoRoot" `
  "-DFETCHCONTENT_SOURCE_DIR_GLFW=$glfwRoot"
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
.\build\Release\rm_balance_sim.exe `
  .\models\MJCF\COD-2026RoboMaster-Balance.xml
```

Without `FETCHCONTENT_SOURCE_DIR_GLFW`, CMake clones the pinned GLFW revision
during configuration. This requires working Git network access. On Windows, an
interrupted FetchContent clone can leave Git child processes and a read-only
temporary pack file under `build/_deps`; stop those processes before deleting
the build directory. Passing `MUJOCO_ROOT` only configures MuJoCo and does not
change how GLFW is obtained. Prefer the explicit `-D` argument over a
shell-local environment variable, especially when reusing a build directory.

The GUI uses a 1 ms physics and control period and renders at the display refresh
rate. It overrides the MuJoCo timestep in memory and does not modify the source
MJCF. Headless tests can be built without the viewer using
`-DBALANCE_BUILD_GUI=OFF`.

## Performance benchmark

`rm_balance_performance` is a diagnostic benchmark rather than a CTest gate.
It independently resets and runs coarse forward/reverse and left/right yaw
sweeps, classifying stability, tracking, stopping, and actuator saturation.
Run it with an output directory for the per-case summary and 100 Hz trace:

```powershell
.\build\Release\rm_balance_performance.exe `
  .\models\MJCF\COD-2026RoboMaster-Balance.xml `
  .\build\performance\initial
```

The generated `summary.csv` and `trace.csv` stay under the ignored build tree.
The benchmark challenges `+/-1, 2, 2.5, 3 m/s` translation and
`+/-pi, 2*pi, 3*pi, 4*pi rad/s` in-place yaw. These targets map the current
controller boundary; reaching the configured maximum is not a build-pass
requirement. See `docs/notes/performance-baseline.md` for the first measured
baseline and its acceptance criteria.

To separate acceleration capability from top speed, run the dedicated
`+/-2 m/s` sweep at `0.5, 1, 2, 3, 5 m/s^2`:

```powershell
.\build\Release\rm_balance_performance.exe `
  .\models\MJCF\COD-2026RoboMaster-Balance.xml `
  .\build\performance\forward-acceleration `
  --suite forward-acceleration
```

The matching in-place yaw sweep fixes the target at `+/-2*pi rad/s` and tests
`1, 2, 3, 5, 7.5, 10, 15 rad/s^2`:

```powershell
.\build\Release\rm_balance_performance.exe `
  .\models\MJCF\COD-2026RoboMaster-Balance.xml `
  .\build\performance\yaw-acceleration `
  --suite yaw-acceleration --leg-length 0.18
```

Generate the fixed-length, unsaturated `A/B + K` prior for the same positive
yaw cases with the LQR Python environment:

```bash
python tools/lqr/yaw_response.py \
  --output build/performance/yaw-prior \
  --leg-length 0.18
```

This writes an ideal-model `summary.csv` and 1 kHz `trace.csv`. Passing one or
more benchmark traces with `--mujoco-trace` also compares perturbations around
the measured pre-ramp standing trim. It does not run MuJoCo or change the
generated controller schedule.

To identify the common leg-angle reference without allowing the position
channel to hide steady drift, run the dedicated trim scan:

```powershell
.\build\Release\rm_balance_trim_scan.exe `
  .\models\MJCF\COD-2026RoboMaster-Balance.xml `
  .\build\performance\leg-trim `
  --leg-length 0.18
```

The default scan covers `-5` through `+15 deg` in `1 deg` steps. Each case uses
the normal free-drop and balance-engagement path, sets the LQR `S` error to
zero while retaining `DS`, and writes `summary.csv` plus a 100 Hz `trace.csv`.
The controller defaults use the calibrated `+5.5 deg` trim at the `0.18 m`
baseline leg length. The scanner overrides that value for each case; use the
range and step options shown by the executable to refine another interval.

This suite always runs the complete timed schedule. Contact, attitude, and
state-machine problems are annotated in the console and CSV instead of ending
the case early; only non-finite simulation data stops a run. The output also
records virtual-leg common/differential angles, vertical leg projection,
initial position error, base height, and MuJoCo base velocity alongside
wheel-odometry velocity. `finite`, `tracked`, and `settled` are direct
diagnostic columns, not gates that control execution; the tool deliberately
does not synthesize a single pass or stability label.

For a diagnostic comparison without changing the controller default, append
`--leg-length <metres>`. For example:

```powershell
.\build\Release\rm_balance_performance.exe `
  .\models\MJCF\COD-2026RoboMaster-Balance.xml `
  .\build\performance\leg-0p20 `
  --suite forward-acceleration --leg-length 0.20
```

`leg_length_valid` continues to show whether the measured length stayed in the
original `0.13-0.20 m` observation band. Chassis contact or leaving that band
is reported independently and is not treated as controller divergence.

The GUI can replay any one of the exact same benchmark cases:

```powershell
.\build\Release\rm_balance_sim.exe `
  .\models\MJCF\COD-2026RoboMaster-Balance.xml `
  --case forward_pos_2
```

Valid names are `forward_pos_1` through `forward_pos_3`, their `forward_neg_*`
counterparts, and `yaw_pos_1pi` through `yaw_pos_4pi` with matching
`yaw_neg_*` cases. Acceleration cases use names such as
`forward_pos_2_a0p5`, `forward_neg_2_a2`, `yaw_pos_2pi_a3`, and
`yaw_neg_2pi_a7p5`. The camera follows the chassis. Playback notes problems in
the title and continues to the scheduled end; press `R` to replay the case and
use Space to pause or resume while it is running.
