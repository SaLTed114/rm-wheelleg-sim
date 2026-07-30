# rm-balance-sim

Minimal MuJoCo host for the wheel-legged balance robot. The runtime is split into
a C11 control core and a C++17 simulation layer. The initial controller is an
intentional zero-torque stub.

The robot assets under `models/` are imported from the open-source model released
by the Liaoning University of Science and Technology COD RoboMaster team. See
`models/README.md` for upstream attribution, revision, and licensing notes.

## Linux build with the official MuJoCo SDK

Download and extract the official Linux release, then point `MUJOCO_ROOT` at
the directory containing `include` and `lib`. The current local installation is
`/home/l/.local/opt/mujoco-3.9.0`:

```bash
cmake -S . -B build \
  -DMUJOCO_ROOT=/home/l/.local/opt/mujoco-3.9.0
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

## Windows build with Visual Studio

Download and extract the official MuJoCo Windows release, then configure with
the Visual Studio generator. `MUJOCO_ROOT` must contain `include`, `lib`, and
`bin`; CMake copies `mujoco.dll` beside each executable.

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  -DMUJOCO_ROOT=C:\libs\mujoco-3.x.x-windows-x86_64
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
.\build\Release\rm_balance_sim.exe `
  models\MJCF\COD-2026RoboMaster-Balance.xml
```

The GUI uses a 1 ms physics and control period and renders at the display refresh
rate. It overrides the MuJoCo timestep in memory and does not modify the source
MJCF. Headless tests can be built without the viewer using
`-DBALANCE_BUILD_GUI=OFF`.
