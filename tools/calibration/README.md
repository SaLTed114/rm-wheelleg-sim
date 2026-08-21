# MuJoCo leg-adapter calibration

`calibrate_leg_adapter.py` fits the four per-side joint offsets used by the
MuJoCo adapter while keeping the shared `0.175/0.208 m` virtual-leg geometry
and joint signs fixed.

The fit uses exact closed-chain poses from `0.18 m` through `0.21 m`. A second
`0.186-0.39 m` scan is diagnostic only. It does not claim full-range
kinematic equivalence.

Run with the repository's `sim` environment:

```powershell
conda run -n sim python tools/calibration/calibrate_leg_adapter.py --check
conda run -n sim python tools/calibration/calibrate_leg_adapter.py --write
```

`--write` updates:

- `generated/leg_adapter_calibration.json`, containing provenance, samples,
  metrics, limits, and fitted values;
- `src/sim/generated/mujoco_leg_calibration.hpp`, consumed by the adapter and
  MuJoCo regression test.

迁移前的辽科 COD adapter 参数保留在 `src/sim/generated/ustl/mujoco_leg_calibration.hpp`，只供旧 COD 模型的 C 回归案例使用；默认生成命令不覆盖该文件。
