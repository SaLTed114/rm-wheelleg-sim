# MuJoCo leg-adapter calibration

`calibrate_leg_adapter.py` fits the four per-side joint offsets used by the
MuJoCo adapter while keeping the shared `0.215/0.254 m` virtual-leg geometry
and joint signs fixed.

The fit uses exact closed-chain poses from `0.18 m` through `0.21 m`. A second
`0.186–0.39 m` scan is diagnostic only and must not regress relative to the
legacy offsets. It does not claim full-range kinematic equivalence.

Run with the repository's `armsim` environment:

```powershell
conda run -n armsim python tools/calibration/calibrate_leg_adapter.py --check
conda run -n armsim python tools/calibration/calibrate_leg_adapter.py --write
```

`--write` updates:

- `generated/leg_adapter_calibration.json`, containing provenance, samples,
  metrics, limits, and fitted values;
- `src/sim/generated/mujoco_leg_calibration.hpp`, consumed by the adapter and
  MuJoCo regression test.
