# LQR 参数工具

这些脚本用于替代 `references/matlab_scripts/lqr.m` 对 MATLAB 的依赖，需要
Python、NumPy、SciPy、SymPy 和 MuJoCo Python 包。仓库使用的 `sim` Conda
环境已经包含这些依赖；脚本只使用跨平台的 Python 和 MuJoCo API。

## 验证移植结果

修改动力学方程或数值计算代码后，应首先运行旧实车参数 golden test：

```bash
conda run -n sim python tools/lqr/verify_lqr.py
```

该测试使用 `lqr.m` 的原始参数重新生成 3 ms 增益调度，并逐项对比实车固件
中的系数。允许的最大绝对误差为 `1e-6`。

## 生成当前模型参数

```bash
conda run -n sim python tools/lqr/build_current_model.py
```

该命令会先运行旧实车参数验证，然后：

1. 在可达腿长范围内分别求解两侧 MuJoCo 闭链；
2. 聚合每侧实际腿部刚体，明确排除轮子；
3. 在等效硬杆参数中保留偏置质心造成的惯量；
4. 生成并密集验证 1 ms LQR 增益调度；
5. 输出 JSON 参数记录、C 系数表和验证报告。

以下生成结果随仓库提交，便于审查模型或生成器变化造成的参数差异：

- `tools/lqr/generated/current_model_schedule.json`
- `tools/lqr/generated/current_model_schedule.h`
- `docs/notes/lqr-validation.md`

当前调度保留旧实车的 Q/R 权重，以便单独观察物理模型变化。它针对 1 ms
仿真控制周期生成，不能与现有 3 ms 实车调度混用。

需要诊断偏航惯量匹配时，可以把整机 Izz 候选生成到构建目录，避免覆盖正式
参数和报告：

```bash
conda run -n sim python tools/lqr/build_current_model.py \
  --reuse-model-parameters tools/lqr/generated/current_model_schedule.json \
  --yaw-inertia-source assembly \
  --output build/lqr-assembly \
  --report build/lqr-assembly/report.md
```

随后可用 `BALANCE_LQR_SCHEDULE_DIR` 指向该目录建立独立构建。该选项只用于
控制器与 plant 的 A/B 诊断，不会改变默认的 `base-link` 正式来源。

## 参数约定

- 物理左侧映射到 XML `Right_*`，物理右侧映射到 XML `Left_*`，与仿真
  adapter 保持一致。
- `body_yaw_inertia_actual` 是转换到机体坐标后的 `base_link` Izz。
- `body_yaw_inertia_model = body_yaw_inertia_actual * scale`，当前选定的
  `scale` 明确记录为 `1.0`。
- `--yaw-inertia-source assembly` 会自动把 scale 设置为整机诊断 Izz 与
  `base_link` Izz 的比值，并在 JSON 和报告中标明来源。
- 两倍偏航惯量只用于敏感性对照，不作为正式参数。
- 状态顺序为 `[s, ds, psi, dpsi, theta_l, dtheta_l, theta_r, dtheta_r,
  theta_b, dtheta_b]`。
- 输入顺序为 `[T_wheel_l, T_wheel_r, Tp_leg_l, Tp_leg_r]`。
