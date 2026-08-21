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
- `docs/archive/validation/lqr-validation.md`

`tools/lqr/generated/ustl/` 保留辽科 COD 参数，只供旧 COD 模型的 C 回归案例使用。本分支的 C++20 控制核心是死代码，不读取该参数目录，也不做复旦适配。

当前调度针对 1 ms 仿真控制周期生成，不能与现有 3 ms 实车调度混用。
左右腿角代价使用共同/差分坐标：共同腿角保持基础权重，差分腿角由
`--leg-angle-difference-weight` 单独设置；腿角速度也通过
`--leg-angular-velocity-difference-weight` 独立设置。这样可以加强原地旋转
时的腿姿态约束，而不同时限制直线加速所需的共同摆腿。

需要批量生成候选、建立隔离构建并运行 MuJoCo/A-B 对照时，使用
`tools/experiments/run_experiment.py`；配置和产物说明见
`tools/experiments/README.md`。`forward_response.py` 由该 runner 调用，要求
输入关闭 `S` 位置反馈的 performance trace，并围绕运动前稳态比较扰动响应。

## 参数约定

- 物理左侧映射到 XML `Right_*`，物理右侧映射到 XML `Left_*`，与仿真
  adapter 保持一致。
- `body_yaw_inertia_actual` 是参数提取器得到的 `base_link` Izz；它不是 LQR
  方程最终使用的整机惯量。
- 提取器在模型名义腿长姿态下，将 base、腿和轮的自身惯量及平行轴项聚合为
  `assembly_yaw_inertia_reference`；复旦默认名义腿长为 `0.24 m`，COD 参数
  扫描保留原 `0.34 m` nominal 姿态，两者的 yaw 惯量参考腿长均为 `0.18 m`。
- `assembly_yaw_inertia_diagnostic` 仍记录提取器 nominal 姿态下的整机 Izz，
  用于观察腿长变化，但不作为正式输入。
- 方程中的 `I_z/(2R_l)` 来自左右差分坐标变换；`I_z` 本身是完整惯量，
  不提供一倍/两倍或 base-link/assembly 来源开关。
- 状态顺序为 `[s, ds, psi, dpsi, theta_l, dtheta_l, theta_r, dtheta_r,
  theta_b, dtheta_b]`。
- 输入顺序为 `[T_wheel_l, T_wheel_r, Tp_leg_l, Tp_leg_r]`。
- `q_diagonal` 保留可直接传回生成器的基础状态权重，`q_matrix` 记录实际
  状态代价矩阵；`leg_angle_cost` 和
  `leg_angular_velocity_cost` 另外记录共同模式、差分模式和转换到左右腿
  坐标后的交叉项。
