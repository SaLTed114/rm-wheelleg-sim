# rm-balance-sim 项目上下文

> 本文是给后续 Codex/开发会话快速恢复上下文用的工作笔记，不是对外设计文档。
> 最近更新：2026-07-30。

## 首要约束

- `references/matlab_scripts/lqr.m` 和 `leg_fit.m` 中的质量、几何、质心、惯量及 LQR 权重是旧控制器/参考脚本参数，不是当前 `models` 仿真机器人的实际参数。
- 不得把这些数值直接移植到仿真模型或新控制器；需要依据当前机器人模型重新确认参数、重新线性化并重新生成控制器。
- `/home/l/SaLT/wheelleg` 是已经放弃继续修补的历史仿真项目，只用于提取经验和测试方法。不要把它的旧 MJCF、模型参数、关节标定、LQR 表或仿真补偿直接搬进本项目。
- 运动学等效不等于动力学等效。控制层可以使用虚拟腿坐标，但 plant 必须保留实际闭链连杆的偏置质心和惯量。
- 用户当前希望先建立架构和构型认知。除非明确要求，不要主动转入“为什么实车没调好”的代码审查或调参诊断。

## 项目目标

建立轮腿平衡机器人的仿真，并将用户已有、上过实车的控制架构接入高保真机器人 plant。优先保留控制软件的分层边界，而不是逐行照搬 STM32/HAL/CAN 代码。

预期总体数据流：

```text
MuJoCo/Isaac Sim 实际闭链 plant
    -> 仿真传感器适配
    -> 虚拟腿运动学与状态估计
    -> 行为状态机和参考量
    -> 增益调度 LQR + 腿长/横滚控制
    -> 虚拟力到关节力矩映射
    -> 仿真执行器适配
    -> plant
```

## 仓库材料

### 当前机器人模型

- `models/MJCF/COD-2026RoboMaster-Balance.xml`
- `models/MJCF/*.STL`
- `references/COD-2026RoboMaster-Balance.usd`：上游 Isaac Sim 模型，仅作参考，不随主仓库提交
- `models/Pictures/`：MuJoCo、Isaac Sim 加载截图
- `references/COD-2026RoboMaster-Balance-Simulation_File.zip`：原始打包文件，不随主仓库提交
- `models/README.md`：来源说明（辽宁科技大学 COD 战队，串联腿闭链模型）
- `models/` 原为独立上游仓库，现以普通文件形式纳入本项目，不使用 submodule；引入版本为 `089e35a97e4be832f293547d283eb6f62a22185f`。
- 上游仓库在引入时没有明确的 LICENSE/NOTICE；来源、原团队、提交作者及不重新许可声明记录在 `models/README.md`。

### 控制代码与参考材料

- `references/rm2026cb-balance-chassis/`：用户自己的实车控制工程，是控制软件架构的主要参考。
- `references/matlab_scripts/`：旧运动学、惯量拟合和 LQR 生成脚本；只参考方法，参数需要重建。
- `references/中石油北京轮腿电控代码/`：其他学校参考工程。
- `references/南航金城轮腿电控代码/`：其他学校参考工程。
- `references/平衡步兵..pdf`：轮腿平衡资料。
- `references/连杆示意/示意图.svg`：用户补充的实际连杆拓扑示意；角度和比例不代表真实值。

## 历史仿真项目 `/home/l/SaLT/wheelleg`

### 定位

这是用户此前尝试的完整轮腿仿真项目。用户认为该项目已经积重难返，因此创建本仓库重新开始。后续把它视为只读的历史实现和踩坑记录，不以它为新项目代码基线。

历史项目已经做过：

- `control_core/`：从 STM32 工程剥离出的硬件无关 C 控制核心；
- `sim/`：MuJoCo C API 仿真器、GLFW 可视化、平台适配和 CSV 日志；
- `tools/lqr/`：MATLAB-free 的 Python LQR 生成器和 golden test；
- 三层行为状态机、FOLLOW/SPIN、故障闭锁、腾空/落地相关实验；
- 大量 headless 复现模式和运行日志。

历史仓库当前状态（检查于 2026-07-30）：

- `HEAD=7d9d8f9`，比 `origin/main=d660063` 多一个本地提交；
- `bc_control.c`、`bc_estimator.c`、`model_params.h`、`core_types.h` 有未提交修改；
- 未提交内容正在继续尝试“落地吸能/腿长环软化”；
- 有未跟踪的 `MUJOCO_LOG.TXT`；
- 不要修改、清理或重置这个历史仓库。

### 历史项目的有效架构经验

以下设计可以参考思想，但应在新项目中重新、小步实现：

1. **纯 SI 控制边界**：`core_input_t -> core_step() -> core_output_t`。平台适配器负责所有关节方向、零位、单位和 actuator 顺序，控制核心不接触 MuJoCo 或硬件类型。
2. **显式平台适配层**：`sim_adapter_read/write` 把仿真状态转换为统一的腿、轮和 IMU 数据，再把六路力矩写回仿真。
3. **控制周期与物理步长分离**：MuJoCo 曾用 1 ms 物理步长，控制器按独立分频运行。
4. **诊断接口与操作接口分离**：正常 `core_command_t` 不承载 bring-up 钩子，单独使用 `core_debug_t`。
5. **无界面回归测试**：为平衡、跟随、转向、失能重接管、腾空落地等场景输出 CSV 和明确的成功/失败指标。
6. **LQR 生成 golden test**：Python 生成器先复现旧 MATLAB/固件结果，再用于新模型，避免数值移植错误和物理参数错误混在一起。
7. **模型名字到索引的集中缓存**：启动时解析关节、actuator、sensor，运行循环只使用缓存索引。

### 不应从历史项目继承的内容

- `sim/assets/MJCF/`：历史 README 明确称其为 approximate、not-the-real-robot model；当前项目已有新的 `models/` 资产。
- `control_core/model/model_params.h` 和旧 `bc_lqr_schedule.c`：均针对历史模型或旧实车参数。
- `sim/platform_sim/sim_calib.h`：其中关节符号、偏置和输出符号是为旧 MJCF 反复拟合得到的模型专用标定。
- 历史仿真中用于抵消模型偏置的常量，例如 FOLLOW 的速度偏置、pitch trim、落地软化参数等。
- 历史行为状态机和腾空/落地实现的具体特例；只能把场景和失败现象作为回归用例。
- `build/`、`logs/`、CSV、截图及本地依赖发现逻辑等生成物。

### 为什么不继续修补

历史项目并非完全失败：提交记录表明它曾达到稳定平衡，并逐步实现 FOLLOW、SPIN、重接管和腾空落地实验。但复杂度是在旧近似 plant 上逐层增长的：

- 模型不是目标真实机构，却同时承担控制器建模、符号标定和行为调试；
- 为闭环现象逐渐加入多组模型专用偏置和补偿，难以区分结构性控制需求与旧模型误差；
- `sim_run.c`、`sim_diag.c`、`sim_conv.c` 累积了大量模式和特殊路径；
- 控制状态、接触估计、行为恢复和仿真测试互相推动演化，修改影响面越来越大；
- 文档已经有实现状态不一致：旧行为设计文档称 AIRBORNE 为不可达占位，而仓库说明和后续提交已经描述/实现了腾空域；
- 当前仍停在落地冲击、速度相关振荡和方向相关几何问题的连续修补中。

新项目应从“可信 plant + 最小闭环 + 可测契约”重新建立，每一步只加入一个误差来源或行为能力。

## 机器人构型：已确认事实

### 运动学抽象

每侧腿在运动学上可以等效成以髋关节和轮轴为对顶点的筝形四连杆：

- 髋关节顶点相邻的两条等效边长度相等；
- 轮轴顶点相邻的两条等效边长度相等；
- 两组边的长度不相等；
- 两个主动髋关节角为 `phi1`、`phi2`；
- 虚拟腿坐标为长度 `L` 和相对车体的腿角 `theta`。

旧控制代码采用的运动学形式为：

```text
delta = (phi1 - phi2) / 2
theta = (phi1 + phi2) / 2
L = l1*cos(delta) + sqrt(l2^2 - l1^2*sin(delta)^2)
```

虚拟力与关节力矩通过虚功/雅可比转置关联：

```text
[tau1, tau2]^T = J(phi1, phi2)^T [F, Tp]^T
```

其中 `F` 是虚拟腿轴向力，`Tp` 是虚拟腿摆动力矩。

### 实际机构

实际机构不是一副质量沿虚拟腿轴对称分布的筝形四连杆。它使用折叠的多连杆闭链和平行四边形传动实现相同的轮轴运动。

关键设计特征：

- 实际连杆质心不在髋关节到轮轴的虚拟腿轴线上；
- 惯量分布具有刻意设计的偏置；
- 运动学可等效，但重力项、惯量矩阵、科氏/离心项不能直接等效；
- 这是构型的特意设计点，不应在仿真中被对称化或简化掉；
- `示意图.svg` 只表达拓扑、闭环和平行四边形传动关系，图中角度和比例均为示意。

因此必须区分：

```text
虚拟腿 = 控制坐标
实际多连杆闭链 = 动力学 plant
```

## 当前 MJCF 模型认知

`models/MJCF/COD-2026RoboMaster-Balance.xml` 显式建立了左右两侧实际刚体，而非仅用等效虚拟杆：

```text
前支链：front -> front_child1 -> front_child2 -> front_child3
后支链：rear  -> rear_child1  -> wheel
```

每侧通过两个 equality `connect` 闭合支链：

- `front_site1 <-> rear_site1`
- `front_site2 <-> rear_site2`

模型中的每段刚体都有独立的：

- `mass`
- `inertial pos`（局部质心）
- 惯量主轴四元数 `quat`
- `diaginertia`
- STL 外观/几何网格

主动输入共六个：左右前髋、左右后髋和左右轮轴。其余连杆关节是闭链中的被动关节。

当前 MJCF 文件观察到的状态：

- `base_link` 作为 `worldbody` 下的 geom，尚未放入带 `freejoint` 的浮动基座 body；
- 文件中尚无地面、轮地接触场景、传感器定义；
- 因而它目前更接近“高保真闭链机构模型”，还需要包装成完整平衡仿真场景；
- 上游 USD 文件已移至 `references/`，当前项目不把 Isaac Sim 作为运行后端。

## 用户实车控制架构

### 线程与主循环

入口：`references/rm2026cb-balance-chassis/User/Src/user_main.c`

`User_Init()` 创建独立的平衡底盘线程和通信线程。底盘线程位于 `Tasks/balance_chassis.c`，名义周期为 3 ms：

```c
while (1) {
    balance_chassis_update();
    balance_chassis_set_command();
    balance_chassis_execute();
    delay(DELAY_TS);
}
```

用户重视这三个阶段的隔离：

1. `update`：读取轮、关节、IMU，计算腿运动学并更新估计状态；
2. `set_command`：行为状态机、参考量、LQR、轮和腿控制；
3. `execute`：关节力矩与轮电流下发。

仿真中应保留这套边界，用仿真传感器/执行器适配器替换硬件层。

### 聚合状态

`balance_chassis_t` 聚合：

- `info`：硬件和机械配置；
- `status`：轮、腿、车体、估计状态和接触状态；
- `command`：LQR、轮、腿和上层命令；
- `filter` / `est`：滤波与估计内部状态；
- `behavior`：行为状态机。

十维控制状态：

```text
x = [
    s, ds,
    psi, dpsi,
    theta_l, dtheta_l,
    theta_r, dtheta_r,
    theta_b, dtheta_b
]
```

其中腿运动学输出的 `leg.theta` 是相对车体角度，LQR 腿角状态还会叠加车体俯仰和 `pi/2`，转换为相对地面的腿角。

### 行为层

控制模式：

- `DISABLED`
- `BRAKE`
- `POSTURE`
- `BALANCE`
- `STEPUP`

行为层生成位置、速度、偏航和腿部参考，并选择以下控制策略：

- 轮：idle / brake PID / LQR；
- 腿长：idle / target PVI / target PVI + 支撑补偿；
- 腿角：idle / target PVI / LQR。

### 控制层

平衡 LQR 是统一的 `4 x 10` 状态反馈，不是左右两套完全独立的二维控制器。

输入：

```text
u = [T_wheel_l, T_wheel_r, Tp_leg_l, Tp_leg_r]
```

功能分工：

- 轮力矩与虚拟腿摆动力矩由 LQR 产生；
- 腿长轴向力 `F` 由独立腿长 PVI 产生；
- Roll 通过左右腿轴向力差控制；
- 重力支撑作为轴向力前馈叠加；
- 起身、收腿、越障使用行为层切换后的腿长/腿角控制；
- 虚拟腿 `F/Tp` 最后通过 `J^T` 映射为四个真实髋关节力矩。

### 旧 LQR 生成方法（方法可参考，参数不可复用）

`references/matlab_scripts/lqr.m` 的方法：

1. 将实际腿降阶为构型相关的等效硬杆；
2. 在固定腿长下建立左右轮、左右虚拟腿和车体的小角度线性模型；
3. 腿长 `L` 不作为动态状态，而作为调度参数；
4. 在 `L` 范围内离散采样，生成连续 `A(L), B(L)`；
5. 以 3 ms ZOH 离散化并求 `dlqr`；
6. 对 `K(L)` 的每个元素做三次多项式拟合；
7. 嵌入式端用平均腿长和 Horner 法在线计算增益。

注意：脚本中的 `Lmin/Lmax`、质量、质心比例、惯量范围、`Q/R`、采样周期等都只是旧版本参考值。新模型必须重新确定。

## 其他两套参考代码的主循环

只作为架构对照，不作为当前实现目标。

### 中石油北京

`code/chassis/Task/chassis_task.c`：

```text
模式选择 -> 反馈/运动学/估计 -> 行为目标 -> 大型控制状态机
         -> 主循环内安全检查和电机发送
```

底盘线程约 1 ms，顶层已有分段，但控制、异常处理和发送耦合较多。

### 南航金城

`Chassis/Task/Src/Chassis_Task.c`：

```text
采集 -> 打滑/角度检测 -> 模式目标 -> 状态融合 -> 特殊行为
     -> 腿控制 -> LQR -> VMC -> CAN 发送
```

也是约 1 ms 的独立底盘线程，流水线清楚，但具体业务步骤直接平铺在顶层。

用户自己的 `update -> set_command -> execute` 抽象层级更高，适合作为仿真控制器接口。

## 当前最小运行骨架

第一版工程骨架已经建立，目的是先贯通接口，不代表最终控制器或 plant：

```text
MuJoCo plant
    -> C++ MujocoAdapter::read
    -> C balance_control_core_step
    -> C++ MujocoAdapter::write
    -> MuJoCo step
```

- control core 使用 C11，simulation core 使用 C++17，通过 `include/balance/control_core.h` 中的纯 C 数据结构连接；
- control core 不暴露六路扁平 actuator 顺序；observation 按左右侧拆分为腿部前/后关节反馈与轮反馈，actuation 对应拆分为腿关节力矩与轮力矩；
- MuJoCo 中六个关节和 actuator 的名称、索引及排列只由 C++ adapter 管理；
- 当前 observation 只包含腿关节和轮的角度、角速度，operator command 只有 `enabled`；
- 空控制器每拍明确清零六路力矩，不包含估计、行为、运动学或控制算法；
- C++ 侧分为 `MujocoPlant`、`MujocoAdapter`、`SimulationRunner` 和 `MujocoViewer`；正常入口实时无限运行到用户关闭 GUI，`run_for()` 只供 headless 测试使用；
- 物理和控制周期暂定均为 1 ms。加载后只在内存中覆盖 `mjModel.opt.timestep`，不修改原始 MJCF；
- 当前仍直接加载固定基座、无地面、无传感器的原始模型；自由基座和完整场景属于下一阶段；
- CMake 支持用 `MUJOCO_ROOT` 指向 Linux Python wheel 或官方 MuJoCo 包，并为 Windows MSVC 官方包复制运行时 DLL；
- 本机官方 MuJoCo 3.9.0 SDK 安装在 `/home/l/.local/opt/mujoco-3.9.0`，当前 build 和 VS Code compilation database 已切换到该 SDK，不再依赖 Anaconda wheel；
- 本机已用 MuJoCo 3.9.0 验证 C 单元测试、模型接线 smoke test 和 1 秒/1000 tick 的零力矩运行。

## 后续需要确认/实现

1. 以 MJCF/MuJoCo 为当前仿真后端；USD 只保留为外部参考资产。
2. 核对当前模型每段质量、质心、惯量是否为目标参数；不要用旧 MATLAB 脚本覆盖。
3. 为 base 添加浮动自由度、总成质量/惯量，并建立地面与轮地接触。
4. 确认模型关节与实车 `JOINT_LF/LB/RF/RB` 的一一映射、正方向和零位。
5. 定义仿真状态到 `balance_chassis_t` 传感器数据的适配接口。
6. 定义关节力矩、轮力矩到仿真 actuator 的接口及限幅。
7. 依据当前模型重新提取/拟合等效腿质心和惯量，并重新生成 LQR 增益调度。
8. 单独参数化执行器、摩擦、结构柔性、传感器和接触误差，避免不同 sim-to-real gap 混杂。
9. 优先先跑通“原控制结构 + 高保真 plant”，再决定是否修改控制器设计。

## 快速恢复时优先阅读

1. 本文件。
2. `references/连杆示意/示意图.svg`。
3. `models/MJCF/COD-2026RoboMaster-Balance.xml`。
4. `references/rm2026cb-balance-chassis/Tasks/balance_chassis.c`。
5. `references/rm2026cb-balance-chassis/Tasks/balance_chassis/bc_control.c`。
6. `references/rm2026cb-balance-chassis/Tasks/balance_chassis/bc_behavior2.c`。
7. `references/rm2026cb-balance-chassis/Tasks/balance_chassis/bc_estimator.c`。
8. `references/matlab_scripts/lqr.m`（仅参考生成方法）。
