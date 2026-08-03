# rm-balance-sim 项目上下文

> 本文是给后续 Codex/开发会话快速恢复上下文用的工作笔记，不是对外设计文档。
> 最近更新：2026-08-03。

## 首要约束

- `references/matlab_scripts/lqr.m` 是用户生成实车 LQR 参数的正式脚本；其中与 `leg_fit.m` 相关的质量、几何、质心、惯量及 LQR 权重仍是旧机器人参数，不是当前 `models` 仿真机器人的实际参数。
- 不得把这些数值直接移植到仿真模型或新控制器；需要依据当前机器人模型重新确认参数、重新线性化并重新生成控制器。
- `/home/l/SaLT/wheelleg` 是已经放弃继续修补的历史仿真项目，只用于提取经验和测试方法。不要把它的旧 MJCF、模型参数、关节标定、LQR 表或仿真补偿直接搬进本项目。
- 运动学等效不等于动力学等效。控制层可以使用虚拟腿坐标，但 plant 必须保留实际闭链连杆的偏置质心和惯量。
- 用户当前希望先建立架构和构型认知。除非明确要求，不要主动转入“为什么实车没调好”的代码审查或调参诊断。

## 已验证：LQR 偏航惯量与生成链

- Python 生成器以旧参数复现当前实车 `bc_lqr_schedule.c`；Windows 自动报告与 Linux 复跑的最大系数误差分别为 `4.962484e-10` 和 `4.974874e-10`，都远低于 `1e-6` golden 阈值，确认该固件表就是 `lqr.m` 当前参数的生成结果。两端重新生成的当前模型 C header 完全一致；
- `equ5` 中全部 `I_z/(2*R_l)` 项结合 `equ7` 后严格化简为 `-I_z*D2psi`；参考 PDF 的原始方程也把 `I` 定义为机体绕竖直轴的完整惯量。式中不存在尚未处理的 `1/2`。
- 当前模型正式采用 `I_z_model_scale=1`，即 `I_z_model=I_z_body_actual=3.047199970 kg*m^2`。两倍值 `6.094399941 kg*m^2` 只作为敏感性对照，不能在其他位置再次乘二。
- 当前模型两种惯量生成的原始增益最大差为 `1.484358039`。完整参数、误差、物理依据及 1 ms 调度验证见 `docs/notes/lqr-validation.md`。
- 工具仍须显式区分 `body_yaw_inertia_actual`、`body_yaw_inertia_scale` 和 `body_yaw_inertia_model`，避免以后重新引入歧义。

## 项目目标

建立轮腿平衡机器人的仿真，并将用户已有、上过实车的控制架构接入高保真机器人 plant。优先保留控制软件的分层边界，而不是逐行照搬 STM32/HAL/CAN 代码。

预期总体数据流：

```text
MuJoCo 实际闭链 plant
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
- `references/matlab_scripts/`：实车使用过的运动学、惯量拟合和 LQR 生成脚本；生成流程可作为正式迁移基线，但物理参数需要重建。
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

- `base_link` 已成为带 `freejoint` 的动态刚体；上游 USD 提供的机体参数为质量 `11 kg`、质心 `(-0.019917, -0.00040396, 0.021412) m`、主惯量 `(2.8640678, 2.8736324, 3.0472) kg*m^2` 和对应主轴四元数；
- 这组机体惯量相对整车尺寸显得偏大，当前只作为上游模型值如实使用，后续仍需按实车复核；
- mocap `base_support` 与 weld 仍保留，但模型默认关闭该约束；测试或后续“托住—起控—释放”流程可显式启用；
- 已加入 `z=-0.43 m` 的地面，机体、腿部连杆和轮均与地面碰撞，但机器人内部自碰撞保持关闭；
- 已加入机体姿态、gyro、左右虚拟髋点、轮轴点和 `framepos` sensor；
- 物理右腿闭链连接点存在约 `1.95 mm` 固定横向错位；参数提取器按每侧
  固有闭合误差再加 `0.5 mm` 判断姿态可行性，不能把它误判为求解失败或
  悄悄把模型对称化；
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

仿真中保留这套职责边界，并把实车 `set_command/execute` 之间混合的计算与
下发进一步拆成独立 `calculate/execute`；这属于本项目接口演化，不是实车
原代码已有的第四阶段。

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

当前第一版观测器不做滤波和 IMU/编码器融合：`ds` 直接来自左右轮速平均，`s` 对 `ds` 按控制周期积分，yaw 在观测器复位时建立自然零点。轮角保留为诊断量，不直接组装 `s`。坐标约定为 FLU：车头 `+X`、左侧 `+Y`、上方 `+Z`，正 pitch 为车头下压，正 yaw 为左转。上游 XML 的 Left/Right 名称与物理位置相反，因此 adapter 使用 `BC_L -> XML Right_*`、`BC_R -> XML Left_*`，不修改上游节点名称。

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

### 实车 LQR 生成方法（结构沿用，参数不可复用）

`references/matlab_scripts/lqr.m` 的方法：

1. 将实际腿降阶为构型相关的等效硬杆；
2. 在固定腿长下建立左右轮、左右虚拟腿和车体的小角度线性模型；
3. 腿长 `L` 不作为动态状态，而作为调度参数；
4. 在 `L` 范围内离散采样，生成连续 `A(L), B(L)`；
5. 以 3 ms ZOH 离散化并求 `dlqr`；
6. 对 `K(L)` 的每个元素做三次多项式拟合；
7. 嵌入式端用平均腿长和 Horner 法在线计算增益。

注意：脚本中的 `Lmin/Lmax`、质量、质心比例、惯量范围、`Q/R`、采样周期等都只是旧版本参考值。新模型必须重新确定。

当前实车工程的 `bc_lqr_schedule.c` 已由 golden test 确认为该脚本的直接生成结果。无 MATLAB 的迁移位于 `tools/lqr/`：SymPy 建立符号方程和 Jacobian、SciPy 完成 ZOH 与离散 Riccati 求解、NumPy 完成多项式拟合，并强制先通过固件系数 golden test，再换入新模型参数。

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

本项目在用户实车三阶段边界基础上形成的
`update -> set_command -> calculate -> execute` 适合作为仿真控制器接口。

## 当前最小运行骨架

第一版工程骨架已经建立，目的是先贯通接口，不代表最终控制器或 plant：

```text
MuJoCo plant
    -> C++ MujocoAdapter::read
    -> C bc_controller_update
    -> C bc_controller_set_command（每个控制周期）
    -> C bc_controller_calculate
    -> C bc_controller_execute
    -> C bc_controller_capture_snapshot
    -> C++ MujocoAdapter::write
    -> MuJoCo step
```

- control core 使用 C11，simulation core 使用 C++17，通过 `include/balance/types.h` 中的纯 C 数据结构连接；
- 正式控制接口收口到 `bc_controller_t`，统一持有 state machine、control core、最新 operator command 和最近一次最终 actuation；operator command 是 `set_command` 与 `calculate` 之间待处理的输入；仿真与后续实车上层只调用 controller 的 `update -> set_command -> calculate -> execute`，不负责拼接内部模块；
- controller 的 update 更新运动学与观测状态，set_command 只保存最新 operator command，calculate 是唯一推进状态机、时间计数并生成低层控制请求的阶段，execute 只根据顶层 system 状态做硬门控、力矩限幅和最终输出；
- 顶层 `controller` 表示完整控制系统的 facade；PD 与 LQR 是由 control core 调用的具体控制律，集中放在 `control_law/` 子目录，且不依赖 controller、state machine 或 observer；
- control core 不暴露六路扁平 actuator 顺序；sensor feedback 按左右侧拆分为腿部前/后关节反馈、轮反馈和 IMU，actuation 对应拆分为腿关节力矩与轮力矩；
- 独立的 `controller_snapshot` 模块通过 `bc_controller_capture_snapshot()` 按需读取 controller 的 system/motion、observer、当前 state reference、roll/roll rate、tick count、限幅前 actuation request 和最近一次最终 actuation，生成由调用者持有的 `bc_controller_snapshot_t`；controller 不持续缓存第二份同步状态，旧 snapshot 也不会被后续控制周期改写；该结构是 GUI、测试以及未来实车遥测、日志、故障诊断和命令确认的统一观察面，不参与实时安全决策，也不是可直接发送的 UART 线格式；
- `SimulationRunner` 持有最近一次 snapshot，在构造、reset 后以及每次 execute 后刷新，只公开 `snapshot()`，不再读取或暴露 state machine/control core 内部字段及分散的 state/leg/actuation getter；未来实车应由控制线程在 execute 后捕获，再把副本交给串口或日志线程，其他线程不直接读取 controller；
- state machine 和 control core 的低层 API 继续供各自单元测试与内部调试使用，但正式仿真路径只使用 controller facade；MuJoCo 腿部定姿等低层集成验证使用测试文件内的专用 harness，不污染 runner 的生产接口；
- MuJoCo 中六个关节和 actuator 的名称、索引、排列、符号及关节零偏只由 C++ adapter 管理；左右腿进入 control core 后共用同一套坐标；左右轮 actuator 均显式限制为 `+/-6.32 N*m`，与 control core 一致；
- operator command 使用持续电平 `system_enabled` 和单周期事件 `balance_restart`：顶层 `SYSTEM_OFF` 时 control core 的 execute 无条件输出零；使能只进入 `SYSTEM_ON/MOTION_IDLE`，不会在状态机内部隐式启动恢复流程，必须由外部发送 restart；开启状态下 restart 会清除计时、reference ramp 和 LQR 参考并重新整理腿部，关闭状态下忽略 restart；
- forward velocity 与 yaw rate 先经过独立的线性 reference ramp，再分别写入 `ds_ref/dpsi_ref` 并积分为 `s_ref/psi_ref`；当前硬上限为 `3 m/s, 5 m/s^2` 与 `4*pi rad/s, 15 rad/s^2`。移动转弯建议不超过 `1.5*pi rad/s`，但随前进速度变化的耦合指令包络尚未实现；system off 仍立即门控最终 actuation，不经过斜坡停机；
- 第一版分层 C 状态机使用 `system -> motion -> task` 命名：当前已建立 `SYSTEM_OFF/ON/FAULT` 和 `MOTION_IDLE/SELF_RIGHTING/LEG_POSITIONING/BALANCE_ENGAGING/ACTIVE`；`SYSTEM_FAULT` 目前只是为后续实车电机在线检测预留的枚举占位，没有任何输入会触发它；若内部状态被置为 `FAULT`，下一次 system update 会直接回到 `OFF`、复位 motion 并保持零控制输出。真正的故障来源、锁存和恢复流程以后单独设计；restart 令 motion 进入 `SELF_RIGHTING`，当前因仿真不会完全翻倒而在同一周期默认跳过到 `LEG_POSITIONING`，稳定摆腿后进入 `BALANCE_ENGAGING`；GUI 和无界面静态测试在首次使能时从输入侧显式模拟一次 restart，尚未实现真正的自扶正动作、进入 ACTIVE 的判据及其下任务状态机；
- system 与 motion 位于 `state_machine/` 子目录：system 的转移和动作只负责 `OFF/ON`、预留的 `FAULT -> OFF` 回退及下层启动/复位，motion 使用先转移、再按转移后状态执行动作的两个 switch，不再把两级转移和平衡控制输出混在一条 if 链中；
- controller 每周期组装一份只读 `bc_state_machine_input_t`，集中提供 operator command、观测状态、双腿运动学和 timestep，system 向 motion 传递同一份输入；持久配置、状态和计时器仍属于各状态机对象，控制策略与目标仍通过独立的 `bc_control_command_t` 输出，不使用可写的大 context；
- controller config 只提供实际生效的 `control` 与 `motion` 两组配置，不在 controller 内保存副本，也不使用仅包装 motion 的空 `system` config；control core 和 motion 初始化时各自复制并持有自己的有效配置；
- 状态转移中“条件连续成立指定时长”的公共逻辑使用头文件内联工具 `bc_condition_hold_t`；条件不成立时自动清零累计时间，各状态机为每个独立条件持有自己的 hold，不把普通延时、超时、锁存或滞回混入这一原语；
- 后续平衡失败不自动恢复，而是回到 `MOTION_IDLE` 等待新的 restart；ACTIVE 下预留 `NORMAL/STAIR_CLIMB/JUMP/RAMP_JUMP` 任务层，`AIRBORNE` 是由接触观测派生、可供跳跃和飞坡共用的状态，不能仅因轮子离地就判定平衡失败；
- 当前不建立通用 scenario 框架；性能 benchmark 与 GUI 单案例回放通过窄接口 `PerformanceScenario` 共用 case spec、固定阶段时间和 operator command，并用相同诊断函数标记问题，从而保证肉眼观察与 CSV 使用同一案例。原有 GUI 循环演示和其他 headless 测试仍使用各自局部输入函数，等出现扰动、回放等更多跨入口需求后再扩展场景抽象；
- control command 不再使用粗粒度的 `POSTURE/BALANCE` 模式，而是由状态机为左右腿自动生成独立的腿长、腿角策略及目标，并生成车轮策略；operator command 只提供系统使能、平衡重启和运动意图，不能从顶层指定低层控制策略；
- control core 使用 `l1=0.215 m`、`l2=0.254 m` 计算虚拟腿运动学、解析雅可比和 `J*qdot`；`LEG_POSITIONING` 选择腿长/腿角位置 PD 和车轮禁用，`ENGAGING` 选择腿长位置 PD + 每腿 `54 N` 支撑前馈、腿角 LQR 和车轮 LQR，最后通过 `J^T` 输出关节力矩；
- LQR 按左右平均腿长调用三次增益调度，误差定义为 `reference - state`；轮力矩限制为实车电机参数换算得到的 `6.32 N*m`，真实关节力矩限制为 `40 N*m`。schedule 已从旧的 `0.186-0.390 m` 重新生成到 `0.160-0.390 m`，默认 `0.16 m` 不再复用 `0.186 m` 边界增益；模型能直接提取等效腿参数的机械范围仍为 `0.186-0.390 m`，下段 `0.160-0.186 m` 明确使用参数拟合延伸。当前 1 ms 仿真权重为 `Q=[90,60,40,15,240,4,240,4,300,60]`、`R=[3.2,3.2,0.7,0.7]`，重点覆盖 `0.16/0.18 m`；全区间验证的最小可控秩为 `10`、最大拟合闭环特征值模约 `0.998641`；
- MuJoCo adapter 已读取 IMU roll 与 roll rate，observer 和 controller snapshot 对外提供这两项观测，但当前十维 LQR state 不扩展、也尚未实现 roll PD 或左右腿差分轴向力补偿；
- GUI 启动后保持所有执行器关闭，让机器人自由落地 `2 s`；随后由行为层用腿长/腿角 PD 将双腿调至目标长度和 `angle_body=-pi/2`，姿态连续稳定 `0.25 s` 后切入完整 LQR/支撑前馈。整个过程不使用 mocap weld，也不人为搬动机体或把轮轴对齐地面；
- 若自由落地 `2 s` 后直接开启完整 LQR，起控瞬间两轮和机体同时接地，但被动腿已塌到约 `0.158/0.145 m`，左右腿角状态约 `-0.72/-0.74 rad`；之后轮和四个关节力矩长期饱和、轮子离地，无法自行站起。加入上述腿部姿态预备后约在 `3.711 s` 切入平衡并成功站立，因此当前 LQR 的恢复域不包含未经整理的被动落地姿态；
- 腿长目标已由 `0.20 m` 改为 `0.16 m`，行为层约在 `2.775 s` 切入平衡；末段双轮接触率 `100%`、无其他部件触地，最大机体俯仰约 `1.95 deg`。最初出现的缓慢回正和机体持续触地并非腿长不足，而是行为层切换时误把整个当前状态复制为 LQR 参考，导致 `theta_l/theta_r/theta_b` 及其角速度参考非零；现已改为参考向量先清零、只捕获当前 `S/PSI`，并由单元测试保证六个姿态参考保持零；
- 不能在轮子仍悬空时直接开启完整 LQR：当前观测器会把轮自转积分为底盘位移，实测可迅速产生约 `-27 m/s` 的错误里程计速度并把腿打到角度限位。已经删除的早期静态 scenario 曾用人为对齐接地高度规避；当前自由落地流程不再使用该做法，后续应由接触状态或融合观测器正式处理；
- 腿长目标为 `0.20 m` 时，无界面静态站立测试在约 `3.711 s` 切入平衡，最后 `3 s` 双轮接触率为 `100%`、无其他部件触地，最大机体俯仰约 `1.48 deg`，最大俯仰角速度约 `0.00065 rad/s`；
- 平衡场景支持前进速度和偏航速度参考；每个控制周期分别积分为 `s_ref` 和 `psi_ref`，同时设置 `ds_ref` 和 `dpsi_ref`。GUI 循环演示静止、`+/-0.25 m/s` 前后运动和 `+/-1.57 rad/s` 左右偏航；
- 无界面运动验证中，前进/后退稳态速度约为 `+0.258/-0.258 m/s`，三秒真实位移约为 `+0.742/-0.769 m`；`+/-1.57 rad/s` 偏航目标实际约为 `+1.88/-1.85 rad/s`，存在约 18 至 20% 超调。四个阶段双轮接触率均为 `100%`，无其他部件触地；
- 独立的 `rm_balance_performance` 不加入 CTest，而是按相同初始条件逐档 reset，粗扫 `+/-1, 2, 2.5, 3 m/s` 与 `+/-pi, 2*pi, 3*pi, 4*pi rad/s`，并在 build 目录输出逐案例 summary 和 100 Hz trace。第一轮结果为：`+/-1 m/s` 稳定且满足 10% 跟踪线，但 2 秒刹停观察仍有约 `0.13 m/s` 反向残速；更高直行档出现底盘或右后腿触地；所有粗扫 yaw 档均在到达末段跟踪窗口前触地并伴随轮力矩限幅。详细判据和结果记录于 `docs/notes/performance-baseline.md`；本轮不据此修改 LQR、roll 补偿或模型参数；
- 固定 `+/-2 m/s`、扫描 `0.5, 1, 2, 3, 5 m/s^2` 的完整时序复测表明，所有加速度档都能保持有界并满足跟踪判据，没有出现控制器直接发散；`3/5 m/s^2` 会在正向加速或负向刹车阶段发生底盘触地。触地附近共同腿角约 `35-39 deg`、腿竖直投影约 `0.119-0.127 m`，这应解释为离地间隙/运动几何问题，而不是加速度稳定性失败；所有案例开始运动前仍有约 `0.286 m` 的 `S_ref-S` 偏差；
- 保持其他外部配置不变覆盖目标腿长后，`0.18 m` 可使正向 `3 m/s^2` 完整通过，但负向仍在刹车后的观察阶段触地；`0.20 m` 可使正反向 `3 m/s^2` 都完整通过并满足稳定、跟踪判据，`5 m/s^2` 仍会触地；`0.24 m` 因站起末段腿角误差约 `8.4/9.3 deg` 超过当前 `8 deg` 状态机阈值而无法进入平衡，尚不能评价其运动性能。本轮未同时修改进入阈值。由于 LQR 按实际平均腿长自动调度，`0.20 m` 的改善同时包含几何离地间隙和调度增益变化，不能只归因于腿更长；
- LQR 复调以 `0.16/0.18 m` 为主：降低 `S/theta_b` 权重并提高双腿角及角速度权重后，两个腿长均能完成并跟踪 `+/-3 m/s`，固定 `+/-2 m/s`、`0.5-5 m/s^2` 扫描也保持有限且无持续饱和；`Q[S]=90` 是本轮兼顾 `0.16 m` 反向刹车鲁棒性与跟踪误差的拐点。`+/-pi rad/s` 可跟踪，但 `2*pi rad/s` 起仍出现腿差分角、roll 和饱和快速增大，yaw 模型对齐与 roll 补偿仍未解决；
- 新增 `+/-2.5 m/s` 可回放档位；正式增益下正向案例在 `0.16/0.18 m` 的保持段 RMSE 分别约为 `0.188/0.182 m/s`，反向分别约为 `0.185/0.180 m/s`；四个案例均完整运行、满足跟踪且无执行器饱和，正向在加速、反向在刹车阶段仍记录到底盘剐蹭；
- 性能 benchmark 定位为快速实验而非通过门禁：案例按固定时间表运行到底，触地、姿态或未进入平衡记录为 `issue/issue_phase`，但不提前终止；仅非有限遥测会中止。工具只报告 `finite`、跟踪/刹停误差、姿态峰值、接触阶段和饱和比例等直接事实，不再合成单一稳定/通过标签，也不再输出最高通过档位；
- 既往仿真经验中，原地旋转时左右腿过度劈叉曾由模型与控制器参数/定义不对齐导致。因此后续 yaw 分析除降低偏航加速度外，应优先核对当前 MJCF 与 LQR 生成模型的质量、惯量、腿长、状态定义和输入符号，再决定是否调整 yaw-to-leg 耦合或加入 roll 补偿；
- 当前 yaw 观测不是简单的 `[-pi, pi]` 回绕角：adapter 输出回绕姿态，observer 对相邻帧差值 wrap 后持续累加，因此连续积分的 `psi_ref` 与连续展开的 `psi` 对齐，跨过 `pi` 不会凭空产生 `2*pi` 误差；
- 用生成器的同一组离散 `A/B` 和正式 `K(0.18 m)` 直接运行无接触、无饱和闭环 `x[k+1]=A_d*x[k]+B_d*K*(r[k]-x[k])` 后，`pi/2*pi/4*pi rad/s`、`15 rad/s^2` 的理想半差分腿角峰值分别约为 `12.0/21.6/31.3 deg`，共同腿角小于 `0.03 deg`，匀速稳定后两腿回零。MuJoCo 的 `pi` 档半差分峰值约 `11.4 deg`，与线性模型吻合，说明低速旋转时左右腿反向摆动是当前 yaw 模型和 LQR 控制分配主动产生的，不是单纯的仿真符号错误；
- `0.18 m / +2*pi` 的异常发生顺序为：目标 ramp 后约 `0.28 s` 单侧轮力矩先达到 `6.32 N*m`，约 `0.30 s` 一侧轮子离地，约 `0.36 s` 半差分腿角超过线性模型的 `21.6 deg` 峰值，约 `0.40 s` 关节开始饱和并带动共同腿角失控；腿差分超过理想峰值时 roll 仍仅约 `0.2 deg`。因此小幅劈叉属于理想控制动作，继续扩大则首先与接触不对称和饱和相关；缺少 roll 补偿会妨碍后续恢复，但在该正向案例中不是最早触发源；
- `2*pi rad/s` 偏航加速度扫描已经落地为 `yaw-acceleration` suite，并由 `tools/lqr/yaw_response.py` 使用正式 schedule 的同一组离散 `A/B + K` 生成无接触、无限幅先验。线性模型在 `0.18 m` 的 `1/2/3/5/7.5/10/15 rad/s^2` 全部档位都预测单轮峰值低于 `6.32 N*m`；MuJoCo 双向可用边界位于 `2-3 rad/s^2`：`1 rad/s^2` 正负两向跟踪和停止稳定均通过，`2 rad/s^2` 两向可跟踪但停止窗口仍有残余振荡，`3 rad/s^2` 正向失效而负向通过；
- `0.18 m / a3` 的 1 kHz trace 表明最早接触丢失主要是 `1-3 ms` 闪断，不是持续静态卸载。正向约在 `1.79 s` 同时进入接触丢失、法向力冲击和轮力矩饱和的正反馈，不能把第一帧闪断或轮力矩饱和单独解释为根因；
- `0.18 m` 静态站立共同腿角约 `+7.15 deg`；实际腿约 `65 mm` 垂直 COM 偏置对应约 `2.0 N*m` 常值重力矩和约 `+8 deg` 自然 trim。当前生成器只把轴向 COM 投影和等效惯量带入齐次零点 `A/B`，没有表示该仿射常值项。MuJoCo 对照现已改为围绕 ramp 前一秒站立均值预测扰动，不能再把实际非零状态直接当作零平衡模型的绝对状态；
- trim 对齐后，`0.18 m / a3` 正向的半差分/共同模态误差分别约在 `1.272/1.714 s` 超过 `2 deg`，负向分别约在 `1.410/2.009 s`。即使全程接触且无饱和的 `a1`，高偏航速度下仍有约 `2-3 deg` 误差；因为当前线性方程不含速度乘积项，`A/B` 只适合初期力矩和腿摆量级先验，不能覆盖高偏航率科氏、离心和陀螺耦合；
- motion config 保留默认开启的 `S` 位置反馈开关。关闭位置反馈时只令 `ref_s=s`，`DS` 速度通道仍正常工作；
- `rm_balance_trim_scan` 在 `0.18 m` 下关闭 `S` 并扫描共同腿角参考：零补偿时 pitch 约 `-2.075 deg`、车以约 `-0.32 m/s` 滑移；`+5.25/+5.50/+5.75 deg` 均基本消除速度漂移，其中 `+5.50 deg` 的 pitch 均值约 `+0.055 deg`，3 秒基座位移约 `-1.6e-6 m`，双轮持续接触且无饱和；
- 默认通用腿长已统一为 `0.18 m`。`+5.50 deg` 共同腿角偏置已移入 control config 的 LQR compensation，作为该腿长下的平衡工作点补偿固化；motion/state machine 继续输出零腿角名义参考，控制核心仅在调用 LQR 前生成有效参考并叠加补偿，不修改观测、运动学零点或腿部定位目标；
- 纯 PD 在固定 `0.30 m / -pi/2` 的 8 秒测试中稳态误差约为 `12 mm / 2.1 deg`，暂未加入积分或重力前馈；
- 解析运动学与 MuJoCo `framepos` 多姿态对照的已知最大偏差约为 `9.1 mm / 1.8 deg`，该偏差保留为当前实际闭链模型的可见特性；
- C++ 侧分为 `MujocoPlant`、`MujocoAdapter`、`SimulationRunner` 和 `MujocoViewer`；正常入口实时无限运行到用户关闭 GUI，`run_for()` 只供 headless 测试使用；
- benchmark 已按 `common/performance/trim` 拆到 `src/benchmark`：`CsvWriter` 统一目录创建、转义和列数检查，`SimulationSampler` 统一基座自然坐标速度与轮地接触采样，`CommonDiagnostics` 统一有限性、接触、执行器饱和和峰值统计，`SampleStatistics/LinearTrend` 统一基础统计；`PerformanceBenchmark/PerformanceScenario` 与 `TrimScanner` 各自拥有具体实验实现，各自的 `main.cpp` 只保留 CLI、案例选择和打印。GUI 案例回放共用 performance scenario 和公共问题判定，仿真核心不再编译具体实验时序。重构后不再保留只有入口文件的顶层 `benchmarks` 目录；重构前后直线加速度 suite 及 `5.5 deg` trim 单点的 summary/trace 均字节一致；
- 物理和控制周期暂定均为 1 ms。加载后只在内存中覆盖 `mjModel.opt.timestep`，不修改原始 MJCF；
- 当前机体具有自由基座、上游 USD 惯性参数、地面和轮地接触；mocap weld 默认关闭，只在测试中显式启用；
- CMake 支持用 `MUJOCO_ROOT` 指向 Linux Python wheel 或官方 MuJoCo 包，并为 Windows MSVC 官方包复制运行时 DLL；Windows 原生链接还需要 `lib/mujoco.lib`，当前检查到的 `armsim` Python wheel 只有头文件和 `mujoco.dll`，不能单独作为 Windows C++ SDK；
- Linux 环境的官方 MuJoCo 3.9.0 SDK 位于 `/home/l/.local/opt/mujoco-3.9.0`；当前 `470e08f` 已在该环境完成构建和 11/11 CTest，并再次验证 C 运动学/雅可比、模型接线与功率方向、site 几何对照和 8 秒悬空定姿测试。该路径只是本机记录，不是跨平台默认值；
- 当前 Windows 环境把不入库的本地依赖放在 `third_party/mujoco-3.9.0` 与 `third_party/glfw`，使用 MSVC 19.44、Windows SDK 10.0.26100 和 MuJoCo 3.9.0 完成 Release 构建，11 个 CTest 全部通过，GUI 已实际启动并运行站立、前后运动与左右偏航阶段；
- Windows 配置应同时显式传入 `-DMUJOCO_ROOT=<third_party/mujoco-3.9.0>` 与 `-DFETCHCONTENT_SOURCE_DIR_GLFW=<third_party/glfw>`。只设置 `MUJOCO_ROOT` 不会阻止 GLFW FetchContent 联网；中断 Git clone 可能留下仍在运行的子进程和 `build/_deps/glfw-src/.git/objects/pack/tmp_pack_*` 只读文件，导致后续删除 build 目录受阻；
- MuJoCo 3.9.0 将尺寸类型 `mjtSize` 定义为 `int64_t`；adapter 的 actuator 数量缓存使用同一类型，避免将 `mjModel::nu` 收窄到 `int` 的 MSVC C4244 警告。

## 后续需要确认/实现

1. 以 MJCF/MuJoCo 为当前仿真后端；USD 只保留为外部参考资产。
2. 核对当前模型每段质量、质心、惯量是否为目标参数；不要用旧 MATLAB 脚本覆盖。
3. 恢复 `S` 通道复测 `0.18 m / +5.50 deg` 静态位置误差，再重跑 yaw suite；若以后使用其他通用腿长，再标定补偿与腿长的关系，不直接沿用当前常值。
4. 继续用 `yaw-acceleration` suite 区分 `0.18 m / 2*pi rad/s` 的左右闭链/轮接触不对称与缺少 roll 控制的贡献；`A/B + K` 只作为 trim 周围的初期先验，不用于替代高偏航率非线性验证。
5. 在无噪声十维状态方向验证完成后，再加入 IMU 与编码器速度融合和状态滤波。
6. 将当前已验证的 MuJoCo 关节映射与后续实车 adapter 分开维护，避免把模型 joint axis 符号机械照搬到硬件。
7. 修正明显偏大的腿部质量参数后，重新提取等效腿质心/惯量并生成 LQR 增益调度；当前 XML 单侧腿和轮共 `4 kg`，实车经验值约不超过 `2 kg`。
8. 加入接触状态或融合观测器，消除轮子离地时纯轮速里程计造成的错误位移反馈。
9. 单独参数化执行器、摩擦、结构柔性、传感器和接触误差，避免不同 sim-to-real gap 混杂。

## 快速恢复时优先阅读

1. 本文件。
2. `docs/notes/performance-baseline.md`。
3. `docs/notes/lqr-validation.md`。
4. `references/连杆示意/示意图.svg`。
5. `models/MJCF/COD-2026RoboMaster-Balance.xml`。
6. `references/rm2026cb-balance-chassis/Tasks/balance_chassis.c`。
7. `references/rm2026cb-balance-chassis/Tasks/balance_chassis/bc_control.c`。
8. `references/rm2026cb-balance-chassis/Tasks/balance_chassis/bc_behavior2.c`。
9. `references/rm2026cb-balance-chassis/Tasks/balance_chassis/bc_estimator.c`。
10. `references/matlab_scripts/lqr.m`（仅参考生成方法）。
