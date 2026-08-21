# C++20 控制核心

更新日期：2026-08-21

> `feat/fudan-model-adaptation` 分支状态：本文件记录的 C++20 控制核心是死代码。源码继续保留，但不进入 CMake 构建，不做复旦参数适配，不注册测试，也不提供专用 viewer。以下内容是 2026-08-18 的历史实现记录，不代表本分支当前能力或验收范围。

新控制核心不是对 C 文件的机械改写，而是以 `c-core-v1` 为行为基线建立的独立 C++20 实现。两套核心互不依赖；当前 C 核心继续承担完整仿真、GUI 和 benchmark，C++ 核心最终供其他仓库的实车工程直接复用。

## 第一阶段边界

C++ 核心当前只实现系统启停和初次起立纵切片。system 与 motion 保持为两个层级：system 负责 `Off/On`，motion 负责 `Idle -> SelfRighting -> LegPositioning -> BalanceEngaging -> Active`。没有 Fault 状态。打开 system 后仍停在 Idle，收到 `balance_restart` 才开始起立；system On 时从任意 motion 状态再次收到 restart 都会重新开始起立。

system 和 motion 的每拍顺序都保持为 `transition -> action`。restart 先进入 `SelfRighting`，motion transition 在同一拍转到 `LegPositioning`，随后执行定位 action，与 C 基线的起立时序一致；状态转移和 action 不合并成单个按条件堆叠的 update。

Active 捕获并固定首次站立位置与 heading，长期输出原地 LQR 平衡力矩。它不接受前进、云台 heading follow、support、落地、STEP_DOCK、跳跃或自旋语义；本阶段的操作命令只有 system enable 和 balance restart。

公共接口是有状态黑盒 `Controller::tick(sensor, command, dt)`。输入为操作手使能/重启命令和原始可获得观测，输出为限幅前请求、最终六路电机力矩和诊断 snapshot。控制周期内不使用动态分配、异常、RTTI、虚接口或仿真类型。

`Controller` 只编排一拍。各算法状态保留在对应模块对象内，facade 不平铺或重复保存这些内部状态：

```text
SensorFrame
  -> Observer (LegKinematicsSolver + VelocityEstimator)
  -> Estimate
  -> SystemStateMachine
       -> MotionStateMachine
  -> ControlCommand
  -> ControlCore
       -> PdController
       -> LqrController
  -> ControlOutput
  -> OutputGate
  -> Actuation
```

`SnapshotAssembler` 从上述模块的本拍输出组装诊断，不参与算法。所有模块都是独立类型，可以脱离 `Controller` 构造和测试；模块之间不持有 `Controller` 指针，也没有反向回调或通过共享字段形成的隐式依赖。

配置也遵循相同边界。`ObserverConfig`、`MotionConfig`、`ControlCoreConfig` 和 `OutputGateConfig` 分别定义在所属模块旁，运动学、速度估计和 PD 控制器只接收各自更小的配置；没有参数的模块不创建空配置。顶层 `ControllerConfig` 只组合这些模块配置，`Controller` 构造时将对应子配置分发给模块，不把整份配置向下透传。

目录沿用已验证的 C 核心领域分层：起立协调和条件保持属于 `state_machine/`，PD/LQR 策略属于 `control_law/`；observer、运动学和速度估计保留在控制根层。根层同时保留 facade、公共类型、输出门和 snapshot 组装，不把不同领域的类平铺为一份清单。

轮速观测的开启具有明确时序：`Controller` 在 observer 更新前，根据 system 和 motion 的上一拍状态生成 `ObservationContext`。因此进入 `BalanceEngaging` 的转换拍仍不使用轮速，下一拍才开始累计轮速启动延迟。这既保持 `c-core-v1` 行为，也避免 observer 读取状态机内部状态。

`OutputGate` 是执行器前的最终边界，不是控制策略。system 未使能时整帧六路输出归零；六路请求中任一路出现 `NaN/Inf` 时同样整帧归零；请求全部有限时才分别应用轮电机 `6.32 N*m` 和关节电机 `40 N*m` 限幅。当前没有 fault 锁存、诊断或恢复状态，这部分明确留待后续设计。

内部复刻 `c-core-v1` 的腿运动学、Jacobian、速度估计起立路径、十维状态、PD、增益调度 LQR、roll 支撑、参考捕获、输出门控和力矩限幅。定位稳定保持为 `100 ms`，平衡接管为 `50 ms`，目标为 `0.18 m/-90 deg`。进入 Active 前的状态转换、估计和输出通过 C/C++ 同输入对照；C++ Active 不复刻旧 C ACTIVE 的 support 和运动协调器，只保持固定位置和 heading 的原地 LQR 平衡，因此入口之后独立做闭环验收。

## 验证、仿真与可视化

C++ 仿真代码集中在 `src/sim/cpp/`。其中 `balance::sim::cpp::SimulationRunner` 是隔离的 MuJoCo 适配器，复用 `balance_mujoco` 中的平台和通道标定，但不依赖旧 `balance::sim::SimulationRunner` 或 C 控制核心。它每拍只转换 sensor、调用 C++ controller、写回 actuation 并推进 MuJoCo，不拥有 startup 时序；测试和 viewer 分别在外层生成 system enable/restart 命令。当前闭环门禁要求持续站立 `8 s`，末 `3 s` pitch 小于 `5 deg`、pitch-rate 小于 `0.2 rad/s`、左右腿长差不超过 `2 mm`、双轮接触率至少 `99%` 且没有非轮触地。

常驻验证分为三层：`cpp_controller_startup` 对照 C/C++ 的关闭、待机、起立转换、平衡接管、restart 和关闭行为；`cpp_control_modules` 独立检查数学、observer、状态机、PD/LQR、控制核心、完整帧输出安全和 snapshot；`mujoco_cpp_startup` 独立运行 C++ 核心的 MuJoCo 起立与原地保持。2026-08-18 的完整仓库回归为 `36/36` 通过。

`rm_balance_cpp_viewer` 使用同一 `cpp::SimulationRunner` 提供无 UI 的交互场景窗口：viewer app 在仿真两秒后自动使能并发出起立命令，鼠标只控制相机。`MujocoViewer` 已独立为 `balance_mujoco_viewer`，只读取 MuJoCo `mjData`，接受通用 viewport insets 和显式相机输入许可；它不包含 ImGui，不读取 sidebar、C/C++ snapshot，也不保存暂停、复位、驾驶或 STEP 状态。完整 C GUI 的入口、应用编排、UI 和 trace 集中在 `src/sim/gui/`；键位现由独立 `InteractiveKeyboard` 采集为按键电平和边沿，暂停/复位由 `SimulationApp` 解释，STEP 请求锁存由 `InteractiveScenario` 持有。

现有 `SimulationUi` 仍是完整 C 核心专用诊断面板，这不妨碍 C++ viewer 独立运行。未来若需要 C++ 诊断，应在 app/UI 边界增加中立 display model 或独立面板，不能重新让 viewer 认识控制器数据。

后续每扩展一项能力，先在 C++ 核心内定义所有权与语义，再增加对应模块测试、C 对照和独立闭环测试；不要为了复用而让新核心调用旧 C 算法，也不要在仿真 runner 中补控制状态。motion action 产生结构化 `ControlCommand`，明确腿长、腿角和轮端策略、目标参考及反馈屏蔽；控制律只消费 `Estimate + ControlCommand`，不根据 motion 状态反推行为。状态机状态和条件保持时间通过独立诊断状态进入 snapshot，输出门只处理使能、有限性和力矩限幅。
