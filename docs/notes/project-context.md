# rm-balance-sim 项目上下文

> 用于后续开发会话快速恢复当前事实、设计边界和未决事项，不保存逐轮实验流水账。最近整理：2026-08-12。

## 当前状态

项目已经具备可运行的 MuJoCo 闭链 plant、纯 C 控制核心、分层状态机、状态估计、增益调度 LQR、GUI、benchmark 和 TOML 实验工具。平地 NORMAL 运动的纵向、原地 heading、稳态转弯和开环八字已经形成稳定基线，本轮不再把一般平面运动性能或 LQR Q/R 调参作为默认主线。支持力识别、抬升释放和实体平台驶离实验已经建立；ACTIVE 内首版 support phase 已接管离地、空中伸腿、落地收腿和恢复，当前主线转为扩大落地工况验证并继续收紧 ACTIVE 的职责边界。

最近一次已提交参数变更是 LQR 差分模态降档：共同腿角/角速度权重保持 `60/1`，差分权重从 `1920/32` 降为 `960/16`。当前未提交工作把支持力融合和落地策略接入生产 ACTIVE 路径，并新增正式 controller 平台案例；完整回归为 `26/26`。

## 设计边界

- 控制核心使用纯 C 和 SI 单位，不依赖 MuJoCo、GLFW、CAN、HAL 或线程实现；仿真与未来嵌入式工程通过各自 adapter 接入同一控制核心。
- 控制层使用虚拟腿长度/角度和虚拟力 `F/Tp`，plant 保留真实多连杆闭链、偏置质心和惯量；运动学等效不代表动力学等效。
- 不根据闭环表现随意填写未知 plant 参数。物理参数变化后必须重新验证闭链、符号、运动学/Jacobian、参数提取、LQR 调度和性能基线。
- `/home/l/SaLT/wheelleg` 是只读历史项目；`references/rm2026cb-balance-chassis/` 和 `references/matlab_scripts/` 是实车架构及算法参考。旧参数不能静默混入当前 plant。
- 调参和新功能回归必须分开检查响应、姿态、腿状态、轮地接触、非轮触地、净空以及限幅前后输出，不能用单一 pass/fail 或 RMSE 掩盖物理代价。
- 前进和刹停共用同一个加速度斜坡。起立固定在 `0.18 m` 完成，进入 ACTIVE 后才线性改变工作腿长；`0.38 m` 是需要降额的特殊工况。
- 实验开关和案例参数放在 benchmark/TOML 层，不为方便测试污染生产 controller config。

## Plant 与坐标

- 当前模型为 `models/MJCF/COD-2026RoboMaster-Balance.xml`。每侧由前后支链和 equality `connect` 形成真实闭链，主动输入为左右前髋、后髋和轮轴。
- 控制运动学使用每侧两个主动髋角计算虚拟腿长度 `L`、相对车体腿角 `theta` 和解析 Jacobian，通过 `[tau_front,tau_rear]^T = J^T[F,Tp]^T` 映射输出；几何参数为 `l1=0.215 m`、`l2=0.254 m`。
- 坐标为 FLU：车头 `+X`、左侧 `+Y`、上方 `+Z`，正 pitch 为车头下压，正 yaw 为左转。上游 XML 左右命名与物理侧相反，adapter 固定映射 `BC_L -> XML Right_*`、`BC_R -> XML Left_*`。
- adapter 零偏由 `tools/calibration/calibrate_leg_adapter.py` 在 `0.18-0.21 m` 站立区拟合。当前最大站立区长度/角度误差约 `1.594 mm/0.328 deg`；全行程误差更大，若修改 joint/site frame 必须重新标定和回归。
- 总质量为 `21.450 kg`，其中 `base_link=17.650 kg`、单侧腿不含轮 `1.190 kg`、单轮 `0.710 kg`。base pitch/yaw 惯量为 `0.367565/0.413477 kg*m^2`；LQR 使用全装配体在 `0.18 m` 工作点的 yaw 惯量 `0.588029054 kg*m^2`，不是 base 惯量，也不额外乘二。
- 物理右腿闭链存在约 `1.95 mm` 固定横向错位；机体复杂 mesh 仍直接参与地面碰撞；物理和控制周期均为 `1 ms`。这些都是解释接触和实车差异时必须保留的限制。

## 控制架构

正式数据流为：

```text
MujocoAdapter::read
    -> bc_controller_update
    -> bc_controller_set_command
    -> bc_controller_calculate
    -> bc_controller_execute
    -> bc_controller_capture_snapshot
    -> MujocoAdapter::write
    -> MuJoCo step
```

- `bc_controller_t` 是正式 facade；`update` 更新运动学和 observer，`calculate` 推进状态机、参考和控制律，`execute` 只做 system 硬门控、限幅和最终输出。
- `bc_controller_snapshot_t` 是调用者持有的诊断快照，供 GUI、benchmark、测试和未来遥测使用；它不是控制器内部状态副本，也不是串口协议。
- actuator 在 C core 中表达为左右腿前/后关节力矩和左右轮力矩，不暴露 MuJoCo actuator 下标。轮/关节限幅为 `6.32/40 N*m`。

状态机当前分层为：

```text
system: OFF / ON / FAULT
motion: IDLE / SELF_RIGHTING / LEG_POSITIONING / BALANCE_ENGAGING / ACTIVE
forward: IDLE / HOLD / VELOCITY
support: GROUND / AIRBORNE / LANDING_RETRACT / GROUND_RECOVER
```

- `BC_MOTION_ACTIVE` 当前统一合成纵向、云台跟随、front/rear、工作腿长和 support 请求，尚无独立 ACTIVE 类型。support 模块不接收或覆盖整份命令，而是输出轮能力、反馈 mask 和逐腿策略请求；motion 是最终组装者。目标架构仍将 task、longitudinal、yaw、腿长和 support 分离，避免组合状态和隐式覆盖顺序。
- ACTIVE 从 `HOLD` 开始；`VELOCITY` 关闭 `S` 位置误差但保留 `DS`。命令和参考归零、融合 `DS` 与原始轮速均低于 `0.05 m/s`、轮速可靠并连续满足 `0.25 s` 后回到 `HOLD`，重新捕获停车位置。
- 普通 heading follow 与 forward mode 独立，在 `HOLD/VELOCITY` 中始终启用 `PSI/DPSI`。motion 根据云台相对角以 `5 deg` 滞回选择 chassis front/rear，rear 时反转纵向命令。
- operator command 目前只有系统使能、单周期 restart 和云台坐标下前进速度；heading 来自云台相对角/速度。任务级高速 SPIN 尚未实现，不能由普通 heading 命令幅度隐式触发。
- 默认起立和工作腿长均为 `0.18 m`。`LEG_POSITIONING/BALANCE_ENGAGING` 固定使用 `0.18 m`，ACTIVE 后以 `0.10 m/s` 拉到配置工作点。

## 参考、估计与控制律

- forward reference 上限为 `3 m/s`、默认斜坡为 `5 m/s^2`，输出 `DS_ref` 并积分 `S_ref`。yaw reference 从云台反馈重建 `PSI_ref/DPSI_ref`，正式上限为 `+/-1.5*pi rad/s`、加速度为 `10 rad/s^2`。
- observer 用 IMU 和共同轮速融合 `DS`，再积分得到 `S`。轮速创新使用 NIS 与 `20 ms` 迟滞；AIRBORNE 时生产状态机强制轮速不可靠、KF 只做 IMU 预测，触地后重新经过恢复持续时间。降级不重置状态或 bias。IMU 外参、噪声和阈值仍需实车标定。
- LQR 状态为 `[s,ds,psi,dpsi,theta_l,dtheta_l,theta_r,dtheta_r,theta_b,dtheta_b]`，输入为 `[T_wheel_l,T_wheel_r,Tp_leg_l,Tp_leg_r]`，调度范围为 `0.160-0.390 m`。
- 正式基础 `Q=[90,260,40,15,60,1,60,1,900,120]`、`R=[1.6,1.6,0.7,0.7]`。非对角腿块表达共同/差分模态：腿角为 `60/960`，腿角速度为 `1/16`；它允许纵向共同摆腿，同时约束偏航差分劈腿。
- yaw 加速度前馈为 `u = K(r-x) + 0.9*F_yaw*DDPSI_ref`。关闭它会显著增大 heading 滞后和停转残振，因此保留；其模型基于接近零纵向速度的线性工作点，尚未做速度调度。
- roll 不进入十维 LQR，由 `Kp=800 N/rad`、`Kd=60 N/(rad/s)`、限幅 `200 N` 的独立 PD 生成左右腿差分轴向力；落地阻抗复用同一普通支撑和 roll 前馈，不复制另一套参数。
- `control/support_force` 从每侧两个关节的实际力矩经 `J^T` 反解轴向力/腿矩，再投影为竖直支持力；输出原始值、滤波值及带滞回和持续时间的 `GROUND/AIR` 诊断。当前默认低通系数为 `0.2`，离地/触地持续时间为 `20/15 ms`；整机 support phase 另融合两侧滤波支持力和 IMU 比力用于快速离地/触地。
- 站立工作点使用共同腿角补偿 `+2.42 deg` 和每腿 `76.204 N` 支撑前馈。定位阶段只用长度/角度位置反馈，支撑与 roll 差分力只属于 `POSITION_SUPPORT`。

## 已建立的性能基线

- 正式 benchmark 只保留 `+/-3 m/s @ 5 m/s^2` 直线与 `+/-1.5*pi rad/s @ 10 rad/s^2` heading 四例；探索扫描由 TOML 展开，结果将 `valid`、响应、停车、接触和饱和分开报告。
- `0.18 m` 正式直线实际 `t90=0.784/0.778 s`，最大 pitch 约 `2.19/2.44 deg`；正式 heading 稳态约 `+/-4.711 rad/s`。四例均正常停车、双轮接触且无执行器饱和。
- `0.24 m` 可运行 `+/-3 m/s @ 5 m/s^2`；`0.38 m` 已以 `+/-1.5 m/s @ 2 m/s^2` 验证，但不使用正常腿长性能门槛。
- 转弯粗包络显示 `0.5*pi rad/s` 在 `1-3 m/s` 范围稳定；`pi rad/s` 在约 `2 m/s` 仍可靠，继续升速开始明显卸载内侧轮；`1.5*pi rad/s` 的实际横向加速度约在 `7.3 m/s^2` 附近封顶。
- `figure_eight_open_loop` 使用 `2 m/s、+/-pi rad/s`，最大 pitch/roll 约 `2.81/2.90 deg`，路径阶段双轮接触率 `99.68%`，无饱和或非轮触地；参考闭合约 `4 mm`，实际轮轴闭合误差约 `0.307 m`。它是开环运动复现，不应冒充严格路径跟踪。
- 探索性 `+/-4*pi rad/s @ 10/15 rad/s^2` 在理想力矩模型中均通过；`15 rad/s^2` 的 `t90` 约 `0.763 s`。MJCF 没有电机反电动势或转速-转矩曲线，因此该结果只证明控制稳定性，不能直接作为实车速度能力结论，正式 yaw 上限仍为 `1.5*pi rad/s`。
- 差分权重从 `1920/32` 降到 `960/16` 后，正式基线不变；`4*pi @ 15 rad/s^2` 的最大腿差分约从 `0.94 deg` 增至 `1.26 deg`，仍双轮接触且无饱和。完整扫描和历史调参记录见 `docs/notes/performance-baseline.md`。

## 仿真与实验工具

- C++ 仿真层由 `MujocoPlant`、`MujocoAdapter`、`SimulationRunner`、`MujocoViewer`、`SimulationUi` 和场景编排组成。GUI 与 performance benchmark 共用 `PerformanceScenario`。
- GUI 侧栏显示状态机、state/reference、roll、云台反馈、腿运动学和限幅前后输出；`--trace <csv>` 可写交互式 1 kHz trace。
- 独立 drop benchmark 既支持以轮胎碰撞几何净空执行 `200/400 mm` 抬升释放，也支持出生在实体平台后自动驶离的 `200/400 mm` 跌落。平台基线覆盖 `0.18/0.24 m` 腿长、`0.5/1.0/2.0 m/s` 与 `length_only/leg_lqr` 共 24 例，另有 `200 mm、1.5/2.0/2.5 m/s` 的固定腿长与空中伸到 `0.38 m` 对照；速度参考跨越空中与落地保持不变。MuJoCo 接触真值只负责实验切换，支持力与 KF 对真值误差只并行记录，不进入生产控制器。
- 当前实体平台实验首先测到的是直角边缘通过性，不能直接解释为纯空中控制能力。`0.18 m` 低速驶离时底盘或后腿会持续碰撞平台边缘，碰撞在 pitch 恶化之前发生；`0.24 m` 明显缩短碰撞，并使 200 mm 六例全部恢复，但 400 mm 的 `0.5/1.0 m/s` 仍会碰边发散，两个高度的 `2.0 m/s` 均无边缘碰撞并恢复。KF 空中误差仍需在排除边缘碰撞的独立实验中归因。
- 200 mm 正常速度下的探索实验表明，离台后将腿从 `0.18 m` 目标快速伸至 `0.38 m`，可把首次触地前机体下降从约 `171-178 mm` 降至 `19-24 mm`，触地 pitch 降至约 `0.22-0.31 deg`，触地竖直速度降至约 `0.67-0.78 m/s`。实际触地腿长约 `0.31-0.35 m`，空中关节峰值请求约 `30-31 N*m` 且没有触发 `40 N*m` 限幅；这只验证快速接地有效，触地顺应和缓慢回收尚未设计。
- `leg_lqr` 空中策略的直接目标是让虚拟腿相对世界接近竖直，而不是最小化机体 pitch；有效腿角参考包含约 `+2.42 deg` trim。按正确指标比较，`2.0 m/s` 名义触地腿角由 `length_only` 的约 `6.89/5.24 deg` 收到 `4.35/3.96 deg`，左右差由 `1.65 deg` 降到 `0.39 deg`；`2.5 m/s` 由约 `8.15/8.27 deg` 收到 `5.09/5.05 deg`。在 `+/-0.5 rad/s` 离台 pitch-rate 扰动下，LQR 也都降低最大腿角误差和左右差。它会与机体交换角动量，因此机体 pitch 可能略大；不能用机体 pitch 单独否定该策略。
- 首轮主动悬挂探索固定 `200 mm、2.0 m/s、0.18 -> 0.38 m、leg_lqr`，触地后恢复轮 LQR 和完整姿态反馈，仅比较保留支撑前馈并强撑 `0.38 m` 与五组单腿轴向阻抗。六例均恢复，阻抗候选的最大腿压缩为 `92-99 mm`，明显大于位置强撑的约 `37 mm`。峰值数据复查后不能用来选择 `K/D`：共同的约 `267.2 N` MuJoCo 法向力峰值来自触地第一个采样，此时参数尚未来得及产生差异；共同的约 `187.0 N` 估计支持力峰值则由五组均撞上 `180 N` 轴向力上限主导。请求力峰值实际分布在约 `235-322 N`，说明候选有差异但被执行力约束截平。当前只能确认这套受限阻抗可以完成落地，尚不能确认单一最优参数或把峰值下降完全归因于 `K/D`；真值接触、阻抗参数和力约束仍只存在于隔离 benchmark。
- Jacobian 复核表明 `0.18-0.38 m` 腿长范围内，`40 N*m` 软件限幅在纯轴向输出时至少对应约 `263 N/leg`；实验上限已从 `180 N` 放宽到 `240 N`。同组 `200 mm` 案例仍全部恢复且没有关节饱和，`K=400/800/1200,D=80` 最大压缩约为 `83/76/73 mm`，`K=800,D=40/120` 约为 `84/73 mm`。这种约 `10 mm` 的竖直差异很难从 GUI 整机姿态辨认，而且除 `D=40` 外仍会碰到 `240 N` 上限，因此参数仍未定稿。下一轮应以压缩余量、竖直速度衰减、反弹/冲量和关节力矩余量评价少量软/中/硬候选，并跨触地强度验证，而不是继续靠视觉密扫相近参数。
- 固定触地平衡点会一直顶住长腿，不符合落地后快速回收的意图；隔离策略现改为每腿触地捕获 `L_eq=L_touch`，再以 `0.8 m/s` 将平衡点降至正常工作腿长。相同 `200 mm` 案例均恢复且没有失撑、反弹、关节饱和或其他部件触地，实际腿长约 `0.23-0.35 s` 内收至 `0.20 m` 以下，瞬时最低约 `0.154-0.170 m`。方向可行，但回收末端存在约 `10-26 mm` 动态下冲；下一步重点是末端减速和退出/切回条件，而不是继续密扫固定平衡点的 `K/D`。
- 隔离实验已加入无扰退出：双轮接触、平衡点到达工作腿长、双腿长度速度和机体竖直速度均低于 `0.1 m/s` 并保持 `50 ms` 后，按当前轴向力反算位置 PD 等效参考，再以 `0.1 m/s` 拉回 `0.18 m`。五组在触地后约 `0.47-0.59 s` 切回 `POSITION_SUPPORT`，切换首采样支持力变化小于 `1 N`，随后 `50 ms` 最大关节力矩约 `12.5-13.3 N*m`，未产生二次冲击。当前真值接触和退出编排仍只属于 benchmark；下一步才是将该阶段语义接入 ACTIVE 接触状态机并改用生产支持力诊断。
- support phase 接管前先以影子方式验证：只等待单腿完整诊断时离地/触地延迟约为 `55/17 ms`；增加“两腿滤波支持力 `<50 N`、IMU 比力范数 `<5 m/s^2`、持续 `5 ms`”的快速离地融合，以及深度卸载后支持力回升持续 `3 ms` 的触地确认后，名义案例延迟降至约 `33/4-5 ms` 且阶段不抖动。
- 首版 support phase 已从影子诊断升级为正式 ACTIVE 请求：AIRBORNE 关闭轮输出、伸腿至 `0.38 m` 并只保留腿角 LQR；LANDING_RETRACT 恢复轮和完整姿态 LQR，以 `K=800 N/m,D=80 N*s/m`、`0-240 N`、`3000 N/s` 的逐腿轴向阻抗和 `0.8 m/s` 移动平衡点快速收腿；GROUND_RECOVER 反算位置 PD 等效参考并以 `0.1 m/s` 回到普通支撑。support 不覆盖整份命令，最终合成仍由 motion ACTIVE 完成。
- 新增 `platform_drop_200mm_l0p18_v2p0_leg_lqr_landing_controller` 不使用 MuJoCo 接触真值或 `step_with_control_transform()` 驱动控制。实际边缘速度约 `1.94 m/s`，离台后约 `10 ms` 进入 AIRBORNE，触地后约 `8 ms` 进入 LANDING_RETRACT，约 `0.457/0.618 s` 进入 RECOVER/回到 GROUND；无发散、反弹、其他接触或关节饱和。正式直线、偏航和八字仍全程 GROUND。上述阈值和落地参数仍只经过理想仿真，未覆盖实车噪声、颠簸、单轮卸载、非对称落地或 `400 mm` 压力工况。
- `tools/experiments/run_experiment.py` 从 TOML 生成隔离 schedule、构建和案例结果；`tools/experiments/plot_trajectory.py` 生成无额外依赖的轨迹 SVG。
- 本地 MuJoCo、GLFW、ImGui 可置于被忽略的 `third_party/`，通过 `MUJOCO_ROOT`、`FETCHCONTENT_SOURCE_DIR_GLFW`、`IMGUI_ROOT` 显式指定；详细构建命令见根 README。

## 仍需记住的未决事项

- ACTIVE 接管首采样曾出现约 `9.7 deg` pitch 瞬态，发生在工作腿长拉升前，应作为独立起立接管问题处理。
- 轮速被拒绝期间没有绝对位置观测，恢复后不能修正累计的 `S` 漂移；停车安全门槛使用速度与可靠性，不声称绝对位置可观。
- 高横向加速度下的内侧轮卸载仍是物理现象；未来涉及高速转弯、路径控制或 roll 策略时，需要接触观测和实车等效输入，不能只提高 PD/LQR 权重。
- 当前 plant 只有力矩限幅，没有电机速度、功率、热和电池模型。涉及性能上限的新功能必须先明确是否需要补这些约束。
- Windows 构建由用户侧复核；本地 Linux 是当前自动验证环境。

## 主要材料与恢复顺序

1. 本文件：当前事实和边界。
2. `docs/notes/active-motion-design.md`：ACTIVE 运行期架构、模块所有权、交叉约束和重构顺序。
3. `docs/notes/performance-baseline.md`：逐轮实验、失败归因和完整数值。
4. `docs/notes/lqr-validation.md`：当前模型、Q/R 和调度验证。
5. `docs/notes/hardware-bringup.md`：实车逐级部署。
6. `models/MJCF/COD-2026RoboMaster-Balance.xml`：当前 plant。
7. `references/rm2026cb-balance-chassis/`、`references/matlab_scripts/lqr.m` 和 `references/SJTU balance control/WBR_modeling.html`：实车与理论来源。
