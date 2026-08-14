# rm-balance-sim 项目上下文

> 用于后续开发会话快速恢复当前事实、设计边界和未决事项，不保存逐轮实验流水账。最近整理：2026-08-13。

## 当前状态

项目已经具备可运行的 MuJoCo 闭链 plant、纯 C 控制核心、分层状态机、状态估计、增益调度 LQR、GUI、benchmark 和 TOML 实验工具。平地 NORMAL 运动的纵向、原地 heading、稳态转弯和开环八字已经形成稳定基线，本轮不再把一般平面运动性能或 LQR Q/R 调参作为默认主线。支持力识别、抬升释放和实体平台驶离实验已经建立；ACTIVE 内首版 support phase 已接管离地、空中伸腿、落地收腿和恢复，当前主线转为扩大斜坡、碰撞和落地工况验证并继续收紧 ACTIVE 的职责边界。

生产 ACTIVE 落地路径已由提交 `0bd19ba` 接入，benchmark 清理和地形工具已由 `c1ecb5f` 提交。独立 `17 deg` 宽坡回归已修复碰撞后 support phase 卡死和 KF 轮速失锁自锁；当前又增加了带平台和下坡的工作腿长对照矩阵，完整回归为 `27/27`。

## 设计边界

- 控制核心使用纯 C 和 SI 单位，不依赖 MuJoCo、GLFW、CAN、HAL 或线程实现；仿真与未来嵌入式工程通过各自 adapter 接入同一控制核心。
- 后续产品方案已经决定全面使用 C++，但当前阶段继续维护现有 C 控制核心及其边界，不在功能修复中夹带 C 到 C++ 的迁移。当前架构审查和所有权整理仍然有效，也应为未来迁移保留清晰模块边界。
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
- observer 用 IMU 和共同轮速融合 `DS`，再积分得到 `S`。system 状态机集中导出 `DISABLED/GROUND/CONTACT_TRANSIENT/AIRBORNE` 观测 context；observer 自己持有 `0.5 s` 启动迟滞，并把 context 解释为跳过、正常 NIS、受限重捕获或强制拒绝。KF 不读取 support 状态或散落的 enable bool。GROUND 下的低速重捕获要求轮速不超过 `0.5 m/s`、单帧变化处于 `25 m/s^2` 包络并稳定 `100 ms`，随后最多以 `2 m/s^2` 拉近预测；CONTACT_TRANSIENT 仍允许正常 NIS 恢复，但禁止该强制拉近。原 `NIS <= 9` 和 `20 ms` 恢复持续时间保留，降级不重置状态或 bias。这些阈值仍需实车标定。
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
- 差分权重从 `1920/32` 降到 `960/16` 后，正式基线不变；`4*pi @ 15 rad/s^2` 的最大腿差分约从 `0.94 deg` 增至 `1.26 deg`，仍双轮接触且无饱和。完整扫描和历史调参记录见 `docs/notes/controller-experiment-log.md`。

## 仿真与实验工具

- C++ 仿真层由 `MujocoPlant`、`MujocoAdapter`、`SimulationRunner`、`MujocoViewer`、`SimulationUi` 和场景编排组成。GUI 与 performance benchmark 共用 `PerformanceScenario`。
- GUI 侧栏显示状态机、state/reference、roll、云台反馈、腿运动学和限幅前后输出；`--trace <csv>` 可写交互式 1 kHz trace。
- `--keyboard` 会在出生点前方显示两处场地：右侧为三角形 `15 deg` 坡接 `2.0 x 2.0 m、200 mm` 高平台，左侧为独立的 `860 mm` 宽、`17 deg` 三角坡，坡顶高 `350 mm` 且不接平台；这些几何默认埋藏，不参与 benchmark 或非 keyboard 仿真。
- 独立 `mujoco_ramp_climb_recovery` 使用宽 `4 m`、高 `350 mm` 的 `17 deg` 三角坡和 `2.0 m/s` 前进命令。未提前伸腿时，`base_link` 在轮轴约 `x=1.453 m` 处撞坡；移除 fast-air 后，真实接触诊断仍会在碰撞约 `160 ms` 后进入 AIRBORNE，状态机可以正常恢复，最大 pitch 约 `10.72 deg`。默认 `0.18 m` 腿长仍会因净空不足物理卡坡，程序不自动判断上坡或替操作手伸腿。
- 同一宽坡案例复现了撞击后 KF 自锁：旧预测与正确轮速的 `NIS` 长期超门限，导致 wheel reliability 和 forward HOLD 都无法恢复。受限重捕获后，松开命令约 `0.64 s` 恢复轮速可靠，约 `1.49 s` 进入 HOLD；最终真值、估计和轮速为 `-0.00155/-0.00034/-0.00214 m/s`，`S-ref` 误差约 `6.7 mm`，最大 pitch 约 `14.38 deg`。正式 `forward_pos_3` 抽查仍为 `t90=0.784 s`、最大 pitch `2.19 deg` 且无接触或饱和回归。
- `ramp_course` 使用宽 `4 m`、高 `350 mm` 的 `17 deg` 上坡、`2 m` 平台和镜像下坡，固定 `2.0 m/s` 并保证入口正对坡面。长期回归收缩为四例：`0.18 m` 上坡停车净空负例、`0.24 m` 上坡停车、`0.24 m` 独立下坡和 `0.24 m` 完整穿越。起立仍固定从 `0.18 m` 开始，ACTIVE 后才拉到案例工作腿长。`0.20/0.22 m` 扫描、`0.18 m` 重复下坡/穿越和坡顶倒角对照已退出注册表；倒角地形能力仍保留供隔离实验复用。
- 首轮矩阵中，只有 `0.24 m` 上坡停车组无非轮碰撞并完成，最大 pitch 约 `2.47 deg`；`0.18/0.20/0.22 m` 都在坡入口附近碰到 `base_link`，随后分别反向或姿态发散。这说明 `0.24 m` 足以通过坡脚，但不证明能通过坡顶：`0.24 m` 完整穿越在上坡到平台的尖锐过渡后碰撞并发散。独立下坡四组也都发生 `base_link` 碰撞，只有 `0.18 m` 最终停车，其余在停车阶段反向。坡顶过渡和下坡姿态是后续要单独处理的问题，不能把上坡入口净空结果外推为全程通过性。
- 矩阵输出每例独立 1 kHz trace 和总 summary，记录非轮碰撞部件/阶段、最小净空、姿态、左右轮接触时差、腾空/落地、KF 失锁恢复、HOLD 恢复及限幅前后力矩。探索性失败使用 `reversed_during_course`、`reversed_while_stopping`、`forward_progress_lost` 或 `attitude_diverged` 明确结束，不再拖到笼统的全局超时。
- 坡顶发散已定位为 fast-air 与坡面瞬态耦合，而不是单纯缺少倒角：原尖角加正式 fast-air 时，支持力短时低于 `50 N/leg` 且比力低于 `5 m/s^2`，support 在接触诊断仍为 GROUND 时进入 AIRBORNE；轮子和支持力恢复后又因没有 AIR/深度卸载证据而无法进入 LANDING_RETRACT，腿持续伸向 `0.38 m`，最终发散。相同尖角只在隔离 benchmark 禁用 fast-air 后完整通过，最大 pitch 约 `1.37 deg`，无非轮碰撞或饱和。给坡顶增加约 `0.4 m` 的短倒角可避开坡顶误触发，但正式 fast-air 仍在下坡段进入同类链路，最大 pitch 约 `51.4 deg` 后停车倒退。正式控制器没有采用 benchmark 开关或阈值改动；现有支持力、比力模长和接触迟滞不足以无损区分真实跌落与坡面短时卸载，下一步需要补充方向性垂向运动证据或重构 fast-air 候选确认语义。
- 系统消融确认 fast-air 属于性能预响应而非基本功能。关闭后五个平地运动案例完全不变；`0.24 m` 完整坡道和四个独立下坡从发散/大姿态改善为稳定完成，最大 pitch 分别约 `1.37 deg` 和 `1.18-3.26 deg`。四个生产平台跌落的 AIRBORNE 识别则由 `10-12 ms` 推迟到 `39-40 ms`，`200 mm` 正反两例新增后腿触地且峰值冲击明显增加，但四例仍全部完成恢复。为优先保证单一路径和坡面鲁棒性，fast-air 已从正式控制器及常驻 benchmark 移除；旧实现、消融数据和未来重启边界归档于 `docs/archive/fast-air/`。
- 独立 `ramp_jump` 用宽 `4 m`、高 `350 mm` 的 `17 deg` 单坡测量冲坡起飞和落地，不设性能合格线。五档 `2.0/2.25/2.5/2.75/3.0 m/s` 在坡前均稳定，保守落距依次约 `0.498/0.574/0.653/0.733/0.508 m`，五例均落地并恢复。`2.75/3.0 m/s` 已出现底盘碰坡；在 `3.0 m/s` 下，有限刚度的腿长控制让名义 `0.24 m` 腿长动态压缩至约 `0.191 m`，前底盘首次撞坡将真实水平速度由约 `2.78 m/s` 降至 `1.65 m/s`，随后才触发双轮离坡、AIRBORNE、假落地和二次碰撞。禁用底盘碰撞的非物理归因对照可恢复约 `2.98 m/s` 的坡顶水平速度及 `0.924 m` 落距，证明高速退化主因是动态净空，不是助跑或平地 LQR 性能。项目不为 `3.0 m/s` 尖锐坡脚专门增加控制复杂度：`2.5 m/s` 作为干净测量档，`2.75 m/s` 作为擦碰边界，`3.0 m/s` 作为压力案例；支持力被快速伸腿反力误触发仍需在无碰撞空中案例中单独验证。
- 独立 `jump_impulse` 首轮每腿 `140/180/220 N × 60 ms` 中，`140 N` 未离地，`180 N` 只有单轮短暂离地，但两例在卸载归零后都被支持力反解误判为 AIRBORNE；纯关节力矩支持力不能在主动卸载阶段独立区分“仍接触但不施力”和真实腾空。第二轮 `240 N × 60/90/120 ms` 全部真实离地并恢复，其中 `90 ms` 获得约 `36.1 N*s` 净冲量、`1.69 m/s` COM 起飞速度、`145 mm` 起飞后 COM 上升和 `0.332 s` 双轮离地，是首个视觉明确的小跳候选，但落地最大 pitch 已约 `7.9 deg`。`120 ms` 请求因 `0.34 m` 腿长保护在 `110 ms` 提前卸载，净冲量约 `43.6 N*s`，落地 pitch 升至约 `13.5 deg`，只作为压力案例。三档最大关节请求约 `38.2-38.5 N*m`，已接近 `40 N*m` 软件限幅，不需要也不应直接改用 `400 N/leg`。下一步设计 jump task 时必须显式处理推力卸载语义和空中腿意图，不能简单持续推力直到 support 报告 AIRBORNE。
- 实车每条腿计划安装始终受压、断电可将腿推至最长限位的气弹簧，机械端初步信息为两端约 `350 N`、中段可能更小、独立阻尼器已拆除、长度约 `20-30 cm`。安装几何和等效腿轴向力曲线尚缺，不能把 `350 N` 直接加入虚拟轴向力；若有效行程约 `0.10 m`，两腿储能量级可能达到 `40-70 J`，已足以推翻当前无气弹簧 plant 的跳跃参数结论，并会显著增加落地回弹和主动阻尼需求。`jump_impulse` 保留为隔离测量工具，但跳跃调参和正式 task 暂停，主线先回到搬运腿上台阶；恢复前应先建模气弹簧、加入腿长相关补偿并复测基础支撑和落地。
- 搬运腿上台阶不是步态或自动地形识别，而是操作手触发的 task。operator command 已增加默认 NORMAL 和 STEP_DOCK 枚举；keyboard 用 `T` 切换。ACTIVE 下首版状态链为 `INACTIVE -> PREPARE -> IMPACT_PASSIVE`：PREPARE 允许边走边将工作腿长拉到 `0.38 m`，强制机体正面朝向云台，误差超过 `5 deg` 时通过原斜坡暂时减速；IMPACT_PASSIVE 锁存并关闭两轮、腿长和腿角全部 strategy，只有系统复位退出。固定搬腿和重新平衡尚未实现。
- 正式 observer 内已加入常驻影子 `impact_observer`：直接将原始 IMU specific force 变换到航向坐标，维护 `5/10 ms` 梯形积分窗口，并输出前向/垂向速度增量、pitch-rate 增量、两腿世界角速度增量、轮速增量及轮速与 IMU 积分失配。它没有 enable 配置、碰撞 bool、状态机转移或控制输出。正样本先测得启动后约 `-3.889 deg` 的共同航向偏移，再补偿初始姿态，使碰撞瞬间的世界系航向实际达到约 `0.004/+2.011/-1.968 deg`；负样本覆盖同一 `0.38 m、2.0 m/s、3.0 m/s^2` 包线下的加速、稳态、主动停车和 `+/-0.2 rad/s` 小转向。首轮数据否定了“前向减速冲量单独作为主判据”：两组碰撞的 `5 ms` 反向前向增量没有超过小转向背景；相反，`5 ms` 腿角速度增量和轮速-IMU 失配的三组正样本最小峰值分别约为 `2.29 rad/s`、`0.457 m/s`，允许检测的负样本最大值仅约 `0.076 rad/s`、`0.025 m/s`。这只支持下一轮融合设计，不构成生产阈值，实车噪声、结构柔性和更多台阶样本仍未覆盖。
- 加速未完成时撞台阶也属于正样本。CTest 现按底盘从加速起点实际行驶 `0.3/0.6/0.9/1.2/1.5 m` 校准平台位置和启动 yaw，碰撞速度约为 `1.20-1.92 m/s`；五组最终都留在平台上但仍有残余摆动。首版 task 已使用同一 `5 ms` 窗口内腿世界角速度增量超过 `0.5 rad/s`、轮速-IMU 增量失配超过 `0.12 m/s` 并持续 `2 ms` 的融合候选；门禁为两腿进入 `0.38 m +/- 20 mm`、正面误差不超过 `5 deg`、可靠正向轮速至少 `0.3 m/s` 且 support 为 GROUND，不要求助跑距离或速度稳定。生产 `step_dock_passive` 从普通 `0.18 m` 腿长边走边伸腿，真值碰撞速度约 `1.990 m/s`，正式状态机在接触后 `2 ms` 进入零输出并留在平台上；`20 mm` 腿长包络用于覆盖位置支撑约 `12 mm` 的静载闭合误差。低速顶墙、颠簸、结构柔性和实车噪声仍未验证，因此这些仍是仿真首版参数。
- 独立 drop benchmark 保留 6 个手动抬升释放案例、2 个平台空中机理对照和 4 个生产 ACTIVE 落地案例。平台机理只比较 `200 mm、2.0 m/s、0.18 -> 0.38 m` 下的 `length_only/leg_lqr`；历史 36 例净空、速度和扰动矩阵已归档到 `docs/notes/platform-drop-exploration-archive.md`，不再常驻注册表。生产案例覆盖正反向 `200/400 mm`，速度参考跨越空中与落地保持不变。
- 当前实体平台实验首先测到的是直角边缘通过性，不能直接解释为纯空中控制能力。`0.18 m` 低速驶离时底盘或后腿会持续碰撞平台边缘，碰撞在 pitch 恶化之前发生；`0.24 m` 明显缩短碰撞，并使 200 mm 六例全部恢复，但 400 mm 的 `0.5/1.0 m/s` 仍会碰边发散，两个高度的 `2.0 m/s` 均无边缘碰撞并恢复。KF 空中误差仍需在排除边缘碰撞的独立实验中归因。
- 200 mm 正常速度下的探索实验表明，离台后将腿从 `0.18 m` 目标快速伸至 `0.38 m`，可把首次触地前机体下降从约 `171-178 mm` 降至 `19-24 mm`，触地 pitch 降至约 `0.22-0.31 deg`，触地竖直速度降至约 `0.67-0.78 m/s`。实际触地腿长约 `0.31-0.35 m`，空中关节峰值请求约 `30-31 N*m` 且没有触发 `40 N*m` 限幅；这只验证快速接地有效，触地顺应和缓慢回收尚未设计。
- `leg_lqr` 空中策略的直接目标是让虚拟腿相对世界接近竖直，而不是最小化机体 pitch；有效腿角参考包含约 `+2.42 deg` trim。按正确指标比较，`2.0 m/s` 名义触地腿角由 `length_only` 的约 `6.89/5.24 deg` 收到 `4.35/3.96 deg`，左右差由 `1.65 deg` 降到 `0.39 deg`；`2.5 m/s` 由约 `8.15/8.27 deg` 收到 `5.09/5.05 deg`。在 `+/-0.5 rad/s` 离台 pitch-rate 扰动下，LQR 也都降低最大腿角误差和左右差。它会与机体交换角动量，因此机体 pitch 可能略大；不能用机体 pitch 单独否定该策略。
- 首轮主动悬挂探索固定 `200 mm、2.0 m/s、0.18 -> 0.38 m、leg_lqr`，触地后恢复轮 LQR 和完整姿态反馈，仅比较保留支撑前馈并强撑 `0.38 m` 与五组单腿轴向阻抗。六例均恢复，阻抗候选的最大腿压缩为 `92-99 mm`，明显大于位置强撑的约 `37 mm`。峰值数据复查后不能用来选择 `K/D`：共同的约 `267.2 N` MuJoCo 法向力峰值来自触地第一个采样，此时参数尚未来得及产生差异；共同的约 `187.0 N` 估计支持力峰值则由五组均撞上 `180 N` 轴向力上限主导。请求力峰值实际分布在约 `235-322 N`，说明候选有差异但被执行力约束截平。这轮只能确认受限阻抗可以完成落地，不能确认单一最优参数或把峰值下降完全归因于 `K/D`；这些结论来自当时使用真值接触的隔离 benchmark。
- Jacobian 复核表明 `0.18-0.38 m` 腿长范围内，`40 N*m` 软件限幅在纯轴向输出时至少对应约 `263 N/leg`；实验上限已从 `180 N` 放宽到 `240 N`。同组 `200 mm` 案例仍全部恢复且没有关节饱和，`K=400/800/1200,D=80` 最大压缩约为 `83/76/73 mm`，`K=800,D=40/120` 约为 `84/73 mm`。这种约 `10 mm` 的竖直差异很难从 GUI 整机姿态辨认，而且除 `D=40` 外仍会碰到 `240 N` 上限，因此参数仍未定稿。下一轮应以压缩余量、竖直速度衰减、反弹/冲量和关节力矩余量评价少量软/中/硬候选，并跨触地强度验证，而不是继续靠视觉密扫相近参数。
- 固定触地平衡点会一直顶住长腿，不符合落地后快速回收的意图；隔离策略现改为每腿触地捕获 `L_eq=L_touch`，再以 `0.8 m/s` 将平衡点降至正常工作腿长。相同 `200 mm` 案例均恢复且没有失撑、反弹、关节饱和或其他部件触地，实际腿长约 `0.23-0.35 s` 内收至 `0.20 m` 以下，瞬时最低约 `0.154-0.170 m`。方向可行，但回收末端存在约 `10-26 mm` 动态下冲；下一步重点是末端减速和退出/切回条件，而不是继续密扫固定平衡点的 `K/D`。
- 隔离实验曾加入无扰退出：双轮接触、平衡点到达工作腿长、双腿长度速度和机体竖直速度均低于 `0.1 m/s` 并保持 `50 ms` 后，按当前轴向力反算位置 PD 等效参考，再以 `0.1 m/s` 拉回 `0.18 m`。五组在触地后约 `0.47-0.59 s` 切回 `POSITION_SUPPORT`，切换首采样支持力变化小于 `1 N`，随后 `50 ms` 最大关节力矩约 `12.5-13.3 N*m`，未产生二次冲击。这套真值接触编排用于探索阶段，随后已由 ACTIVE 接触状态机和生产支持力诊断取代。
- support phase 接管前先以影子方式验证：只等待单腿完整诊断时离地/触地延迟约为 `55/17 ms`；增加“两腿滤波支持力 `<50 N`、IMU 比力范数 `<5 m/s^2`、持续 `5 ms`”的快速离地融合，以及深度卸载后支持力回升持续 `3 ms` 的触地确认后，名义案例延迟降至约 `33/4-5 ms` 且阶段不抖动。
- 上述延迟只说明当前理想平台跌落中融合判据能及时跟随 MuJoCo 真值，不能据此结束离地检测设计。斜坡、平台边缘逐轮卸载、短时失重、颠簸和强加减速可能产生相似的支持力/比力组合；在这些工况完成误报、漏报和状态抖动验证前，阈值、持续时间和融合语义均视为未定稿。
- 首版 support phase 已从影子诊断升级为正式 ACTIVE 请求：AIRBORNE 关闭轮输出、伸腿至 `0.38 m` 并只保留腿角 LQR；LANDING_RETRACT 恢复轮和完整姿态 LQR，以 `K=800 N/m,D=80 N*s/m`、`0-240 N`、`3000 N/s` 的逐腿轴向阻抗和 `0.8 m/s` 移动平衡点快速收腿；GROUND_RECOVER 反算位置 PD 等效参考并以 `0.1 m/s` 回到普通支撑。support 不覆盖整份命令，最终合成仍由 motion ACTIVE 完成。
- `platform_drop_200mm_l0p18_v2p0_leg_lqr_landing_controller` 不使用 MuJoCo 接触真值或 `step_with_control_transform()` 驱动控制。实际边缘速度约 `1.94 m/s`，离台后约 `10 ms` 进入 AIRBORNE，触地后约 `8 ms` 进入 LANDING_RETRACT，约 `0.457/0.618 s` 进入 RECOVER/回到 GROUND；无发散、反弹、其他接触或关节饱和。正反向 `200/400 mm` 四例随后均完成且恢复，正式直线、偏航和八字也仍全程 GROUND；上述阈值和落地参数仍只经过理想仿真，未覆盖实车噪声、颠簸、结构柔性或不平地面。
- 生产接管验证完成后，benchmark 侧重复的 C++ 落地悬挂控制器、五组 `K/D` 扫描和真值驱动回收逻辑已删除。当前仅保留 `length_only/leg_lqr` 空中机理对照，以及正反向 `200/400 mm` 四个完全由生产 ACTIVE 控制器驱动的落地案例；CLI 入口为 `--active-landing`。
- `tools/experiments/run_experiment.py` 从 TOML 生成隔离 schedule、构建和案例结果；`tools/experiments/plot_trajectory.py` 生成无额外依赖的轨迹 SVG。
- 本地 MuJoCo、GLFW、ImGui 可置于被忽略的 `third_party/`，通过 `MUJOCO_ROOT`、`FETCHCONTENT_SOURCE_DIR_GLFW`、`IMGUI_ROOT` 显式指定；详细构建命令见根 README。

## 仍需记住的未决事项

- ACTIVE 接管首采样曾出现约 `9.7 deg` pitch 瞬态，发生在工作腿长拉升前，应作为独立起立接管问题处理。
- 当前周期仍按 `controller_update(observer/support force) -> controller_calculate(support phase)` 执行，因此 observation context 使用周期开始时的 support 状态。新进入 AIRBORNE 后的立即 reject 能把轮速标为不可靠，却不能撤销本周期已经发生的 KF 校正；这额外的一周期延迟与检测判据本身的确认延迟必须分开看。未来应考虑拆成“基础运动学与支持力 -> support phase -> observation context -> KF -> 其余状态机与控制律”，本轮不重排周期。
- 轮速被拒绝期间没有绝对位置观测，恢复后不能修正累计的 `S` 漂移；停车安全门槛使用速度与可靠性，进入 HOLD 时重新捕获参考，不声称历史绝对位置可恢复。
- 上坡腿长变化仍是操作手必须明确选择的工作姿态，不由程序自动识别坡面。当前严格仿真中 `0.24 m` 已能无碰撞通过坡脚；移除 fast-air 后，原尖角坡顶和四个下坡案例均能稳定完成，倒角不再承担规避状态机误触发的职责。
- 高横向加速度下的内侧轮卸载仍是物理现象；未来涉及高速转弯、路径控制或 roll 策略时，需要接触观测和实车等效输入，不能只提高 PD/LQR 权重。
- 当前 plant 只有力矩限幅，没有电机速度、功率、热和电池模型。涉及性能上限的新功能必须先明确是否需要补这些约束。
- Windows 构建由用户侧复核；本地 Linux 是当前自动验证环境。

## 主要材料与恢复顺序

1. 本文件：当前事实和边界。
2. `docs/notes/active-motion-design.md`：ACTIVE 运行期架构、模块所有权、交叉约束和重构顺序。
3. `docs/notes/controller-experiment-log.md`：2026-08-10 起的逐轮实验、失败归因和完整数值；更早记录按时间放在 `docs/archive/experiments/`。
4. `docs/notes/lqr-validation.md`：当前模型、Q/R 和调度验证。
5. `docs/notes/platform-drop-exploration-archive.md`：实体平台、空中伸腿和早期悬挂探索归档。
6. `docs/notes/hardware-bringup.md`：实车逐级部署。
7. `models/MJCF/COD-2026RoboMaster-Balance.xml`：当前 plant。
8. `references/rm2026cb-balance-chassis/`、`references/matlab_scripts/lqr.m` 和 `references/SJTU balance control/WBR_modeling.html`：实车与理论来源。
