# rm-balance-sim 项目上下文

> 用于新开发会话快速恢复当前事实、设计边界、已验证能力和未决事项。过程性实验留在实验日志，不在此重复。最近整理：2026-08-15。

## 当前状态

- 项目包含 MuJoCo 闭链 plant、纯 C 控制核心、分层状态机、状态估计、增益调度 LQR、GUI、benchmark 和 TOML 实验工具。
- NORMAL 平地纵向、heading、稳态转弯和开环八字已有稳定基线；ACTIVE 已接管离地、空中伸腿、落地回收和恢复。
- 操作手触发的 `STEP_DOCK` 已完整迁入控制核心，生产案例不再使用 C++ control transform。当前主线是继续验证台阶任务的实车适用性，而不是重新扫描平地 LQR 或搬腿轨迹。
- 跳跃任务暂停。现有 `jump_impulse` 只用于隔离轴向力、冲量和 support 接管；气弹簧模型完成前，当前无气弹簧 plant 的跳跃参数不能升级为实车结论。

## 不可破坏的边界

- 控制核心使用纯 C 和 SI 单位，不依赖 MuJoCo、GLFW、CAN、HAL 或线程实现；仿真与未来嵌入式工程通过 adapter 接入同一核心。
- 产品后续计划使用 C++，但当前功能开发不夹带 C 到 C++ 迁移。模块所有权和纯 C 边界仍需保持清楚。
- MuJoCo 接触真值只用于 benchmark 断言，不能参与生产状态切换、参考更新或命令改写。
- task 和 support 只输出结构化请求；`motion` 是 ACTIVE 内唯一的参考管理者和 `bc_control_command_t` 组装者。不要依靠调用顺序让多个模块覆盖整份命令。
- 不根据闭环表现猜填未知 plant 参数。几何、质量、惯量或 joint/site frame 改动后，必须重验闭链、符号、运动学/Jacobian、标定、LQR 调度和性能基线。
- 回归要分别检查响应、姿态、腿状态、轮地接触、非轮触地、净空和限幅前后输出，不能用单个 RMSE 或 pass/fail 掩盖物理代价。
- 开发和验收使用 Windows Release；Debug 不作为性能或 MuJoCo 时序结论依据。

## Plant 与坐标

- 当前模型为 `models/MJCF/COD-2026RoboMaster-Balance.xml`。每侧前后支链通过 equality `connect` 形成真实闭链，主动输入为左右前髋、后髋和轮轴。
- 控制运动学用两个主动髋角计算虚拟腿长度 `L`、相对车体腿角 `theta` 和解析 Jacobian，并通过 `[tau_front,tau_rear]^T = J^T[F,Tp]^T` 映射输出；`l1=0.215 m`、`l2=0.254 m`。
- 坐标为 FLU：车头 `+X`、左侧 `+Y`、上方 `+Z`，正 pitch 为车头下压，正 yaw 为左转。XML 左右命名与物理侧相反，adapter 固定映射 `BC_L -> XML Right_*`、`BC_R -> XML Left_*`。
- adapter 零偏在 `0.18-0.21 m` 站立区拟合，最大长度/角度误差约 `1.594 mm/0.328 deg`；修改 joint/site frame 后必须重新标定。
- 总质量 `21.450 kg`；base pitch/yaw 惯量为 `0.367565/0.413477 kg*m^2`，LQR 使用全装配体在 `0.18 m` 的 yaw 惯量 `0.588029054 kg*m^2`。
- 物理右腿闭链有约 `1.95 mm` 固定横向错位，机体复杂 mesh 仍直接参与碰撞，物理与控制周期均为 `1 ms`。台阶平台对 `base_link` 使用 `0.001` 的专用滑动摩擦近似导轮；推广到其他平面尚未实施。

## 运行时架构

正式数据流：

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

- `bc_controller_t` 是 facade；`update` 更新运动学和 observer，`calculate` 推进状态机、参考和控制律，`execute` 只做 system 硬门控、限幅和最终输出。
- `bc_controller_snapshot_t` 是调用者持有的诊断快照，供 GUI、benchmark、测试和未来遥测使用，不是控制器内部状态副本或通信协议。
- C core 的执行器接口为左右腿前/后关节力矩和左右轮力矩。轮/关节软件限幅为 `6.32/40 N*m`。

状态机分层：

```text
system:  OFF / ON / FAULT
motion:  IDLE / SELF_RIGHTING / LEG_POSITIONING / BALANCE_ENGAGING / ACTIVE
forward: IDLE / HOLD / VELOCITY
support: GROUND / AIRBORNE / LANDING_RETRACT / GROUND_RECOVER
step:    INACTIVE / PREPARE / IMPACT_PASSIVE / TRANSFER / TRANSFER_HOLD
         / RECOVER / RECOVER_LOCK / COMPLETE / RECOVERY_FAILED
```

- ACTIVE 从 forward `HOLD` 开始；运动时进入 `VELOCITY` 并关闭 `S`、保留 `DS`。命令和参考归零、融合速度与原始轮速均低于 `0.05 m/s`、轮速可靠并稳定后，回到 HOLD 并捕获停车位置。
- heading follow 与 forward mode 独立，始终使用 `PSI/DPSI`。motion 以 `5 deg` 滞回选择 chassis front/rear，选择 rear 时同步反转云台坐标下的纵向命令。
- 起立固定在 `0.18 m` 完成，ACTIVE 后才按默认 `0.40 m/s` 腿长斜坡进入配置工作点。`0.38 m` 是需要降额的特殊工况。
- support 从 GROUND 进入 AIRBORNE 只依据双腿接触诊断均为 AIR；旧 fast-air 已删除。AIRBORNE 关闭轮端并伸腿至 `0.38 m`；LANDING_RETRACT 使用逐腿轴向阻抗回收；GROUND_RECOVER 无扰切回普通位置支撑。最终命令仍由 motion 合成。

## STEP_DOCK 生产语义

STEP 是操作手明确触发的 task，不是步态或自动地形识别。keyboard 用 `T` 启动或取消：

```text
INACTIVE -> PREPARE -> IMPACT_PASSIVE -> TRANSFER -> TRANSFER_HOLD
         -> RECOVER -> RECOVER_LOCK -> COMPLETE -> INACTIVE(disarmed)
                       \-> RECOVERY_FAILED
```

- PREPARE 将工作腿长设为 `0.38 m`，强制使用机体正面；对齐误差超过 `5 deg` 时通过原 forward 斜坡减速。两腿进入 `0.38 m +/- 20 mm`、正面已对齐、可靠正向轮速至少 `0.3 m/s` 且 support 为 GROUND 后才武装碰撞检测。
- 碰撞候选使用 `5 ms` observer 窗口：最大腿世界角速度增量超过 `0.5 rad/s` 且轮速与 IMU 积分失配超过 `0.12 m/s`，持续 `2 ms` 后确认。这些是仿真首版阈值，尚未覆盖实车噪声、结构柔性、低速顶墙或颠簸。
- IMPACT_PASSIVE 全部 disabled 一个 `1 ms` 周期。TRANSFER 从碰撞实测状态线性经过 `0.08 s: 0.24 m/-50 deg`、`0.16 s: 0.16 m/-30 deg`、`0.34 s: 0.17 m/-125 deg`、`0.50 s: 0.18 m/-90 deg`，再 HOLD `0.10 s`；轮端关闭，腿使用位置控制。
- RECOVER 使用普通 `0.18 m` 支撑、腿角 LQR 和轮 LQR，但暂时屏蔽 `S/PSI`，允许轮轴移动到质心下方。满足姿态、速度、腿状态、双侧 GROUND 和轮速可靠门槛 `0.05 s` 后进入 RECOVER_LOCK，才原子捕获当前 `S/PSI`、清零 `DS_ref/DPSI_ref` 并恢复完整反馈；再稳定 `0.05 s` 后完成。
- pitch/pitch-rate 交还容差为 `5 deg/0.5 rad/s`，腿世界角速度容差为 `0.15 rad/s`。RECOVER 两段累计超过 `4 s` 进入锁存失败，只能系统复位。
- COMPLETE 保留一个周期后回到普通 ACTIVE。核心要求看到一次 NORMAL 才重新武装；keyboard 会在完成后自动清除 `T` 锁存并输出该次 NORMAL，下一次只需再按一次 `T`。

Release `step_dock_complete` 的当前基线：碰撞速度约 `1.99 m/s`，HOLD 双轮台面接触率 `100%`，最小轮轴越边余量约 `94 mm`，最大 pitch 约 `18.2 deg`；RECOVER/LOCK 约 `0.639/0.051 s`，最终 pitch/pitch-rate 约 `3.34 deg/-0.161 rad/s`。最大关节请求约 `27.5 N*m`，未饱和；RECOVER 中轮请求短暂超过 `6.32 N*m` 并被钳制，属于允许的平衡冷启动行为，不再增加软接管阶段。

## 参考、估计与控制

- forward reference 上限 `3 m/s`、默认斜坡 `5 m/s^2`；yaw 上限 `+/-1.5*pi rad/s`、加速度 `10 rad/s^2`。yaw 加速度前馈比例为 `0.9`。
- observer 用 IMU 和共同轮速融合 `DS` 并积分 `S`。system 集中导出 `DISABLED/GROUND/CONTACT_TRANSIENT/AIRBORNE` observation context；KF 根据 context 跳过、正常 NIS、受限重捕获或强制拒绝，不直接读取 support 状态。
- 碰撞后低速受限重捕获已解决轮速可靠性永久自锁；轮速拒绝期间没有绝对位置观测，累计 `S` 漂移只能在重新进入 HOLD 时通过参考重捕获隔离，不能恢复历史绝对位置。
- LQR 状态为 `[s,ds,psi,dpsi,theta_l,dtheta_l,theta_r,dtheta_r,theta_b,dtheta_b]`，输入为 `[T_wheel_l,T_wheel_r,Tp_leg_l,Tp_leg_r]`，调度范围 `0.160-0.390 m`。
- 正式 `Q=[90,260,40,15,60,1,60,1,900,120]`、`R=[1.6,1.6,0.7,0.7]`；腿角共同/差分权重为 `60/960`，腿角速度为 `1/16`。
- roll 不进入十维 LQR，由 `Kp=800 N/rad`、`Kd=60 N/(rad/s)`、限幅 `200 N` 的差分轴向力控制。普通支撑前馈为每腿 `76.204 N`，共同腿角 trim 为 `+2.42 deg`。

## 已验证能力与常驻案例

- 正式 performance registry：`+/-3 m/s @ 5 m/s^2` 直线、`+/-1.5*pi rad/s @ 10 rad/s^2` heading，以及显式运行的 `figure_eight_open_loop`。`0.18 m` 直线 `t90` 约 `0.78 s`、最大 pitch 约 `2.2-2.4 deg`，均正常停车、双轮接触且无饱和。
- 转弯粗包络表明 `0.5*pi rad/s` 在 `1-3 m/s` 稳定，`pi rad/s` 在约 `2 m/s` 仍可靠；继续提高横向加速度会物理卸载内侧轮。八字是开环运动复现，不是严格路径跟踪。
- ACTIVE 平台落地常驻正反向 `200/400 mm` 四例，均由生产 controller 驱动并完成恢复；另保留两个空中 `length_only/leg_lqr` 机理对照。历史 36 例矩阵和悬挂扫描已归档。
- `ramp_course` 常驻四例：`0.18 m` 上坡净空负例、`0.24 m` 上坡停车、`0.24 m` 下坡、`0.24 m` 完整穿越。fast-air 删除后，尖角坡顶与下坡不再触发原有发散链；`0.18 m` 坡脚净空不足仍是物理限制。
- `ramp_jump` 保留 `2.0-3.0 m/s` 五档测量曲线，不设性能合格线。`2.5 m/s` 是无底盘碰撞的干净档，`2.75 m/s` 是擦碰边界，`3.0 m/s` 是碰撞主导压力档。
- `jump_impulse` 常驻 `140 N x 60 ms` 不离地负样本、`240 N x 90 ms` 代表性起跳和 `240 N x 120 ms` 压力案例。完整六档扫描只留在实验日志。
- STEP 常驻案例只保留 `step_dock_complete`。GUI/CLI 总注册表当前为 30 个案例；preview、延迟档和探索扫描不再常驻。

## 未决事项与下一步

1. STEP 碰撞阈值、低摩擦导轮近似和固定 `200 mm` 搬腿轨迹仍只经过理想仿真。下一步应优先补实车噪声、低速顶墙、颠簸、结构柔性和不同接近速度；不要先继续压缩 RECOVER 时间或扫增益。
2. `base_link` 对其他平面的低摩擦推广仍延期。推广前要明确导轮方向性和碰撞几何，不能把台面专用 contact pair 直接复制到所有接触。
3. 气弹簧安装几何与等效腿轴向力曲线缺失。约 `350 N` 的端点信息不能直接作为虚拟轴向力；完成模型和基础支撑/落地复测前，暂停 jump task 与跳跃调参。
4. support 进入 AIRBORNE 现在等待双腿完整 AIR 诊断，真实跌落预判比已删除 fast-air 慢约 `28 ms`；低落差可能增加结构接触。未来若恢复预响应，必须以方向性垂向证据重新设计，不能复活旧的短时卸载阈值。
5. 当前周期顺序使 observation context 使用周期开始时的 support 状态；新进入 AIRBORNE 后的 reject 不能撤销当拍已经发生的 KF 校正。若要消除这一拍，应重排为“运动学/支持力 -> support -> context -> KF -> 其余状态机”，不要用阈值掩盖。
6. ACTIVE 接管首采样曾有约 `9.7 deg` pitch 瞬态；高横向加速度内侧轮卸载仍需接触观测与实车等效输入。二者都应独立处理，不要回头盲调已通过的通用 Q/R。
7. plant 只有力矩限幅，没有电机速度、功率、热和电池模型。任何新的性能上限结论都必须先说明是否受这些缺失约束影响。
8. SPIN 尚未实现；它应是协调纵向、yaw 和许可的 task，不能由普通 heading 命令幅度隐式触发。

## 恢复与验证顺序

1. 本文件：当前事实、边界和下一步。
2. `docs/notes/active-motion-design.md`：ACTIVE 所有权与 STEP 生产语义。
3. `docs/notes/controller-experiment-log.md`：当前实验基线和后续新增结果；完整过程按时间归档在 `docs/archive/experiments/`。
4. `docs/archive/validation/lqr-validation.md`：当前生成模型、Q/R 和调度验证归档。
5. `docs/archive/experiments/platform-drop-exploration.md` 与 `docs/archive/fast-air/`：已退出主线的探索归档。
6. `docs/notes/hardware-bringup.md`：实车逐级部署。
7. `models/MJCF/COD-2026RoboMaster-Balance.xml` 与 `references/`：当前 plant 和理论/实车来源。

常用 Release 验证：

```powershell
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
.\build\Release\rm_balance_sim.exe .\models\MJCF\COD-2026RoboMaster-Balance.xml --keyboard
.\build\Release\rm_balance_sim.exe .\models\MJCF\COD-2026RoboMaster-Balance.xml --case step_dock_complete
```
