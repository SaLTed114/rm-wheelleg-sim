# ACTIVE 运行期架构设计

更新日期：2026-08-12

本文记录 ACTIVE 内部的职责划分、数据流和状态机边界。当前代码已实现 NORMAL 下的纵向 `HOLD/VELOCITY`、云台跟随、front/rear 映射、工作腿长规划和首版 support phase；这些逻辑仍由 `bc_motion_t` 内的 ACTIVE 分支统一合成。SPIN 和独立的 ACTIVE 类型尚未实现。

## 为什么 ACTIVE 不能再作为一个单一动作

顶层 motion 状态机描述整机从失能到能够持续运动的生命周期：

```text
IDLE -> SELF_RIGHTING -> LEG_POSITIONING -> BALANCE_ENGAGING -> ACTIVE
```

前四个状态回答“机器人是否已经准备好运行”，ACTIVE 则表示“已经进入持续运行期”。进入 ACTIVE 后，机器人还需要同时处理任务选择、纵向运动、偏航跟随、腿长规划、接触条件和以后可能增加的落地恢复。把这些全部继续写在 `BC_MOTION_ACTIVE` 的一个 action 分支中，会把长期运行的协调逻辑膨胀成隐式状态机。

这些概念也不是同一种状态，不能简单画成若干平权且正交的枚举：

- `NORMAL/SPIN` 是任务语义，回答“操作手希望整机做什么”；
- `HOLD/VELOCITY` 是纵向反馈模式，回答纵向轴当前锁位置还是跟速度；
- `GROUND/AIRBORNE/LANDING` 是物理支撑阶段，回答哪些观测和执行器当前可信、可用；
- forward/yaw/leg-length ramp 是连续参考规划器，不应为了形式统一而包装成状态机。

它们既有并行关系，也有交叉约束。例如 NORMAL 可以同时直行和转弯；SPIN 进入前需要纵向停车；AIRBORNE 会使轮速观测和轮输出失去原有语义；LANDING 会暂时改变腿长策略，但不应丢掉操作手持续给出的速度和朝向意图。因此不能建立 `NORMAL_GROUND_HOLD` 之类的笛卡尔积状态，也不能让多个模块按调用顺序覆盖同一个 `bc_control_command_t` 字段。

## 目标分层

ACTIVE 应当是运行期协调器，而不是一种具体运动：

```text
motion lifecycle
└── ACTIVE coordinator
    ├── task sequencer       NORMAL / SPIN phases / future explicit tasks
    ├── longitudinal         HOLD / VELOCITY + forward reference
    ├── yaw                  heading follow / rate control + yaw reference
    ├── leg-length planner   normal working length and task request
    └── support phase        GROUND / AIRBORNE / LANDING
```

这张图只表示代码所有权，不表示五个模块平权。它们在控制周期内应按下面的数据流工作：

```text
operator command + gimbal feedback
                |
                v
        task sequencer produces intent
                |
                v
 longitudinal / yaw / leg planners produce continuous references
                |
                v
 support phase applies physically valid control capabilities
                |
                v
 ACTIVE coordinator assembles one bc_control_command_t
```

任务层产生意图，轴模块负责参考连续性，support phase 施加物理条件约束，ACTIVE 协调器是最终控制命令的唯一组装者。任何子模块都不应先生成一份完整命令，再由另一个模块事后“修补”或清零字段。

## 各模块的职责和所有权

| 模块 | 拥有的状态或连续量 | 可以决定 | 不应该决定 |
| --- | --- | --- | --- |
| motion lifecycle | 起立、接管、ACTIVE 的进入与退出 | 是否允许运行 ACTIVE | ACTIVE 内各轴的具体策略 |
| task sequencer | NORMAL、SPIN 内部阶段及未来显式任务 | 各轴期望、任务许可和阶段条件 | 接触真值、LQR 输出、执行器限幅 |
| longitudinal | `HOLD/VELOCITY`、`S/DS` 参考、速度斜坡 | 纵向位置或速度反馈模式 | 偏航模式、腿部接触阶段 |
| yaw | front/rear 映射、heading/rate 参考 | `PSI/DPSI` 参考及相应反馈模式 | 纵向停车判定、腿长策略 |
| leg-length planner | 工作腿长参考和变化速率 | 连续腿长目标 | 是否允许轮输出、是否已经触地 |
| support phase | 支持力判定、离地和落地阶段 | 当前可用的观测、执行器和腿部控制策略 | 操作手任务意图、正常参考的长期目标 |
| ACTIVE coordinator | 模块调用顺序和冲突处理 | 最终 `bc_control_command_t` | 在内部复制一套轴级状态机 |

这里的“纵向单独拉出来”不是增加一个与转弯互斥的 `STRAIGHT` 任务。更准确的模块名是 longitudinal：它与 yaw 可以同时工作，因而自然覆盖站立、直行、原地转向和边走边转。

## 交叉关系必须显式处理

模块之间允许请求和约束，不允许直接篡改彼此的内部状态。第一版至少需要明确以下关系。

### Task 与轴模块

NORMAL 将操作手的前进速度和云台反馈分别交给 longitudinal 与 yaw。SPIN 是整车任务序列：它可以请求 longitudinal 减速至 `HOLD`，等待纵向模块报告停稳，再请求 yaw 进入高速 rate control；退出时也由 SPIN 序列等待偏航降速后恢复 NORMAL heading follow。任务层不直接写 `S/DS/PSI/DPSI` 的具体数值，也不伪造纵向模块的状态。

普通运动不应拆成“直行、转弯、边走边转”三个互斥任务。它们只是 longitudinal 和 yaw 两个轴同时得到不同参考的组合。

### Support 与任务意图

support phase 是运行条件，不是操作手任务。意外离地不应把 `task=NORMAL` 改成一个名为 AIRBORNE 的任务，也不应丢弃操作手仍在发送的速度和朝向意图。参考规划器是否继续推进必须逐项定义；目前平台实验支持“纵向和偏航意图持续、轮输出在空中关闭”，但正式语义仍需由状态机实验确认。

support phase 对最终控制能力拥有更高约束权。例如任务请求轮 LQR，而物理阶段为 AIRBORNE 时，最终命令必须关闭轮输出；这应当由 ACTIVE 合成规则表达成“任务请求与可用能力的交集”，而不是 support 模块在末尾覆盖任务生成的命令。

### Support 与 longitudinal/KF

AIRBORNE 时轮速不再代表机体平移速度，因此 support phase 必须向 observer/速度估计链提供“轮速观测不可用”的可靠门控，并向 longitudinal 提供当前无法依靠轮地作用执行速度控制的事实。longitudinal 仍需保存参考连续性，但不能用空转轮速误判停车并切入 `HOLD`。

这一门控横跨当前 `update -> calculate` 边界：支持力在 `bc_control_core_update()` 中计算，而状态机在 `bc_controller_calculate()` 中运行。正式接入时必须明确本周期判定和下一周期生效的时序，不能在仿真层读取 MuJoCo 真值绕过这个问题。

### Support 与腿长规划

AIRBORNE 可以临时请求快速伸腿，LANDING 可以临时请求顺应、吸能和缓慢回收，但正常工作腿长仍属于 ACTIVE 的长期目标。临时支撑策略结束后，腿长规划器应从当前实际/参考状态连续恢复，不能突然跳回正常目标。

正式 controller case 已验证 `0.18 m -> 0.38 m` 的空中快速伸腿、腿角 LQR、触地轴向阻抗、快速收腿和无扰恢复。参数仍只有理想仿名义 `200 mm` 工况依据，尤其 `400 mm` 和实车噪声尚未验证，因此它是首版仿真策略而非实车定稿。

空中伸腿的作用应理解为主动悬挂的准备阶段：尽早建立轮地接触，并预留随后可压缩的腿部行程。主动悬挂真正发生在触地以后；若继续用高刚度位置环强追最大腿长，会主动抵抗压缩，不能代表正式 LANDING_CATCH。正常工作腿长是长期目标，空中伸展目标、触地捕获长度和恢复目标是不同量，不能共用一个 reference 并互相覆盖。

山东理工大学控制器报告提供了有价值的控制对象划分：以机体高度/高度速度和 Roll/Roll rate 为状态，以左右腿支持力为输入，并显式限制支持力及其变化率。这支持 LANDING_CATCH 采用轴向力/阻抗而非简单锁腿长。不过报告没有给出完整接触状态机、腾空腿零接触力约束或可复现的落地参数；其中 MPC、LESO 和具体数值不能直接移植到当前十状态 LQR。

## 控制命令的合成规则

最终实现应遵守“一类字段一个所有者，协调器统一合成”的规则：

1. task sequencer 输出任务意图和阶段请求，例如目标前进速度、NORMAL heading follow 或 SPIN rate control；
2. longitudinal、yaw 和 leg-length planner 更新自己的状态与连续参考，输出轴级请求；
3. support phase 根据生产支持力估计输出能力约束和支撑策略；
4. ACTIVE coordinator 按明确规则合成 wheel strategy、leg strategy、state reference 和 disabled feedback mask；
5. control core 只执行合成后的命令、计算 LQR/PD 并限幅，不反向改变任务状态。

特别禁止以下写法：

- 多个模块接收同一个 `bc_control_command_t *`，依靠调用先后覆盖 strategy、target 或 mask；
- 用十几个 `disabled_state_feedback` 位掩码在各处模拟没有名字的阶段；
- 为测试方便把 MuJoCo 接触真值、case enable 或 bypass 开关放入生产 controller config；
- 为避免模块协调而枚举所有 task、support、longitudinal、yaw 的组合状态。

现有 `bc_forward_mode_update(..., bc_control_command_t *output)` 会直接修改 `S` feedback mask，它在当前规模下可工作，但不符合目标数据流。重构时 longitudinal 应输出自己的模式/请求，由 ACTIVE 统一决定 mask。

## 当前 NORMAL 语义

NORMAL 已实现，重构必须保持现有行为和性能基线。

纵向使用 `HOLD/VELOCITY`：ACTIVE 从 HOLD 开始；只有收到非零运动请求才进入 VELOCITY，并在该模式关闭 `S`、保留 `DS`。命令与参考归零、融合速度和原始轮速均低于阈值、轮速可靠并持续满足时间门槛后回到 HOLD，重新捕获停车位置。

偏航的语义是底盘跟随云台朝向，不是操作手直接命令底盘角速度。motion 实时比较底盘正方向和反方向对齐云台所需的转角，以 `5 deg` 滞回选择 front/rear；选择 rear 时同步反转纵向命令。NORMAL 无论 HOLD 或 VELOCITY 都使用 `PSI/DPSI`，因此原地转向和边走边转不需要新增任务状态。

当前 operator command 只有：

```c
uint8_t system_enabled;
uint8_t balance_restart;
float forward_velocity;
```

`forward_velocity` 使用云台正方向坐标。云台 YAW 电机的相对角和相对转速属于 sensor feedback；下板负责 front/rear 选择、纵向符号转换以及 heading reference。仿真虚拟云台必须保持与未来真实 YAW 电机反馈相同的边界。

## SPIN 的预留语义

SPIN 尚未实现。它是协调纵向、偏航和命令许可的整车任务，不是 yaw reference 内的一个大角速度分支。当前候选序列为：

```text
NORMAL
  -> SPIN_BRAKING_FORWARD
  -> SPINNING
  -> SPIN_BRAKING_YAW
  -> NORMAL
```

进入时请求 longitudinal 减速并进入 HOLD，之后才允许高速自旋；SPINNING 中禁止纵向运动请求，关闭 `PSI` 而保留 `DPSI`；退出时先做 rate-only 制动，实际偏航速度降至重入门槛后才恢复 NORMAL 云台跟随。具体停稳阈值、阶段重叠、加速度和重入速度尚未确定。

如果 SPIN 中发生离地，task 与 support 的优先级必须有明确定义。当前没有实验依据决定继续自旋、制动还是直接退出 SPIN，因此第一版架构只保留显式冲突处理入口，不预写行为。

## Support phase 的当前边界

候选支撑阶段为：

```text
GROUND -> AIRBORNE -> LANDING -> GROUND
             ^           |
             +-----------+  rebound
```

该结构属于 ACTIVE 内部，首版已进入纯 C 生产状态机。目前可以确认的边界是：

- 生产 `control/support_force` 的单腿诊断、滤波支持力和 IMU 比力范数已进入 state-machine input；MuJoCo 真值不参与 controller case 的阶段切换；
- benchmark 的旧隔离候选仍保留用于历史对照，但新增 `landing_controller` case 全程只调用普通 `runner.step()`，不再通过 `step_with_control_transform()` 修改命令；
- AIRBORNE 候选策略是关闭轮输出、快速伸腿并只保留腿角 LQR；腿角目标应使用真正世界竖直还是保留地面 `+2.42 deg` trim 仍未决定；
- LANDING 不能等同于第一次接触后立即恢复完整地面 LQR。左右轮异步触地、反弹、顺应吸能和腿长连续回收仍需要实验；
- 支持力进入/退出条件需要明确使用单腿、双腿还是合力语义，当前每侧检测结果不能直接等同于整机阶段。

### 2026-08-12 主动悬挂首轮实验

首轮实验仍使用 MuJoCo 单轮接触真值隔离触地控制律，不是正式 support phase。固定场景为 `200 mm、2.0 m/s、0.18 m -> 0.38 m、leg_lqr`；空中快速伸腿，触地后恢复轮 LQR 和完整姿态反馈，只比较腿长轴：基线继续位置保持 `0.38 m`，五组候选使用

```text
F = 76.204 + K * (L_touch - L) - D * dL + F_roll
```

每腿轴向力限制为 `0-180 N`，变化率限制为 `3000 N/s`。探索点为 `K={400,800,1200} N/m, D=80 N*s/m` 和 `K=800 N/m, D={40,120} N*s/m`。

六例均完成、无姿态发散、无反弹、无关节饱和。五个阻抗候选允许约 `92-99 mm` 的腿部压缩，而位置保持 `0.38 m` 的基线仅压缩约 `37 mm`，说明两类策略确实产生了不同的落地运动。但首轮峰值指标不能分辨五组 `K/D`：共同的约 `267.2 N` MuJoCo 法向力峰值发生在触地第一个采样，早于参数产生可见差异；约 `187.0 N` 的估计支持力峰值则出现在候选已经达到共同 `180 N` 轴向力上限之后。各候选请求力峰值实际约为 `235-322 N`，只是被实际力上限截平。`K=800,D=120` 的连续稳定确认时间为 `0.567 s`，但单个完成时间不足以据此宣布它最优。

因此首轮只能证明“受 `180 N / 3000 N/s` 约束的轴向阻抗可以完成该落地”，以及它相对位置强撑提供了更多压缩行程；不能把共同峰值当成 `K/D` 的效果，也不能据此定稿参数。后续比较应观察触地后前 `100-300 ms` 的请求力、实际力、压缩行程、竖直速度衰减和姿态，并记录进入力上限的时间占比。若这些局部响应仍被限幅主导，应重新设计扫描范围或控制律，而不是直接提高上限来制造曲线差异。

随后按当前腿部 Jacobian 复核执行能力：在 `0.18-0.38 m` 腿长范围内，只施加轴向力时，`40 N*m` 软件关节限幅至少对应约 `263 N/leg`，`54 N*m` 电机上限至少对应约 `356 N/leg`。实验轴向力上限因此由 `180 N` 放宽到 `240 N`；最不利腿长下约占用 `36.4 N*m`，仍保留少量软件限幅余量，且本次 MuJoCo 案例没有出现关节饱和。

同组六例在 `240 N` 下均恢复。五组候选的最大压缩开始分化：`K=400/800/1200,D=80` 分别约为 `83/76/73 mm`，`K=800,D=40/120` 分别约为 `84/73 mm`。差异只有约 `10 mm`，主要存在于触地后数百毫秒的竖直运动，因此 GUI 中的整机姿态看起来接近是正常的。除 `K=800,D=40` 请求峰值约 `231 N` 未明显撞上力上限外，其余候选请求约 `266-319 N`，仍被 `240 N` 截断；当前扫描依然不足以凭视觉或单一稳定时间选择参数。

后续调参应先明确落地悬挂的取舍，而不是继续密集扫相邻 K/D：压缩行程必须保留触底余量，竖直速度应平滑衰减，向上反弹和轮地冲量应受控，同时关节力矩需要保留腿角控制余量。在一组落差和触地速度上得到“最优点”没有意义；至少需要用不同落差或触地竖直速度验证软、中、硬三档候选，再缩小范围。

进一步复核控制意图后，固定 `L_touch` 的阻抗也不是完整落地动作：它会持续试图恢复触地时的长腿，而实际需要在承接冲击的同时快速收回正常工作腿长。隔离实验因此改用以 `0.8 m/s` 移动的平衡点：每条腿首次触地时令 `L_eq=L_touch`，随后将 `L_eq` 线性降至场景工作腿长 `0.18 m`，轴向力仍使用

```text
F = 76.204 + K * (L_eq - L) - D * dL + F_roll
```

在同一 `200 mm、2.0 m/s` 场景中，五组候选仍全部恢复，且没有反弹、关节饱和或非轮部件触地。实际腿长在触地后约 `0.23-0.35 s` 内低于 `0.20 m`，瞬时最低约 `0.154-0.170 m`，最终约为 `0.169-0.175 m`；后者接近该模型正常 `0.18 m` 目标下的承载长度。整个过程中单腿应用力没有降到零，说明这次快速撤回平衡点没有造成完全失撑。当前最明显的问题转为回收末端约 `10-26 mm` 的动态下冲，不应继续加快回收。

隔离 benchmark 还验证了向正常腿长控制退出。双轮持续接触、两侧 `L_eq` 已到工作腿长、两腿长度速度均小于 `0.1 m/s`、机体竖直速度小于 `0.1 m/s` 并连续保持 `50 ms` 后，策略从轴向力切换到 `POSITION_SUPPORT`。切换参考不是直接设为 `0.18 m`，而是用当时的应用轴向力、腿速和 Roll 差分力反算位置 PD 的等效参考，再以 `0.1 m/s` 拉回工作目标。五组切换发生在触地后约 `0.47-0.59 s`；切换首采样的估计支持力变化均小于 `1 N`，随后 `50 ms` 内最大关节力矩约 `12.5-13.3 N*m`，没有二次下沉、反弹、饱和或其他接触。这说明无扰退出机制在当前名义场景可行，但触地和退出条件仍使用 benchmark 真值，尚未进入正式状态机。

生产状态机先以不改变控制命令的影子 support phase 验证了 `GROUND -> AIRBORNE -> LANDING_RETRACT -> GROUND_RECOVER -> GROUND`。只等待两侧完整 AIR 诊断时，名义案例的 AIRBORNE 比真实双轮离台晚约 `55 ms`，LANDING_RETRACT 比首次真实触地晚约 `17 ms`；当前离台到首次触地只有约 `71 ms`，因而这个基础路径不足以接管空中伸腿。

影子状态随后增加了仅用于整机阶段融合的快速离地候选：两侧滤波支持力均低于 `50 N`、IMU 比力范数低于 `5 m/s^2` 并持续 `5 ms`。进入 AIRBORNE 后必须先观察到两侧均低于 `10 N` 的深度卸载或完整 AIR 诊断，随后任一侧支持力回升至 `15 N` 以上并持续 `3 ms` 才允许进入 LANDING_RETRACT；这样避免旧的 GROUND 锁存或空中估计回摆被误判为触地。

名义平台案例中，快速影子 AIRBORNE 比真实离台晚约 `33 ms`，LANDING_RETRACT 比首次真实触地晚约 `4-5 ms`，五组阻抗的 GROUND_RECOVER/GROUND 分别出现在触地后约 `0.465-0.588 s / 0.598-0.639 s`，阶段不再抖动。正式 `+/-3 m/s` 直线、`+/-1.5*pi rad/s` 偏航和 `figure_eight_open_loop` 全程保持 GROUND，未出现影子误报；加入影子状态前后的落地控制指标也逐值相同。

首版接管现已复用同一融合判据。support 模块输出结构化请求，ACTIVE 最终合成命令：AIRBORNE 关闭轮输出、腿长位置快速伸至 `0.38 m` 并只保留腿角 LQR 所需反馈；LANDING_RETRACT 恢复轮和完整姿态 LQR，每腿在支持力回升后独立捕获触地长度，以 `K=800 N/m,D=80 N*s/m`、`0-240 N` 和 `3000 N/s` 的轴向阻抗将平衡点以 `0.8 m/s` 收回工作腿长；GROUND_RECOVER 反算等效位置参考并以 `0.1 m/s` 回归普通 `POSITION_SUPPORT`。速度和偏航参考规划器在整个过程中继续推进。

新增平台 `landing_controller` 案例不使用 MuJoCo 真值驱动控制。名义 `200 mm、约 1.94 m/s` 结果为：离台后约 `10 ms` 进入 AIRBORNE，首次真实触地后约 `8 ms` 进入 LANDING_RETRACT，约 `0.457 s` 进入 GROUND_RECOVER、`0.618 s` 返回 GROUND；案例完成且恢复，无发散、反弹、非轮接触或关节饱和。空中轮速被强制标记不可靠，KF 只做 IMU 预测，落地后重新经过轮速恢复持续时间。

正式 `+/-3 m/s` 直线、`+/-1.5*pi rad/s` 偏航和 `figure_eight_open_loop` 再次全程保持 GROUND，性能数值与历史基线一致。但快速判据和 `K/D` 仍只覆盖理想仿真；未经过实车噪声、颠簸、单轮卸载、非对称落地和 `400 mm` 压力工况，不能称为最终可靠接触参数。

实验也暴露了一个边界：若触地后长期沿用空中策略、持续关闭轮输出和机体 pitch 反馈，六例都会在约 `0.17 s` 后开始明显失稳。LANDING_CATCH 不能无限期替代地面平衡能力；最终设计必须明确承接阶段如何与轮/姿态控制协同以及何时进入 RECOVER，而不是只选择一组腿部 `K/D`。

## 推荐的重构顺序

这轮重构先改变所有权，不同时引入新的控制行为：

1. 建立 ACTIVE coordinator，将现有 ACTIVE 数据和更新入口从 `bc_motion_t` 中收拢；
2. 把现有 forward mode 与 forward reference 合并为独立 longitudinal 模块，使其输出轴级请求而不是直接修改控制命令；
3. 将 NORMAL 的 front/rear 映射与 yaw reference 明确放入 yaw/normal-task 边界，并用当前 benchmark 验证行为逐值不变；
4. 将工作腿长 planner 纳入 ACTIVE，但保持当前 `0.10 m/s` 语义不变；
5. 给 state-machine input 接入生产支持力输出，再新增 support phase；
6. 用平台 benchmark 将生产 support phase 与 MuJoCo 真值对照，逐步删除仿真层的魔法接管；
7. 最后再设计 LANDING 和 SPIN，避免架构重构与新控制参数同时变化。

每一步都应保持 C core 不依赖 MuJoCo、保持 Windows 构建，并用现有 NORMAL 直线、heading、综合运动和 drop case 分别检查行为回归。

## 仍未确定

- ACTIVE coordinator 的最终 C 类型和文件命名；上述模块名表达职责，不强制一一对应源文件；
- task sequencer 第一版是否只保留隐式 NORMAL，等 SPIN 实现时再增加枚举；
- support 的整机离地/触地融合条件与本周期/下一周期时序；
- AIRBORNE 中各参考规划器继续推进、冻结或重捕获的精确定义；
- 空中腿角参考是否移除地面 trim；
- LANDING 的支撑前馈、阻尼、回收和反弹处理；
- SPIN 与 AIRBORNE 同时发生时的安全行为。
