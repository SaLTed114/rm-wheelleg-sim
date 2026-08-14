# ACTIVE 运行期架构设计

更新日期：2026-08-15

本文记录 ACTIVE 内部的职责、数据流、优先级和已实现任务语义。实验演化与参数扫描见 [`controller-experiment-log.md`](controller-experiment-log.md) 及其时间归档。

## ACTIVE 的定位

顶层 motion 状态机描述从失能到持续运行的生命周期：

```text
IDLE -> SELF_RIGHTING -> LEG_POSITIONING -> BALANCE_ENGAGING -> ACTIVE
```

前四个状态回答“机器人是否已准备运行”；ACTIVE 是长期运行协调器，不是一种具体动作。它需要同时协调：

- task：`NORMAL/STEP_DOCK`，未来可能增加 `SPIN`；
- longitudinal：`HOLD/VELOCITY` 与 forward reference；
- yaw：front/rear、云台 heading follow 与 yaw reference；
- leg length：正常工作点及 task 临时目标；
- support：`GROUND/AIRBORNE/LANDING_RETRACT/GROUND_RECOVER`；
- control：反馈 mask、LQR/PD 策略和唯一执行器命令。

这些概念有交叉约束，不能展开成 `NORMAL_GROUND_HOLD` 一类组合状态，也不能让多个模块依靠调用先后覆盖同一个 `bc_control_command_t`。

## 所有权

```text
operator command + gimbal feedback + observation
                         |
                         v
                task / support state
                         |
                         v
          forward / yaw / leg references
                         |
                         v
              motion ACTIVE coordinator
                         |
                         v
              one bc_control_command_t
```

| 模块 | 拥有 | 可以输出 | 不应拥有 |
| --- | --- | --- | --- |
| motion lifecycle | 起立、平衡接管、ACTIVE 进退 | 运行许可 | ACTIVE 内任务阶段 |
| STEP task | 阶段、计时、碰撞确认、搬腿轨迹、恢复门槛 | 结构化 task request | LQR、长期参考、执行器力矩 |
| forward | `HOLD/VELOCITY`、`S/DS` 参考和速度斜坡 | 纵向反馈请求 | yaw、support、task 阶段 |
| yaw | front/rear 与 `PSI/DPSI` 参考 | heading follow 请求 | 纵向停车、腿策略 |
| leg planner | 工作腿长连续参考 | 普通腿长目标 | 接触阶段、轮端能力 |
| support | 接触阶段、触地捕获和落地腿请求 | 物理能力约束与支撑策略 | 操作手意图、长期参考 |
| motion ACTIVE | 调用顺序、优先级、参考事件处理 | 最终 `bc_control_command_t` | 复制子模块内部状态机 |

MuJoCo 真值不属于任何生产模块。benchmark 只能观察真值并断言，不能用它推动上述状态。

## 每周期合成顺序

ACTIVE 当前按以下顺序工作：

1. support phase 使用生产支持力诊断更新，但尚不直接修改命令；
2. STEP task 读取 support 状态、observer 特征和操作手命令，更新结构化请求；
3. motion 完成 front/rear 与纵向命令映射；
4. motion 处理 STEP 的单周期参考事件；
5. motion 按 STEP control mode 或普通 NORMAL 路径组装唯一命令；
6. NORMAL 路径最后将 support request 与普通平衡控制合成。

STEP 优先级是显式的：

- `PASSIVE`：不生成任何执行器策略；
- `TRANSFER`：关闭轮端，只使用 STEP 腿位置轨迹，support 仍更新诊断但请求不参与输出；
- `RECOVER`：使用普通 balance control，按阶段决定是否屏蔽 `S/PSI`；support 请求不覆盖 STEP 恢复；
- `NORMAL/PREPARE`：使用普通 forward/yaw/腿长控制，并允许 support 约束最终能力。

禁止：

- task、support 或 benchmark 各自生成整份命令后互相覆盖；
- 用匿名 feedback mask 组合代替有名字的任务阶段；
- 把 case enable、MuJoCo 接触真值或 bypass 开关放入生产 config；
- task 直接计算 LQR、虚拟力或执行器力矩；
- support 修改操作手 task 或长期工作参考。

## NORMAL 语义

NORMAL 下纵向与偏航并行，不拆成直行、原地转向和边走边转三个互斥任务。

- forward 从 `HOLD` 开始。收到非零纵向意图后进入 `VELOCITY`，关闭 `S`、保留 `DS`；命令和参考归零、融合速度与原始轮速均低于门槛、轮速可靠并稳定后，回到 HOLD 并捕获停车位置。
- yaw 的语义是底盘跟随云台，而不是操作手直接给底盘角速度。motion 以 `5 deg` 滞回选择正面或反面；选择 rear 时同步反转云台坐标下的纵向命令。
- NORMAL 无论 HOLD 或 VELOCITY 都使用 `PSI/DPSI`，因此纵向模式切换不会关闭 heading follow。
- 起立固定使用 `0.18 m`，ACTIVE 后按 `0.40 m/s` 连续拉到配置工作腿长。

当前 operator command：

```c
uint8_t system_enabled;
uint8_t balance_restart;
float forward_velocity;       /* Gimbal-forward coordinates. */
bc_operator_task_t task;      /* NORMAL or STEP_DOCK. */
```

云台相对角和相对角速度属于 sensor feedback。下板负责 front/rear、纵向符号和 heading reference；仿真虚拟云台必须保持同一接口边界。

## Support phase 语义

生产状态链：

```text
GROUND -> AIRBORNE -> LANDING_RETRACT -> GROUND_RECOVER -> GROUND
               ^              |                 |
               +--------------+-----------------+  confirmed unload
```

- GROUND 只有在双腿接触诊断都为 AIR 后进入 AIRBORNE。旧 fast-air 已删除，不允许短时低支持力直接升级为整机离地。
- AIRBORNE 关闭轮端、快速伸腿至 `0.38 m`，只保留腿角 LQR 所需反馈；任一腿恢复 GROUND 诊断或支持力超过门槛并确认后进入 LANDING_RETRACT。
- LANDING_RETRACT 每腿捕获触地长度，使用 `K=800 N/m`、`D=80 N*s/m`、`0-240 N`、`3000 N/s` 的轴向阻抗，并以 `0.8 m/s` 把平衡点收回当前工作腿长。
- GROUND_RECOVER 根据当前轴向力反算位置 PD 等效参考，再以 `0.1 m/s` 连续回到普通 `POSITION_SUPPORT`。
- support request 是临时能力与腿策略，不拥有 forward/yaw 的长期意图。普通参考规划器在 support 阶段继续推进；轮速 observation context 根据 system 导出的支撑上下文决定融合、拒绝或重捕获。

正反向 `200/400 mm` 四个生产平台落地案例已覆盖该路径，且不使用 C++ control transform。当前入口等待双腿完整 AIR 诊断，真实跌落预判比已删除 fast-air 慢约 `28 ms`；低落差结构接触是已知代价。若未来恢复预响应，必须增加方向性垂向证据，不能复活旧的支持力/比力短时阈值。

## STEP_DOCK 语义

STEP 是操作手触发的 ACTIVE task，不是自动地形识别。状态链：

```text
INACTIVE -> PREPARE -> IMPACT_PASSIVE -> TRANSFER -> TRANSFER_HOLD
         -> RECOVER -> RECOVER_LOCK -> COMPLETE -> INACTIVE(disarmed)
                       \-> RECOVERY_FAILED
```

### PREPARE 与碰撞

PREPARE 请求 `0.38 m` 工作腿长并强制 FRONT。若正面偏差超过 `5 deg`，纵向目标通过原 forward 斜坡减至零；对齐后恢复操作手命令。伸腿不是单独的阻塞准备阶段。

碰撞武装门禁：双腿 `0.38 m +/- 20 mm`、正面误差不超过 `5 deg`、可靠正向轮速至少 `0.3 m/s`、support 为 GROUND。确认使用 observer 的 `5 ms` 窗口：最大腿世界角速度增量超过 `0.5 rad/s`，且轮速与 IMU 积分失配超过 `0.12 m/s`，连续 `2 ms` 后进入 IMPACT_PASSIVE。task 只读取 observer 特征，不回写通用 collision bool。

### PASSIVE 与 TRANSFER

IMPACT_PASSIVE 全 disabled 至少一个 `1 ms` 周期。随后从碰撞实测腿状态线性经过：

| 时间 | 腿长 | 相对机体腿角 |
| ---: | ---: | ---: |
| `0.08 s` | `0.24 m` | `-50 deg` |
| `0.16 s` | `0.16 m` | `-30 deg` |
| `0.34 s` | `0.17 m` | `-125 deg` |
| `0.50 s` | `0.18 m` | `-90 deg` |

TRANSFER 关闭轮端，腿长使用 `POSITION_SUPPORT`，腿角使用普通 `ANGLE_POSITION`；随后保持 `0.10 s`。默认腿角 PD `50/6/30` 已通过，不增加 task 专用高响应策略。

### RECOVER 与参考

进入 RECOVER 的单周期事件清空旧参考，将 forward/yaw generator 重置为零速，并屏蔽 `S/PSI`；此时不捕获位置。普通 `0.18 m` 支撑、轮 LQR、腿角 LQR、`DS/DPSI`、pitch 和腿世界角反馈继续工作，让轮轴可以移动到质心下方。

姿态、速度、腿状态、双侧 GROUND 和轮速可靠性连续满足 `0.05 s` 后进入 RECOVER_LOCK；该事件才捕获当前 `S/PSI`、设置 `DS_ref/DPSI_ref=0`、重启 generator 并恢复完整反馈。完整反馈再稳定 `0.05 s` 后进入 COMPLETE。pitch/pitch-rate 容差为 `5 deg/0.5 rad/s`；腿角速度按世界系 `DTHETA_L/R` 判断。

RECOVER 与 RECOVER_LOCK 累计超过 `4 s` 后锁存 `RECOVERY_FAILED`。捕获前失败继续屏蔽 `S/PSI`，捕获后失败保持已捕获参考；两者都禁止操作手运动且只能系统复位。

COMPLETE 保留一个周期并重置 support 为 GROUND，随后返回普通 ACTIVE。核心要求观察一次 NORMAL 才重新武装，防止持续 STEP 高电平重触发；keyboard 自动清除 `T` 锁存并产生该次 NORMAL，因此下一次只需再按一次 `T`。

## SPIN 预留

SPIN 尚未实现。它应是协调纵向停车、偏航 rate control 和命令许可的整车 task，而不是普通 yaw reference 的大角速度分支。候选序列：

```text
NORMAL -> SPIN_BRAKING_FORWARD -> SPINNING
       -> SPIN_BRAKING_YAW -> NORMAL
```

进入时先等待 longitudinal 回到 HOLD；SPINNING 禁止纵向运动并关闭 `PSI`、保留 `DPSI`；退出先做 rate-only 制动，再恢复 NORMAL 云台跟随。SPIN 与 AIRBORNE 同时发生时的安全行为尚无实验依据，第一版实现前必须明确冲突优先级。

## 剩余架构债务

1. `bc_motion_t` 仍同时容纳生命周期状态和 ACTIVE coordinator 数据；暂不为类型纯度迁移，只有当新 task 明显增加耦合时再提取独立 coordinator。
2. `bc_forward_mode_update` 仍直接修改 `S` feedback mask。更清晰的最终形态是 forward 输出轴级模式，由 motion 统一决定 mask；重构必须保持现有性能逐值不变。
3. 当前 `controller_update -> controller_calculate` 顺序使 observation context 使用周期开始时的 support 状态。新进入 AIRBORNE 后的立即 reject 不能撤销当拍已完成的 KF 校正；彻底修复需要重排周期，不应靠阈值补偿。
4. support 的离地预响应、实车接触参数、单轮/非对称落地和 rebound 仍未定稿。MuJoCo 真值只能评估，不得成为生产输入。
5. 气弹簧会改变普通支撑、落地和 jump 的腿轴向力语义；完成其安装几何和等效力曲线前，不新增 jump task。
