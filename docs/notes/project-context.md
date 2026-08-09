# rm-balance-sim 项目上下文

> 本文只用于后续开发会话快速恢复当前事实、设计边界和下一步，不保存逐轮实验流水账。
> 最近整理：2026-08-09。

## 当前焦点

项目已经具备可运行的 MuJoCo 闭链 plant、纯 C 控制核心、GUI、benchmark、状态估计和增益调度 LQR。实车质量/惯量基线已经写入 MJCF，LQR 已按新 plant 重新生成；当前首要问题是：

1. 分析正式 roll PD 已阻止翻倒、但联合工况单轮长时间离地的原因；
2. 为实车控制核心建立可信的轮地接触输入和 roll 回路离地门控；
3. 限制当前 yaw 加速度前馈的工作范围，避免把原地线性化得到的前馈无条件用于高速前进转弯。

用户明确提供的实车事实：同一类控制结构曾在 `2.3 m/s + 1.5*pi rad/s` 下保持正常的 roll、yaw 和腿姿态。因此当前仿真在该工况下失效不能简单解释为命令本身过猛。

## 设计边界

- 控制核心使用纯 C 和 SI 单位，不依赖 MuJoCo、GLFW、CAN、HAL 或线程实现；仿真与未来嵌入式工程通过各自 adapter 接入同一控制核心。
- 控制层使用虚拟腿长度/角度和虚拟力 `F/Tp`，plant 必须保留真实多连杆闭链、偏置质心和惯量；运动学等效不等于动力学等效。
- 不根据闭环表现随意填写未知 plant 参数。物理参数变化后必须重新验证闭链、符号、运动学/Jacobian、自由落体、参数提取、LQR 调度和性能基线。
- `/home/l/SaLT/wheelleg` 是只读历史项目，只用于失败案例和测试方法，不搬运其旧 MJCF、标定、LQR 表或补偿常量。
- `references/rm2026cb-balance-chassis/` 是用户实车实现的主要架构与行为参考；`references/matlab_scripts/` 保存实车 LQR 生成链。旧参数只有在明确作为实车目标或 golden test 时使用，不能静默混入当前 plant。
- 调参必须使用可复现的对照，至少同时检查限幅前后输出、姿态、腿状态、轮地接触、非轮触地、净空和停止阶段，不能只按 RMSE 选参数。

## 主要材料

- 当前 plant：`models/MJCF/COD-2026RoboMaster-Balance.xml` 和 `models/MJCF/*.STL`。
- 上游物理参考：`references/辽科轮腿模型/` 中的 USD/原始包；不作为运行后端。
- 用户实车工程：`references/rm2026cb-balance-chassis/`。
- 用户实车建模脚本：`references/matlab_scripts/lqr.m`、`leg_fit.m`。
- SJTU 原始建模资料：`references/SJTU balance control/WBR_modeling.html`、`WBR_control.html`、`WBR_leg.html`。
- 当前运动设计：`docs/notes/active-motion-design.md`。
- 性能实验归档：`docs/notes/performance-baseline.md`。
- LQR 生成报告：`docs/notes/lqr-validation.md`。
- 实车逐级部署：`docs/notes/hardware-bringup.md`。

## Plant 与质量参数

### 闭链和坐标

每侧实际结构由前、后两条支链和两个 equality `connect` 闭合，轮轴位于后支链末端。六个主动输入为左右前髋、后髋和轮轴，其余关节被动。

控制运动学使用每侧两个主动髋角计算虚拟腿长度 `L`、相对车体腿角 `theta` 和解析 Jacobian，并通过

```text
[tau_front, tau_rear]^T = J^T [F, Tp]^T
```

映射虚拟腿轴向力与摆动力矩。当前几何参数为 `l1=0.215 m`、`l2=0.254 m`。

坐标约定为 FLU：车头 `+X`、左侧 `+Y`、上方 `+Z`，正 pitch 为车头下压，正 yaw 为左转。上游 XML 的 Left/Right 命名与物理侧相反，adapter 固定采用 `BC_L -> XML Right_*`、`BC_R -> XML Left_*`，不重命名上游节点。

### 腿部运动学标定

MuJoCo 导出坐标并非精确镜像，固定 joint sign/offset 与共享理想两杆运动学之间存在分侧偏差。当前 adapter 零偏由 `tools/calibration/calibrate_leg_adapter.py` 从闭链模型重算，不再使用无来源的手写常量；生成的 JSON 保存模型 SHA256、采样姿态、拟合参数和验收结果，C++ adapter 与回归测试共用生成头文件。

标定固定 `l1=0.215 m`、`l2=0.254 m` 和两侧关节符号，只在 `0.18–0.21 m` 站立工作区以 `5 mm` 间隔拟合四个分侧零偏。当前结果为：

| 物理侧 | front offset | rear offset | 长度 RMS/最大误差 | 最大角度误差 |
|---|---:|---:|---:|---:|
| 左 | `-2.965142989 rad` | `-0.067723581 rad` | `0.477/0.749 mm` | `0.154 deg` |
| 右 | `-2.936362292 rad` | `-0.032802659 rad` | `1.011/1.594 mm` | `0.328 deg` |

同一物理姿态下左右虚拟腿长最大差为 `0.845 mm`。`0.186–0.39 m` 只作为不退化诊断区间；标定后左右长度 RMS/最大误差仍分别为 `3.370/5.240 mm`、`6.984/10.756 mm`，因此不能把当前结果描述为全行程运动学等价。后续若修正 joint/site 导出 frame，必须重新运行标定和完整回归。LQR 生成器使用物理 site 长度、运行时按虚拟腿长查表的坐标语义也仍需统一。

纯位置定位在接管前受重力影响，物理 site 长度约为 `0.1669/0.1582 m`，已经落在站立标定区之外；因此 motion 长度接管容差明确采用 `35 mm`。不通过篡改标定偏置、分侧杆长或重新加入支撑前馈伪造 `25 mm` 接管。

### 当前实车质量基线

MJCF 保留上游关节、连杆、轮径、轮距、mesh 和碰撞几何，只替换显式质量、质心与惯量：

| 聚合项 | 当前值 |
|---|---:|
| `base_link` 质量 | `17.650 kg` |
| 单侧腿质量（不含轮） | `1.190 kg` |
| 单轮质量 | `0.710 kg` |
| 总质量 | `21.450 kg` |
| `base_link` pitch 惯量 | `0.367565 kg*m^2` |
| `base_link` yaw 惯量 | `0.413477 kg*m^2` |
| 单轮轴向惯量 | `0.00119422 kg*m^2` |
| 机体质心相对虚拟髋关节高度 | `0.013 m` |

腿内各刚体质量和惯量按原有比例统一缩放，保留当前相对质心与惯性方向；缺少实车逐连杆 CAD/BOM 时不做欠定的重新分配。base roll 惯量缺少独立数据，保持旧 MJCF 的 `Ix/Iy` 比例，当前为 `0.366342 kg*m^2`。轮轴惯量按保留的 `0.058 m` 轮径和实车轮质量计算，其余主惯量保持原张量比例。

其他仍需保留的模型事实：

- `base_link` 是自由刚体；mocap weld 默认关闭，只供特定测试显式启用。
- 物理右腿闭链连接点有约 `1.95 mm` 固定横向错位，参数提取器按每侧固有误差处理，不应悄悄对称化。
- 当前机体复杂 mesh 直接参与地面碰撞，尚无独立简化碰撞体；`0.18 m` 腿长下理论底盘净空较小，后续仍需核对碰撞几何。
- 当前物理与控制周期均为 `1 ms`，运行时覆盖 MuJoCo timestep，不改原 MJCF 文件中的默认值。

### 偏航惯量结论

SJTU 原文中的 `I_z：机器人 z 轴转动惯量*2` 里的 `*2` 是脚注编号，脚注内容是“绕 z 轴的转动惯量简化为常量”，不是乘二。

其基本方程为

```text
I_z * ddphi = (-f_l + f_r) * R_l
```

`equ5` 中的 `I_z/(2R_l)` 来自左右差分坐标；结合 `equ7` 后严格化简为 `-I_z*D2psi`。因此生成器应使用一份完整物理偏航惯量，不能再次乘二。

`0.413477 kg*m^2` 是 `base_link` 自身 CAD yaw 惯量，不是 LQR 最终使用的整机 `I_z`。参数提取器在 `0.18 m` 腿长姿态下聚合所有刚体的自身惯量和平行轴项，得到正式整机值 `0.588029054 kg*m^2`；在 `0.18–0.39 m` 腿长采样中整机值介于 `0.571701774–0.588029054 kg*m^2`，nominal `0.34 m` 附近的诊断值为 `0.576616489 kg*m^2`。原始模型将整机 `I_z` 简化为常量，因此当前 LQR 固定使用 `0.18 m` 工作点值，暂不随腿长调度。

Python 生成器已删除倍数敏感性对照和 `base-link/assembly` 来源开关；旧实车 golden test 仍使用原脚本参数，不受当前 plant 提取方式影响。

## 控制软件

### 主数据流

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

- `update` 更新运动学和 observer；`set_command` 保存操作意图；`calculate` 推进状态机、参考和控制律；`execute` 只做 system 硬门控、限幅和最终输出。
- `bc_controller_t` 是正式 facade。PD/LQR 位于 `control_law/`，不依赖 controller、状态机或 observer。
- `bc_controller_snapshot_t` 是调用者持有的按需快照，统一服务 GUI、benchmark、测试和未来遥测；它不是 controller 内持续同步的第二份状态，也不是串口线格式。
- actuator 以左右腿前/后关节力矩和左右轮力矩表达，不在 C core 暴露 MuJoCo actuator 下标。

### 状态机与 NORMAL 输入语义

分层状态为：

```text
system: OFF / ON / FAULT
motion: IDLE / SELF_RIGHTING / LEG_POSITIONING / BALANCE_ENGAGING / ACTIVE
forward: IDLE / HOLD / VELOCITY
```

- 进入 ACTIVE 后从 `HOLD` 开始；有非零前进命令时进入 `VELOCITY`。
- `VELOCITY` 关闭 `S` 位置误差但保留 `DS`；命令、forward ramp 和融合 `DS` 连续归零 `0.25 s` 后回到 `HOLD` 并重新捕获 `S`。
- 普通 heading follow 与 forward mode 独立，在 `HOLD/VELOCITY` 中始终启用 `PSI/DPSI`。
- operator command 当前只有系统使能、单周期 restart 和云台坐标下的前进速度。上下板和云台 YAW 电机共用 CAN，下板可直接监听电机相对角/速度，不需要上板发送世界 heading。
- motion 根据云台相对角在 chassis front/rear 中选择误差较小的一侧，以 `5 deg` 差值滞回切换；选择 rear 时反转纵向速度。
- GUI 的 A/D 驱动无物理虚拟云台，W/S 始终表示云台正/反方向。任务级 SPIN 尚未实现，不根据普通 heading 命令自动进入。

### 参考、observer 与控制律

- forward reference：上限 `3 m/s`、默认加速度 `5 m/s^2`，写入 `DS_ref` 并积分 `S_ref`。
- yaw reference：由云台相对角/速度直接重建 `PSI_ref/DPSI_ref`，`DPSI_ref` 限制为 `+/-1.5*pi rad/s`，并输出受限差分得到的 `DDPSI_ref`。
- observer 使用 IMU 与共同轮速融合得到正式 `DS`，`S` 对该融合速度积分；轮速创新通过 NIS 和 `20 ms` 可信度迟滞拒绝接触闪断污染。IMU 外参、噪声和迟滞仍是仿真初值，实车到位后必须标定。
- 十维 LQR 状态为 `[s, ds, psi, dpsi, theta_l, dtheta_l, theta_r, dtheta_r, theta_b, dtheta_b]`，输入为 `[T_wheel_l, T_wheel_r, Tp_leg_l, Tp_leg_r]`。
- 当前调度覆盖 `0.160-0.390 m`；`0.160-0.186 m` 是等效腿参数拟合延伸区。当前 `Q=[90,60,40,15,240,4,240,4,300,60]`、`R=[3.2,3.2,0.7,0.7]`，腿角共同/差分权重为 `240/1920`，腿角速度为 `4/32`。
- 默认腿长 `0.18 m`；站立区 adapter 标定后重新扫描的工作点使用 `+2.42 deg` 共同腿角补偿和每腿 `76.204 N` 支撑前馈。`LEG_POSITIONING` 只使用长度/角度位置反馈，定位腿角 PD 的 `Kp=50`；支撑与 roll 差分力只属于 `POSITION_SUPPORT`。
- 轮力矩限幅 `6.32 N*m`，关节力矩限幅 `40 N*m`。
- yaw 加速度前馈为 `u = K(r-x) + 0.9*F_yaw*DDPSI_ref`。它显著改善普通原地 heading 停转过冲，但其线性化工作点接近零前进速度，尚未做速度调度。
- roll/roll rate 不进入十维 LQR，而是驱动独立 PD：`Kp=800 N/rad`、`Kd=60 N/(rad/s)`、差分力限幅 `200 N`。roll 是 `POSITION_SUPPORT` 的固有组成，不再根据 wheel/angle 策略反推是否启用；左右轴向力符号由配置数组给出，默认为 `{+1,-1}`，计算循环不临时判断左右侧。snapshot、UI 和 benchmark trace 直接记录 `roll_force_request`。
- 当前 sensor feedback 尚无可信接触状态，因此第一版没有照搬实车“任一侧离地便关闭 roll 回路”的门控；system OFF、定位和其他非平衡策略会自动将 roll 请求清零。

## 当前验证结论

- 构建与基础测试：Windows Release 18/18 CTest 和 9 项实验配置单测通过；LQR golden test 以旧实车参数复现固件 schedule，最大系数误差约 `5e-10`。
- 站立：质量重整后先按总质量比例把支撑前馈调为 `76.204 N/leg`。adapter 标定使旧坐标下选出的 `+0.7 deg` trim 产生约 `0.15 m` HOLD 位置残差。trim scanner 已改为与 keyboard/GUI 一致地固定虚拟云台世界朝向；重新粗扫并在 `2.34–2.46 deg` 以 `0.01 deg` 加密后选定 `+2.42 deg`。该点三秒评估窗最大位置误差约 `1.110 mm`、位移约 `0.013 mm`、平均 `DS` 约 `4.4e-5 m/s`，双轮持续接地且无饱和。
- 直线：`0.18 m、5 m/s^2` 下 `+/-1、+/-2、+/-2.5、+/-3 m/s` 全部完成、tracked 且 settled；`3 m/s` 最大 pitch 约 `2.22 deg`，没有轮或关节饱和。
- 普通 heading：`0.18 m、10 rad/s^2` 下 `+/-pi、+/-1.5*pi rad/s` 全部完成、tracked 且 settled；最大 pitch/roll 分别约 `1.38/0.97 deg`，没有执行器饱和。
- 用户实车代码的 roll PVI 为 `Kp=800`、`Kv=60`、`Ki=0.2`、输出限幅 `200 N`。当前仿真第一版采用相同 P/D 和限幅但暂不积分，以 `+/-` 差分叠加到左右腿轴向力。
- 无 roll 回路时，新质量 plant 在 `2.3 m/s + 1.5*pi rad/s` 正负转向均于 target-hold 翻倒，最大 pitch 约 `68-78 deg`、roll 到 `180 deg`，轮请求饱和比例约 `0.84-0.86`。
- 加入正式 roll PD 后，正负案例都完整运行并 tracked：最大 pitch 为 `4.76/5.35 deg`、最大 roll 为 `4.79/4.37 deg`、腿角差为 `3.46/3.00 deg`，roll 差分力峰值为 `93.5/88.8 N`，轮与关节均无饱和。这证明缺少 roll 回路确实是此前翻倒的主因。
- 联合案例仍未 settled，双轮同时接触率只有约 `61%`；接触缺口集中在 target-hold，转向内侧轮长时间离地，停止阶段则恢复双轮接触。不能把“没有翻倒”写成完整性能通过，下一步应处理接触状态和联合动力学，而不是继续盲目提高 PD 增益。

逐案例数值、历史 raw-yaw 隔离、observer 对照、trim 和权重扫描保存在 `docs/notes/performance-baseline.md`，不再复制到本文件。

## 仿真与诊断

- C++ 仿真层由 `MujocoPlant`、`MujocoAdapter`、`SimulationRunner`、`MujocoViewer`、`SimulationUi` 和场景编排组成；GUI `main.cpp` 只负责参数解析和启动。
- Dear ImGui 固定侧栏显示状态机、完整 state/reference、roll、云台相对反馈、腿运动学和限幅前后输出；UI 捕获输入时不会同时驾驶或移动相机。
- performance benchmark 与 GUI case 回放共用 `PerformanceScenario`。benchmark 输出逐周期 trace、summary、接触、姿态、限幅前后输出和问题标记，不作为 CTest 门禁。
- `tools/experiments/run_experiment.py` 用于生成独立 LQR 候选、隔离构建和批量案例；实验开关不得进入生产 controller config。
- 本地 MuJoCo、GLFW、ImGui 依赖放在被忽略的 `third_party/`，通过 `MUJOCO_ROOT`、`FETCHCONTENT_SOURCE_DIR_GLFW`、`IMGUI_ROOT` 显式提供；Linux 系统安装的 MuJoCo 可走标准搜索路径。详细命令见根 README。

## 当前未提交工作

- performance 自定义 heading 案例新增 `--coupled-forward` 和 `--forward-lead-seconds`，用于可复现的边走边转隔离；对应场景测试已加入。
- MJCF 已写入实车聚合质量、质心和惯量；参数提取器新增 `0.18 m` 整机 yaw 惯量工作点，LQR 正式使用 `0.588029054 kg*m^2`，生成 JSON/schema/header 和报告均已更新。
- 起立阶段已恢复纯位置定位；默认支撑、trim 和腿角定位 PD 已按新质量基线更新。
- 正式 roll PD 已接入左右腿差分轴向力；snapshot、UI、summary 和 trace 已加入 roll 力请求诊断。
- `LEG_POSITIONING` 保持纯 `POSITION`，roll 固定从属于 `POSITION_SUPPORT`。站立区 adapter 标定已建立可复现生成链；纯位置阶段采用明确的 `0.18+/-0.035 m` 接管条件，不重新偷加支撑前馈。
- LQR 生成器的一倍/两倍偏航惯量倍率和来源选择均已删除，生成 JSON schema 为 v2。
- `docs/notes/hardware-bringup.md` 已新增但尚未提交。
- 上述工作与现有 18 项 CTest 均已通过，工作区尚未提交。

## 下一步

1. 定位全行程运动学残差来自 joint/site 导出 frame 还是共享理想模型，并统一 LQR 物理长度与运行时虚拟调度坐标；不要用分侧杆长或非线性 adapter 静默吸收模型误差。
2. 设计可迁移到实车的轮地接触观测输入，先复刻实车“任一侧离地则关闭 roll 回路”，观察能否消除 target-hold 的长期单轮离地。
3. 扫描更低的前进/heading 组合网格，确定双轮接触开始恶化的边界，并区分物理载荷转移与控制振荡。
4. 将 yaw 加速度前馈至少限制在近零前进速度，之后再根据 `|DS|` 或 `|v*omega|` 实验平滑调度。
5. NORMAL 稳定后再实现任务级 SPIN；实车部署按 `hardware-bringup.md` 逐级进行，不一次上线完整功能。

## 快速恢复阅读顺序

1. 本文件。
2. `docs/notes/lqr-validation.md`。
3. `docs/notes/performance-baseline.md`。
4. `docs/notes/active-motion-design.md`。
5. `docs/notes/hardware-bringup.md`。
6. `models/MJCF/COD-2026RoboMaster-Balance.xml`。
7. `references/matlab_scripts/lqr.m` 与 `references/SJTU balance control/WBR_modeling.html`。
