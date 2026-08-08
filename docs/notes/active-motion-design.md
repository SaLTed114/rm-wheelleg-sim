# ACTIVE 普通运动与高速自旋设计

更新日期：2026-08-08

本文记录已经确认的运动语义、当前实现状态和下一步边界。NORMAL 已实现；
SPIN 仍只是后续设计，不存在控制入口或低层旁路。

## 本轮范围

ACTIVE 任务层第一版只包含：

- `NORMAL`：普通站立、直行、原地转向和边走边转；
- `SPIN`：显式开启的固定方向高速自旋。

跳跃、楼梯、坡跳和其他任务暂不加入枚举；等具体需求和阶段定义明确后再扩展。

## 普通运动

普通偏航的真实语义是“底盘跟随云台朝向”，不是直接控制底盘角速度：

- 仿真先提供独立于 GUI 的逻辑虚拟云台，GUI 键盘和 headless 场景共用；
- A/D 表示虚拟云台偏航速度，积分得到连续的世界朝向；松键后云台保持当前世界朝向；
- 鼠标暂不接入；实车上由真实云台姿态替换虚拟云台输入；
- NORMAL 中底盘无论静止还是前进都始终跟随云台，使用 `PSI + DPSI`；只有 SPIN 暂停跟随；
- W/S 始终表示沿云台正方向/反方向移动，而不是沿底盘固定正方向移动。

命令映射实时比较“底盘正方向对齐云台”和“底盘负方向对齐云台”，选择转角更小的一种：

- 选择负方向时，底盘坐标下的纵向速度命令同时反号；
- 该选择允许在运动中改变，使底盘被撞转约 180 度后仍能沿云台正方向继续运动；
- 在正负方向等距的 90 度附近按候选误差相差 `5 deg` 加入滞回防抖；
- 正/负方向选择立即生效。第一版不增加第二层角度轨迹规划，而是由底盘根据相对 heading 信息直接生成 `PSI_ref/DPSI_ref`；虚拟或真实云台自身的运动不受限制；
- 普通跟随的底盘角速度上限第一版使用 `1.5*pi rad/s`，与高速自旋分开。

纵向仍独立使用 `HOLD/VELOCITY` 反馈模式。因此 NORMAL 可自然覆盖静止、直行、原地转向和边走边转，不为四种组合建立笛卡尔积状态。

上下板和云台 YAW 电机位于同一条 CAN 总线，因此两块板都能直接监听电机反馈。
上板不计算或转发 heading 目标；当前 operator command 只包含：

```c
uint8_t system_enabled;
uint8_t balance_restart;
float forward_velocity;
```

`forward_velocity` 使用云台正方向坐标。云台 YAW 电机的相对角和相对转速属于
`bc_sensor_feedback_t::gimbal`；下板 motion 根据它们选择最近正/负方向并转换
纵向符号。NORMAL 每周期直接设置
`PSI_ref = PSI + selected_relative_yaw`，并将
`DPSI + gimbal_relative_yaw_rate` 限制到 `+/-1.5*pi rad/s` 后作为 `DPSI_ref`；
`PSI/DPSI` 始终启用。纵向模式已正式命名为
`IDLE/HOLD/VELOCITY`，只在 `VELOCITY` 中关闭 `S`。

## 高速自旋

SPIN 是协调纵向、偏航和命令许可的整车任务，不是 drive 或 yaw reference 内的一个轴级状态：

- 仿真输入用切换按键维护持续电平 `spin_enabled`，control core 不处理按键边沿；
- 自旋方向固定，不提供反向需求；目标速度为 `4*pi rad/s`；
- 进入时先把纵向速度降到零并锁住位置，再开始高速自旋；允许多少阶段重叠留到后续精调；
- SPIN 中禁止纵向运动命令，关闭 `PSI`、保留 `DPSI` 速度控制；
- 虚拟或真实云台在 SPIN 中仍独立接受转向输入并保持自己的世界朝向，底盘暂时不跟随；
- 退出时先进入 rate-only 制动阶段；当实际 `|DPSI|` 低于可配置的跟随重入阈值并稳定一段时间后，再恢复 NORMAL 云台跟随；
- 重入阈值不要求理论上完全停转，第一版取保守值，之后用实验确定能够平顺恢复跟随的最高速度。

建议的任务阶段为：

```text
NORMAL
  -> SPIN_BRAKING_FORWARD
  -> SPINNING
  -> SPIN_BRAKING_YAW
  -> NORMAL
```

这些是 SPIN 任务内部阶段，不扩张顶层任务枚举。

## 模块边界

```text
GUI/headless -> virtual gimbal -> motor-equivalent feedback --+
real yaw motor CAN feedback -----------------------------------+-->
upper-board forward/task intent -------------------------------+-->
    control-core NORMAL mapping
      -> forward and yaw reference generation
      -> LQR / PD
```

虚拟云台只负责产生与真实 YAW 电机等价的相对反馈，不进入 balance controller。
正/负方向选择和纵向坐标转换属于下板 NORMAL motion；控制核心只认识数值反馈，
不识别键盘、鼠标、CAN 帧或具体电机协议。

GUI 和 performance scenario 目前共用同一个虚拟云台。A/D 目标为
`+/-pi rad/s`，云台角速度以 `10 rad/s^2` 斜坡变化并限制在
`+/-1.5*pi rad/s`；首次进入 ACTIVE 时捕获底盘朝向。GUI 用机体上方的青色
箭头显示云台世界朝向，并显示 front/rear、heading error、云台角速度和映射后
纵向速度。

2026-08-08 的 Windows Release 验证为 17/17 CTest 通过。独立的 C++ mapper
模块和测试已经删除，其映射覆盖并入 C motion 测试。NORMAL heading
follow 的 `+/-pi` 与 `+/-1.5*pi rad/s` 四个 MuJoCo 档位均完成、有限且满足
跟踪判据，无执行器饱和；默认两秒停止观察窗内仍有偏航残振，四档均未满足
`settled`。这属于下一轮 NORMAL 精调证据，不通过添加 raw yaw-rate 旁路规避。

## 尚未确定

- SPIN 的纵向停稳阈值、阶段重叠、自旋加速度和退出跟随重入速度；
- NORMAL heading follow 的第二层角度轨迹是否必要，以及当前直接跟随的过冲和
  停止残振应如何精调；
- `5 deg` 正/负方向选择滞回是否适合实际操作手感；
- 后续任务的名称与状态。需求出现前不预建跳跃、楼梯等枚举。
