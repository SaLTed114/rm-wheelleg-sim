# 控制器实验日志

更新日期：2026-08-15

本文只记录当前实验主线和后续新增结果，不重复已经关闭的参数扫描。当前配置、架构边界和未决事项以 [`project-context.md`](project-context.md) 为准。

历史归档：

- [`2026-08-02--2026-08-09.md`](../archive/experiments/2026-08-02--2026-08-09.md)：早期运动学、起立、LQR 和平台摸排。
- [`2026-08-10--2026-08-15.md`](../archive/experiments/2026-08-10--2026-08-15.md)：正式 Q/R、转弯包络、坡道、fast-air、平台落地、jump impulse 和 STEP_DOCK 完整迁移过程。
- [`docs/archive/fast-air/`](../archive/fast-air/)：已删除 fast-air 的设计与参考实现。
- [`platform-drop-exploration.md`](../archive/experiments/platform-drop-exploration.md)：平台跌落、空中伸腿和主动悬挂专项归档。

## 当前实验基线：STEP_DOCK 完整迁移

生产状态链已经迁入纯 C 控制核心：

```text
PREPARE -> IMPACT_PASSIVE -> TRANSFER -> TRANSFER_HOLD
        -> RECOVER -> RECOVER_LOCK -> COMPLETE
```

`motion` 全程保持 ACTIVE 并唯一组装控制命令。IMPACT_PASSIVE 全 disabled 一个 `1 ms` 周期；TRANSFER 使用固定 `0.50 s` 四节点轨迹并 HOLD `0.10 s`。RECOVER 先屏蔽 `S/PSI` 完成姿态接管，稳定 `0.05 s` 后才在 RECOVER_LOCK 捕获当前位置和航向，完整反馈再稳定 `0.05 s` 后交还普通 ACTIVE。RECOVER 两段累计超时为 `4 s`。

Release `step_dock_complete` 当前结果：

- 碰撞速度约 `1.988 m/s`，碰撞世界 yaw 约 `-3.88 deg`。
- HOLD 双轮台面接触率 `100%`，最小轮轴越边余量约 `+93.9 mm`，最大 pitch 约 `18.16 deg`。
- RECOVER/RECOVER_LOCK 分别约 `0.639/0.051 s`；最终 pitch/pitch-rate 约 `3.34 deg/-0.161 rad/s`，双轮仍在台面。
- 最大关节请求约 `27.51 N*m`，未触发 `40 N*m` 限幅。RECOVER 最大轮请求约 `12.35 N*m`，被 `6.32 N*m` 短暂钳制后仍正常收敛；该冷启动限幅不作为失败。
- 默认腿角 PD `50/6/30` 已通过，不增加 TRANSFER 专用高响应策略。
- benchmark 不使用 control transform；MuJoCo 真值只做接触、边缘余量、饱和和最终保持断言。

常驻 STEP registry 只保留 `step_dock_complete`。旧 passive、delay、transfer 和 rebalance preview 的完整结果均在 `2026-08-10--2026-08-15.md`，不再承担日常回归成本。

## 当前 benchmark 收口

- `jump_impulse` 常驻 `jump_impulse_f140`、`jump_impulse_f240_t90ms`、`jump_impulse_f240_t120ms`，分别覆盖不离地负样本、代表性真实起跳和压力案例。完整六档扫描留在归档。
- STEP、jump 收口后 GUI/CLI 注册表共 30 个案例。其他 registry 保持正式极值、正交机理对照或有明确物理边界的最小矩阵。

## 下一轮记录要求

STEP 后续实验优先覆盖实车噪声、低速顶墙、颠簸、结构柔性、不同接近速度和非 `200 mm` 台阶。每次至少记录：

- PREPARE 门禁与碰撞确认延迟；
- 碰撞时速度、yaw、腿长和 observer 两项特征；
- 搬腿期间轮/底盘接触、轮轴越边余量和最大姿态；
- RECOVER 两段耗时、参考捕获时状态和最终交还状态；
- 轮/关节限幅前请求、应用值及持续时间；
- 是否需要系统复位、是否可单次按键重新武装。

气弹簧模型完成前不继续 jump task 调参；台面专用低摩擦也不直接推广到所有平面。新的探索若不改变生产结论，先写独立 trace/summary，形成稳定结论后再更新本日志。
