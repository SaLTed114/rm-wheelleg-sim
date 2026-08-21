# 机器人模型

本目录保存 MuJoCo 运行所需的 MJCF 和 STL。当前仿真使用 `MJCF/Fudan-2026RoboMaster-Balance.xml`；`MJCF/COD-2026RoboMaster-Balance.xml` 是迁移前的辽宁科技大学 COD 模型，仅作为 plant 来源和历史参考。

## 当前模型

当前模型保留原 COD plant 的地面、坡道、障碍物和 keyboard 交互对象，将机器人本体替换为复旦开源模型中的真实闭链机构。每侧由前髋、后髋和轮轴三个主动关节驱动，前后支链通过 MuJoCo equality constraint 闭合；模型不包含气弹簧。

机体、腿部连杆和轮子都可以与地面接触。每侧 `f0` 与 `22` 连杆之间额外启用自碰撞，并通过 `tools/model/build_fudan_collision_proxies.py` 生成的凸分片避开转轴周围 `25 mm` 半径区域。轮子完整采用原 COD 模型的左右 STL、`58 mm` 碰撞半径、`0.71 kg` 质量和惯量；由于两套模型的左右命名与局部坐标不同，生成脚本会交叉映射左右轮并分别设置安装姿态。

`MJCF/fudan/` 保存当前模型使用的复旦 STL 和生成的自碰撞分片，`MJCF/ustl/` 保存原 COD 模型资产。生成后的 XML 和 STL 已经包含运行所需内容，正常编译和仿真不依赖 `references/`。

## 重新生成

`MJCF/Fudan-2026RoboMaster-Balance.xml` 是生成文件，不应直接维护。生成器以 `MJCF/COD-2026RoboMaster-Balance.xml` 为 plant shell，从 `references/fudan_rl_wheel_leg/` 读取复旦机器人源模型，再写入当前仓库中的模型和资产路径。

将复旦上游仓库检出到 `references/fudan_rl_wheel_leg/` 后运行：

```bash
conda run -n sim python tools/model/build_fudan_plant.py
```

当前使用的复旦上游版本为 `de0770cd2f5dcd95b576079b9aa52de431c7c310`。`references/` 不随本仓库提交，因此重新生成前需要自行准备这个固定版本。

## 辽科 COD 来源

原项目为辽宁科技大学 COD 战队发布的 `COD-2026RoboMaster-Balance-Simulation_File`，上游仓库为 <https://github.com/GrassFanWang/COD-2026RoboMaster-Balance-Simulation_File>，本仓库引入版本为 `089e35a97e4be832f293547d283eb6f62a22185f`，引入日期为 2026-07-30。上游提交作者为 GrassFanWang（`1985483641@qq.com`）。

`MJCF/COD-2026RoboMaster-Balance.xml`、`MJCF/ustl/` 和 `Pictures/` 来自该项目，并以普通文件形式保存在本仓库中。上游 USD 文件仅用于 Isaac Sim，本项目没有提交该文件；本地参考副本可以放在 `references/辽科轮腿模型/COD-2026RoboMaster-Balance.usd`。

![COD MJCF 在 MuJoCo 中的预览](./Pictures/MJCF%20in%20Mujoco.png)

![COD USD 在 Isaac Sim 5.0 中的预览](./Pictures/USD%20in%20IsaacSim5.0.png)

截至引入日期，COD 上游仓库没有提供明确的 `LICENSE` 或 `NOTICE` 文件。本仓库保留原作者和发布团队署名，不对这些模型资产作重新许可声明。

## 复旦模型来源

机器人本体来自 `fudan_rl_wheel_leg`，上游仓库为 <https://github.com/yly-true/fudan_rl_wheel_leg>，本地源目录为 `references/fudan_rl_wheel_leg/`。`tools/model/build_fudan_plant.py` 只抽取当前模型所需的机体和腿部资产，并结合本仓库的 plant shell、碰撞代理和辽科轮子生成最终 MJCF。
