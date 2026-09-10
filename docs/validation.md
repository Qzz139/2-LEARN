# Jetson 验证记录

验证日期：2026-09-10（Asia/Shanghai）。平台：Jetson Orin / aarch64 / Ubuntu 20.04。

## 环境

| 组件 | 现场安装版本 |
|---|---|
| Gazebo Classic | 11.15.1 |
| ROS 2 | Foxy |
| RViz 2 | 8.2.8 |
| MoveIt 2 | 2.2.3 |
| gazebo_ros2_control | 0.1.1 |
| controller_manager | 0.11.0 |
| joint_state_broadcaster / joint_trajectory_controller | 0.9.0 |

## 构建与模型检查

四个 ROS 包在开发板上通过 `colcon build --symlink-install --parallel-workers 2` 构建。

`python3 -m unittest discover -s tests -v`：5 项全部通过，覆盖固定底盘、独立关节数量、规划组联动关节完整性、URDF 与独立正运动学的一致性（16 组姿态）、末端恒定俯仰、耦合电机限位及场景抓取间隙。

## 端到端抓取

每次从新启动的世界开始。流程为开爪 → 回位 → 接近 → 下探 → 合爪 → 抬升 → 放回 → 开爪 → 撤离 → 回位。所有机械臂轨迹由 MoveIt/OMPL 规划并执行；物块位置从 Gazebo 服务读取。

| 运行方式 | 物块实际抬升 | 放回位置误差 | 退出码 |
|---|---:|---:|---:|
| 无界面 | 45.48 mm | 0.55 mm | 0 |
| VNC :2 同时运行 Gazebo 和 RViz | 45.17 mm | 0.38 mm | 0 |

原始记录：[无界面 JSON](validation/headless-pick.json)、[可视化 JSON](validation/gui-pick.json)。含每段关节反馈、轨迹点数、物块前后位置和时间戳。验收阈值为抬升 ≥25 mm、放回误差 <12 mm。这是两次功能验收，不是统计可靠性或实机精度报告。

VNC 中已检查 Gazebo 场景及 RViz 机器人模型均可渲染。修复了 RViz 不接受裸绝对网格路径的问题，统一使用 `file://` URI。可视化时帧率较低，完整流程仍通过；性能测试不在本次验收范围内。

## 已知提示与限制

- 没有配置通用 IK 插件，RViz 会提示未定义 kinematics；本流程使用两独立关节目标，规划和执行已验证。末端拖拽规划尚未实现。
- 没有深度传感器/Octomap 输入，可能出现未配置 3D sensor 的日志；本版本显式添加已知地面和台座进行碰撞检查。
- 物块由接触和摩擦抬起，没有位姿写入或固定吸附。夹爪、惯性、碰撞体及重力处理存在简化，详见 [模型边界](model.md)。
- 抓取目标尚未作为附着碰撞物体加入 MoveIt，因此本流程不代表任意障碍场景下的搬运规划。
