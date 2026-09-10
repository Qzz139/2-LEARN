# 模型来源与仿真边界

## 复用的公开成果

- 模型仓库：[jeguzzi/robomaster_ros](https://github.com/jeguzzi/robomaster_ros)
- 固定版本：`c05a39d7f0fa8b3b277aa74826aa92e202efc987`
- 原始 `robomaster_description` 的 `urdf/`、`meshes/`、`launch/` 放在同名 ROS 包中。
- 原作者 Jérôme Guzzi，MIT 许可证保留在 `src/robomaster_description/LICENSE`。只修改了构建清单和包元数据，模型适配集中在 `ep_simulation/model.py`，不覆写原始 Xacro。
- 复用 Gazebo Classic 11 / ODE、ROS 2 Foxy、MoveIt 2 / OMPL、ros2_control 和 JointTrajectoryController；不引入实机 RoboMaster SDK。

## 保留与简化

底盘通过 `world_fixed` 固定于世界，轮子保持外观但没有驱动。机械臂沿用上游几何尺寸及关节轴，两独立坐标为：

- `arm_1_joint`：右电机（肩部）角度 `a`。
- `arm_2_joint`：相对肘角 `b`，等于左电机角度减右电机角度。
- 左电机角度为 `a+b`；原 `rod_3_joint` 为 `-b`。

用同轴、零长度的从动关节分解加减关系，使 URDF 保持树结构。末端俯仰满足 `a+b-a-b=0`，夹爪保持水平。MoveIt 的 arm 组包含所有联动关节，但只有两个独立变量。规划使用关节目标，不要求通用六自由度 IK；每段轨迹额外检查左电机的耦合限位。

原始复杂夹爪机构替换为**平行夹爪代理**：保留上游夹爪壳体，两个矩形接触指沿正负 Y 同步运动，每侧开合 0–50 mm。此模型适合定点抓取流程验证，不能据此断言真实 EP 夹爪的精确工作空间、抓力或载荷。

为了稳定运行，碰撞体使用简单凸盒，装饰零件省略碰撞；小连杆惯性使用正定近似值，机械臂关闭重力。物块（25 mm、20 g）保留重力和物理碰撞。因此这是**运动学及接触流程仿真，不是经过标定的整机动力学模型**。规划场景包含地面和台座；目标物块由 Gazebo 进行接触计算，当前没有同步为 MoveIt 的附着碰撞物体。拿起后避障须考虑这一限制。

## 为什么增加 `ep_gazebo_control`

Jetson 已安装的旧版 GazeboSystem 的 position 接口调用 `SetPosition`，机械臂可以移动，但本项目实测夹持物块不能随之抬升。项目驱动保留标准 position 命令接口，内部使用 ODE 的速度电机约束追踪目标，给速度和力/力矩设置上限，从而保留接触摩擦。

驱动约 150 行，无自定义轨迹协议；上层仍是 MoveIt → FollowJointTrajectory → 标准 JointTrajectoryController。被动关节在驱动内跟随独立关节目标。当前参数是仿真调试值：机械臂最高 0.5 rad/s、20 N·m，手指最高 0.04 m/s、2 N，不代表 DJI 实机规格。

抓取不使用 SetEntityState、物体位姿写入或固定吸附。自动验收以 Gazebo 返回的物块位置为依据：抬升至少 25 mm，放回误差小于 12 mm。

## 限制

- 底盘固定后只支持原装两轴臂所在平面内的目标；末端没有独立偏航或俯仰控制。
- 本版本适配 Ubuntu 20.04 / ROS 2 Foxy / Gazebo Classic 11 的 API。
- Gazebo Classic 与 Foxy 均为旧版本；迁移新 ROS 发行版时需要复查硬件接口 API。
- 任何实机部署前必须另做尺寸、负载、关节零位及速度限制标定；本仓库没有实机控制入口。
