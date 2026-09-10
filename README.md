# RoboMaster EP 固定底盘定点抓取仿真

在 Jetson Orin 上运行 Gazebo 11 + ROS 2 Foxy + MoveIt 2。底盘固定，只控制两轴机械臂和夹爪，执行接近、抓取、抬升、放回和回位。

模型复用 `jeguzzi/robomaster_ros`；夹爪接触面、惯性和动力学做了简化。详见 [模型来源与边界](docs/model.md) 和 [验证记录](docs/validation.md)。

## 项目结构

```text
src/
  robomaster_description/  上游 EP 网格、Xacro 和许可证
  ep_gazebo_control/       有界 Gazebo 电机驱动，兼容标准轨迹控制器
  ep_simulation/           模型适配、场景、启动文件及自动抓取
  ep_moveit_config/        MoveIt/OMPL、控制器和 RViz 配置
scripts/                  构建、运行和会话管理入口
tests/                    模型拓扑、联动与运动学测试
docs/                     来源、简化说明与实测记录
work/                     本机运行日志和实验结果，不提交 Git
build/ install/ log/       colcon 生成文件，不提交 Git
```

## 安装与构建

在 Ubuntu 20.04 / ROS 2 Foxy 环境运行；Gazebo 11 应已安装。

```bash
sudo apt-get install -y --no-remove \
  python3-colcon-common-extensions python3-yaml \
  ros-foxy-xacro ros-foxy-robot-state-publisher \
  ros-foxy-rviz2 ros-foxy-moveit ros-foxy-gazebo-ros-pkgs \
  ros-foxy-gazebo-ros2-control ros-foxy-controller-manager \
  ros-foxy-joint-state-broadcaster ros-foxy-joint-trajectory-controller
bash scripts/build.sh
```

## 在 SSH/VNC 中运行

当前 Jetson 项目目录为 `/home/robot/projects/2-LEARN`，VNC 桌面为 `:2`。

```bash
cd /home/robot/projects/2-LEARN
python3 scripts/session.py start --display :2
```

VNC 中会出现 Gazebo 和 RViz。等待模型、控制器加载后执行：

```bash
bash scripts/pick.sh
```

结果写入 `work/pick_result.json`。成功必须同时满足物块实际抬升至少 25 mm、放回误差小于 12 mm；失败返回非零退出码。

```bash
python3 scripts/session.py status
python3 scripts/session.py stop
```

无界面运行使用 `python3 scripts/session.py start --headless`。前台运行可用 `bash scripts/run_sim.sh gui:=true rviz:=true`，用 Ctrl+C 停止。重复实验可以先停止再启动以恢复初始场景；如果物块已偏离抓取点，脚本会明确失败，不会自动把物块瞬移回去。

仿真默认使用 `ROS_DOMAIN_ID=187`、本机 ROS 通信和 Gazebo 端口 `11355`。所有脚本保持同一环境，不要在同一终端混合 source ROS 1 Noetic。模型驱动不连接真实 EP。

## 调整抓取点

`src/ep_simulation/ep_simulation/scene.py` 集中定义 `PICK`、`LIFT`、物块和台座。目标使用弧度制关节坐标；`model.fk()` 可计算工具中心的世界坐标。修改场景后重启仿真。

`ep_moveit_config/config/` 中设置规划限速；`ep_simulation/config/controllers.yaml` 设置轨迹控制器。RViz 可查看机器人和规划轨迹，当前流程不依赖六自由度末端拖拽 IK。

## 验证

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
python3 -m unittest discover -s tests -v
```

通过 Mac SSH 下载 GitHub 仓库再同步到 Jetson 时，不要把 `build/`、`install/` 和 `work/` 跨机复制；代码同步后在 Jetson 上构建。提交应包含源码、许可证和必要验证摘要，不包含构建缓存和大体积日志。

## 课程任务要求
<img width="590" height="766" alt="image" src="https://github.com/user-attachments/assets/9053b3c5-37d4-46f7-9a4b-58b216af0e4b" />

