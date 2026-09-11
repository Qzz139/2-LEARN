# RoboMaster EP 地面瓶子定点取放仿真

在 Jetson Orin 上运行 Gazebo 11 + ROS 2 Foxy + MoveIt 2。底盘固定于地面，使用两轴机械臂夹取竖直瓶子，从固定 A 点移动到地面 B 点并释放。场景中没有台座。

复用 `jeguzzi/robomaster_ros` 的 EP 网格与 Xacro，保留上游 MIT 许可证。瓶子是近似 550 ml 瓶型的刚体代理；尺寸、空瓶质量、装水量和摩擦均可配置。**不能用仿真结果推断 PET 瓶不变形或满瓶在实机上可抓取。**

课程原文要求桌面；地面版本是用户指定的场景调整，需确认教师接受。详见 [课程要求与覆盖范围](docs/course-requirements.md)、[模型边界](docs/model.md)、[地面瓶子验证](docs/bottle-validation.md) 和 [真机准备方案](docs/real-control-plan.md)。

## 真机准备：通信检查

新增 [真机抓放调试入口](docs/real-pick.md)：`bash scripts/run_real.sh` 预览，添加 `--execute` 执行一次从当前夹爪位置开始的抓放。此入口直接使用 SDK，尚未实现完整 ROS 2 真机接口或回零；原有仿真不变。首次启动遇到 EP 连接超时，尚未验证抓取成功。

Jetson 连接 EP 热点后执行 `python3 scripts/check_ep_connection.py`。仅检查 SDK 通信、版本和机械臂位置反馈，不发送运动命令；结果保存到 `work/ep-connection.json`。操作、已发现的坐标解码异常及下一步见 [真机连接说明](docs/real-connection.md)。

## 项目结构

```text
src/
  robomaster_description/  复用的上游 EP 网格、Xacro、许可证
  ep_gazebo_control/       仿真电机驱动，兼容标准关节轨迹 Action
  ep_simulation/           模型、平面逆解、瓶子配置、场景、任务与 Launch
  ep_moveit_config/        MoveIt/OMPL、控制器和 RViz 配置
scripts/                  构建、运行、停止入口
config/                   真机连接参数（不含密码）
tests/                    拓扑、运动学、可达性和场景测试
docs/                     课程要求、来源、验证数据、真机计划
work/                     日志和轨迹结果（不提交 Git）
build/ install/ log/       colcon 缓存（不提交 Git）
```

## 安装与构建

Ubuntu 20.04 / ROS 2 Foxy，Gazebo 11 已安装：

```bash
sudo apt-get install -y --no-remove \
  python3-colcon-common-extensions python3-yaml \
  ros-foxy-xacro ros-foxy-robot-state-publisher \
  ros-foxy-rviz2 ros-foxy-moveit ros-foxy-gazebo-ros-pkgs \
  ros-foxy-gazebo-ros2-control ros-foxy-controller-manager \
  ros-foxy-joint-state-broadcaster ros-foxy-joint-trajectory-controller
bash scripts/build.sh
```

## 一条 Launch 启动完整系统与任务

在 Jetson 上运行，确认本项目没有另一会话占用仿真端口：

```bash
cd /home/robot/projects/2-LEARN
export DISPLAY=:2
export XAUTHORITY=/home/robot/.Xauthority
bash scripts/run_sim.sh run_task:=true cycles:=1
```

底层为同一个 `ep_simulation sim.launch.py`，启动 Gazebo、MoveIt、控制器、状态节点、RViz 和任务节点。`gui:=false rviz:=false` 可关闭可视化；`cycles:=5` 执行五次验收。后台自动验收可用 `python3 scripts/session.py start --headless --run-task --cycles 5`，结果为 `work/bottle-five.json`。结束后 Ctrl+C 关闭系统。

后台会话和手动触发：

```bash
python3 scripts/session.py start --display :2
bash scripts/pick.sh --cycles 1 --output work/bottle-result.json
python3 scripts/session.py status
python3 scripts/session.py stop
```

连续验收使用 `bash scripts/pick.sh --cycles 5 --output work/bottle-five.json`。同一仿真世界中执行五次 A→B；两轮之间机械臂实际把瓶子 B→A 搬回，这四次复位单独记录，不计入五次成绩，没有物体瞬移或重启世界。任务失败即停止后续动作，不为了凑次数继续盲抓。

## 修改瓶子与 A/B

`src/ep_simulation/config/bottle.json` 是共同配置源：

- 近似瓶高 227 mm、直径 64 mm；需要测量实物批次后更新。
- 默认空瓶假设 25 g、装水 150 ml，总重 175 g；150 ml 不是避免形变的保证。
- A 点 x=275 mm，B 点 x=320 mm，y 固定在机械臂平面内。
- 在瓶身离地 105 mm 处夹持，抬升 50 mm；放置时根据观测到的夹持滑移修正高度，使瓶底留约 2 mm 松爪间隙后落地。
- `open_half_gap_m` 是单侧半开口，不是总开口。

配置使用 world 坐标和米。`model.ik(x,z)` 转成两独立关节角，启动前检查可达性/耦合限位；每条 MoveIt 轨迹再检查全部路点。固定底盘不能横向移动瓶子。

修改默认配置后重启仿真。另用配置文件时，Launch 的 `bottle_config:=/绝对路径/bottle.json` 与手动任务的 `--config /绝对路径/bottle.json` 必须一致；用 `run_task:=true` 自动共享配置。

## 输出与限制

[下载仿真演示视频](docs/videos/ground-bottle-demo.mp4)：2026-09-10 录制的一次完整 A→B 取放，包含回零、抓取、抬升、搬运、释放和返回。对应 [轨迹与执行结果](docs/validation/bottle/recorded-demo.json)。视频为原速桌面录制，Gazebo 渲染帧率较低，画面存在卡顿；详细说明见 [视频说明](docs/videos/README.md)。

`work/pick_result.json` 保存配置、每次成绩、复位成绩、实际瓶子位置/倾角、轨迹关节位置/速度/加速度/时间以及错误；会话日志在 `work/sim.log`。单次验收需要实际抬升 ≥25 mm，B 点误差 ≤12 mm，瓶子倾角 ≤10°，一秒稳定性采样漂移 ≤3 mm。运行失败返回非零退出码。

MoveIt 中携带瓶子碰撞体；Gazebo 中依靠夹爪接触摩擦，未用固定吸附。机器人惯性、联动、夹爪仍为简化代理，不能代替完整动力学/损伤验证。真机接口尚未实施。

ROS 默认域为 187，仅本机通信，Gazebo 端口 11355；不连接真实 EP。不在同一终端混合 source ROS 1 Noetic。

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
python3 -m unittest discover -s tests -v
```

旧台座方块验证保留于 `docs/validation.md`，仅代表旧版本结果。

## 课程任务要求
<img width="590" height="766" alt="image" src="https://github.com/user-attachments/assets/9053b3c5-37d4-46f7-9a4b-58b216af0e4b" />

文字转录及范围说明见 [课程要求](docs/course-requirements.md)。
