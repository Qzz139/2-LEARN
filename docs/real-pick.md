# 真机抓放：从瓶子已在夹爪内开始

这是独立的 SDK 实机调试入口，不替换仿真 `pick_demo.py`。目前还不是课程要求的完整 ROS 2 / MoveIt 真机适配，也没有自动回零、寻找瓶子或五次循环。

## 启动

EP 开机，Jetson 连接 EP 热点，保留有线 SSH；退出其他控制 EP 的程序。瓶子含 150 ml 水、竖直放稳地面且位于张开的两爪之间，活动范围内无手和障碍物，现场人员能立即停止供电。

```bash
cd /home/robot/projects/2-LEARN
nmcli device status
bash scripts/run_real.sh
```

上面最后一条只预览，不连接 EP。确认摆放、参数及现场状态后执行一次：

```bash
timeout -k 5s 60s bash scripts/run_real.sh --execute
```

默认动作：以 SDK 出力设置 25 夹持 1.5 秒后暂停夹爪电机，抬升 20 mm，向车头前方移动 30 mm，下降 17 mm，张开夹爪 1.5 秒并暂停。没有夹爪力传感器，出力参数不是经过标定的牛顿值；150 ml 仅为水量，不是测得的总质量。

主控文件：`scripts/pick_real.py`；启动脚本：`scripts/run_real.sh`；参数：`config/real_pick.json`。可用 `nano config/real_pick.json` 修改参数。`lift_mm` 是相对抬升，`forward_mm` 是沿车头前进方向的相对位移，`release_clearance_mm` 是按机械臂位移计算的松爪间隙。存在夹持滑移时，该间隙不等于真实瓶底高度。

每次结果独立保存于 `work/real-pick-时间戳.json`。`commands_completed` 只表示指令和机械臂相对反馈通过；`physical_grasp_success` 初始为 null，必须由现场观察瓶子是否真正离地、放稳、无损来评价。SDK 等待结束不一定代表动作成功，因此程序还检查 `has_succeeded`。

## 失败和停止

程序不自动重试、回位或松爪。动作失败或 Ctrl+C 时会尝试向设备发出取消请求，并暂停夹爪；这个停止路径尚未在实机验证，不等于硬件急停。若运动异常或持续挤压，应由现场人员立即停止供电。外层 timeout 只限制程序运行时间，不能保证设备立即停住。

原始位置订阅存在无符号解析问题；本程序只用 32 位模运算比较运动前后的相对位移，不把异常原始值用作绝对目标。相对反馈误差超过 5 mm 时停止后续动作。数值边界只是本次小范围调试限制，不是完整的关节限位和碰撞规划。

三项假设备测试通过：有符号跨界时的相对位移、完整动作顺序、失败后不继续搬运/下降/松爪。

2026-09-11 首次运行在 SDK 建立连接阶段超时，没有发送夹持或移动命令。此次不计为抓取成功；记录见 [首次启动结果](validation/real/first-pick-attempt.json)。

协议参考：[官方 SDK 夹爪接口](https://github.com/dji-sdk/RoboMaster-SDK/blob/master/src/robomaster/gripper.py)、[官方动作状态实现](https://github.com/dji-sdk/RoboMaster-SDK/blob/master/src/robomaster/action.py)。取消请求字段的用法参考已有模型来源 [jeguzzi/robomaster_ros](https://github.com/jeguzzi/robomaster_ros/blob/c05a39d7f0fa8b3b277aa74826aa92e202efc987/robomaster_ros/robomaster_ros/action.py)。
