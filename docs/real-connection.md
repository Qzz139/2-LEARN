# 真机连接检查：第一步，不执行运动

## 你怎样运行

EP 开机，Jetson 连上 EP 热点；保留有线 SSH。关闭其他正在控制 EP 的程序。
在 Jetson 终端执行：

```bash
cd /home/robot/projects/2-LEARN
python3 scripts/check_ep_connection.py
```

无需启动 Gazebo，也无需构建 ROS 2 包。此脚本不是抓取程序，不会回零、移动机械臂、控制底盘或开合夹爪。SDK 初始化会启用 SDK 模式、重置数据订阅、选择 FREE 模式和重置视觉订阅，因此不是完全不改变设备通信状态的被动读取。

## 文件分别负责什么

| 文件 | 用途 |
|---|---|
| `scripts/check_ep_connection.py` | 连接 EP、读取版本、接收至少两次有效数字形式的位置反馈，并检查数值是否明显异常 |
| `config/ep_connection.json` | 热点直连 IP 和超时时间；`local_ip: null` 根据到 EP 的路由选择 Jetson 地址 |
| `work/ep-connection.json` | 最近一次检查结果，每次运行覆盖 |
| `work/ep-connection.log` | 最近一次 SDK 输出，每次运行覆盖 |
| `tests/test_ep_connection.py` | 使用假设备检查正常、失败、无效和异常大坐标；不会连接机器人 |

配置不保存 Wi-Fi 密码，网络连接由系统管理。当前检查仅支持 AP 热点直连 / UDP。已确认 Jetson 安装 SDK `0.1.1.62` 且可导入，没有重新安装或修改它。

## 怎样理解结果

- `connection_ok: true`：SDK 初始化成功；`robot_version` 有内容，表示版本查询得到响应。
- `position_samples_received`：接收到的有效数字位置样本数量，不等于经过现场标定的位置。
- `position_valid: true`：通过粗略数值检查，不代表坐标已校准、路径可达或允许运动。
- `success: false`：有项目未通过；看 `stage` 和 `error`，不要反复运行碰运气。
- `motion_commands_sent: false`：本脚本没有发送运动命令，不是对其他程序是否控制机器人的判断。

最多等待 40 秒，超过时间终止 SDK 检查子进程并写入失败结果；正常结束时取消订阅并关闭连接。这不是机械臂急停功能。

## 2026-09-11 首次实机发现

已读到 EP 固件 `01.01.1150`，收到机械臂原始位置 x=159、y=4294967293。已安装 SDK 的 `ArmSubject.decode()` 使用 `struct.unpack('<II', ...)`，即无符号 32 位整数。y 值若按有符号 32 位整数解释为 -3 mm，但尚未通过实物位置或其他反馈交叉验证。

脚本保留原始值，额外输出 `signed_int32_candidate_mm` 供排查。遇到异常大坐标时，用 SDK 内置的独立位置查询协议 `ProtoRoboticArmGetPostion`（0x33/0x14，只查询）交叉核对；该响应按有符号整数解码。只有候选值在 ±1000 mm 粗检查范围内，且与独立查询相差不超过 3 mm，才接受查询值。否则失败退出，不自动接受候选值。该范围不是 EP 工作范围；即使通过，也没有完成实物坐标标定。

加入异常检查后再次实测：通信成功，x=159、y=4294967292（有符号候选值 -4 mm），位置检查返回失败、进程退出码 1，未发送运动命令。原始结果见 [实机连接检查记录](validation/real/connection-check.json)。两次读取的候选值约为 -3 至 -4 mm，尚不据此确认物理位置。四项假设备测试全部通过。

用户已确认机械臂、夹爪原装，照片中的黄色部分为胶带。2026-09-11 的最终交叉核对仍读到 x=159、y=4294967293，但独立位置查询超时；因此位置检查继续保持失败，未发送运动命令。记录见 [交叉核对结果](validation/real/signed-position-check.json)。六项假设备测试通过，包含查询一致与不一致两种情况。按用户要求控制工作量，不继续重复实机试验。

下一步需解决反馈解码验证和坐标标定，再编写/验证真机运动适配。现有 `pick_demo.py` 和仿真启动入口未修改；真机抓取程序、共享任务流程重构、ROS 2 真机轨迹接口都尚未实现。

参考：[官方 SDK 入门与查询接口](https://robomaster-dev.readthedocs.io/en/latest/python_sdk/beginner_ep.html)、[官方机械臂反馈解码实现](https://github.com/dji-sdk/RoboMaster-SDK/blob/master/src/robomaster/robotic_arm.py)。
