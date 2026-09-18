#!/usr/bin/env bash
# 加载已构建工作区，在本机 ROS 域中运行仿真取放任务；原样传递命令行参数。
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-187}"
export ROS_LOCALHOST_ONLY=1
exec ros2 run ep_simulation pick_demo.py "$@"
