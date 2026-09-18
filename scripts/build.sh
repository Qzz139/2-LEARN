#!/usr/bin/env bash
# 从仓库根目录加载 ROS 2 Foxy，以两个并行任务构建并使用符号链接安装。
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF
