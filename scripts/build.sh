#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF
