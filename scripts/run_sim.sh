#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/foxy/setup.bash
source install/setup.bash
# A dedicated local ROS domain keeps this simulation separate from real robots.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-187}"
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11355}"
export GAZEBO_IP=127.0.0.1
export GAZEBO_MODEL_DATABASE_URI=""
exec ros2 launch ep_simulation sim.launch.py "$@"
