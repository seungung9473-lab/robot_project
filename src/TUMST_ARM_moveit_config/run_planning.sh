#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/robot_ws/install/setup.bash"
exec ros2 launch soarm4_moveit_config planning.launch.py "$@"
