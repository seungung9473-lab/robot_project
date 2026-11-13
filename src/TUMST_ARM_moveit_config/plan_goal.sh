#!/usr/bin/env bash
set -eo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$HOME/robot_ws/install/setup.bash"
if [[ -d "$here/runtime_deps" ]]; then export PYTHONPATH="$here/runtime_deps${PYTHONPATH:+:$PYTHONPATH}"; fi
exec ros2 run soarm4_moveit_config plan_joint_goal.py "$@"
