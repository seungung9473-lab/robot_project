#!/usr/bin/env bash
set -euo pipefail
package_source="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
robot_workspace="${1:-$HOME/robot_ws}"
mkdir -p "$robot_workspace/src"
robot_workspace="$(cd -- "$robot_workspace" && pwd)"
target="$robot_workspace/src/soarm4_description"
if [[ "$package_source" != "$target" ]]; then
  stage="$(mktemp -d "$robot_workspace/.soarm4_checked_stage_XXXXXX")"
  cp -a "$package_source" "$stage/soarm4_description"
  test -f "$stage/soarm4_description/urdf/soarm4_visualization.urdf"
  if [[ -e "$target" ]]; then
    backup="$robot_workspace/soarm4_description_backup_$(date +%Y%m%d_%H%M%S)_$$"
    mv -- "$target" "$backup"
    echo "기존 패키지 백업: $backup"
  fi
  mv -- "$stage/soarm4_description" "$target"
  rmdir -- "$stage"
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd -- "$robot_workspace"
colcon build --symlink-install --packages-select soarm4_description --cmake-clean-cache
set +u
source install/setup.bash
set -u
echo 'SO-ARM4 CHECKED v5.7 — GUI / 엔코더2047 홈자세로 실행합니다.'
ros2 launch soarm4_description display.launch.py
