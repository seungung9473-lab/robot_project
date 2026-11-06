#!/usr/bin/env bash
set -e
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
backend="${SOARM_DEMO_BACKEND:-${workspace}_mybranch}"
if [[ ! -d "$backend" ]]; then backend="$workspace"; fi
export SOARM_DEMO_BACKEND="$backend"
export PYTHONPATH="$workspace/automation/reconstruction${PYTHONPATH:+:$PYTHONPATH}"
if [[ " $* " == *" --execute "* ]]; then
  source /opt/ros/jazzy/setup.bash
  source "$workspace/install/setup.bash"
  export ROS_DOMAIN_ID=73
fi
runtime="$backend/automation/reconstruction/.sam2_gpu/bin/python"
if [[ ! -x "$runtime" ]]; then
  if [[ " $* " == *" --execute "* || " $* " == *" --interpret "* ]]; then
    echo 'GPU 런타임 없음. SOARM_DEMO_BACKEND를 설치된 작업공간으로 지정하세요.' >&2
    exit 1
  fi
  runtime=/usr/bin/python3
fi
exec "$runtime" "$workspace/automation/reconstruction/tidy_demo.py" "$@"
