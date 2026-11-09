#!/usr/bin/env bash
set -e
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
backend="${SOARM_DEMO_BACKEND:-$workspace}"
export SOARM_DEMO_BACKEND="$backend"
export SOARM_SAM2_DEVICE=cuda
runtime="${SOARM_SAM2_PYTHON:-$backend/automation/reconstruction/.sam2_gpu/bin/python}"
export SOARM_SAM2_PYTHON="$runtime"
export PYTHONPATH="$workspace/automation/reconstruction${PYTHONPATH:+:$PYTHONPATH}"
if [[ " $* " == *" --execute "* ]]; then
  source /opt/ros/jazzy/setup.bash
  source "$workspace/install/setup.bash"
  export ROS_DOMAIN_ID=73
fi
if [[ ! -x "$runtime" ]]; then
  if [[ " $* " == *" --execute "* || " $* " == *" --interpret "* ]]; then
    echo 'CUDA 런타임 없음. .sam2_gpu 또는 SOARM_SAM2_PYTHON을 준비하세요. CPU로 대체하지 않습니다.' >&2
    exit 1
  fi
  runtime=/usr/bin/python3
fi
exec "$runtime" "$workspace/automation/reconstruction/tidy_demo.py" "$@"
