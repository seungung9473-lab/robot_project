#!/usr/bin/env bash
set -e
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID=73
export SOARM_SAM2_DEVICE=cuda
runtime="${SOARM_SAM2_PYTHON:-$workspace/automation/reconstruction/.sam2_gpu/bin/python}"
export SOARM_SAM2_PYTHON="$runtime"
if [[ ! -x "$runtime" ]]; then
  if [[ " $* " == *" --execute "* ]]; then
    echo 'CUDA 런타임 없음. .sam2_gpu 또는 SOARM_SAM2_PYTHON을 준비하세요. CPU로 대체하지 않습니다.' >&2
    exit 1
  fi
  runtime=/usr/bin/python3
fi
exec "$runtime" "$workspace/automation/reconstruction/run_qr_expiry_cycle.py" "$@"
