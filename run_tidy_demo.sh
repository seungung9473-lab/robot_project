#!/usr/bin/env bash
set -e
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -eq 0 || ( $# -eq 1 && "$1" == "--prepare" ) ]]; then
  echo "지능형 로봇대회 시연 · 장치 준비"
  exec "$workspace/run_prepare_demo.sh"
fi
if [[ $# -eq 1 && "$1" == "--help" ]]; then
  echo '인자 없이 실행하거나 --prepare를 주면 시연 장치를 준비합니다.'
fi
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
  export SOARM_SKIP_SUCCESS_EVALUATION=1
  echo "지능형 로봇대회 시연 · 분류 실행"
fi
if [[ ! -x "$runtime" ]]; then
  if [[ " $* " == *" --execute "* || " $* " == *" --interpret "* ]]; then
    echo 'CUDA 런타임 없음. .sam2_gpu 또는 SOARM_SAM2_PYTHON을 준비하세요. CPU로 대체하지 않습니다.' >&2
    exit 1
  fi
  runtime=/usr/bin/python3
fi
exec "$runtime" "$workspace/automation/reconstruction/tidy_demo.py" "$@"
