#!/usr/bin/env bash
set -e
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--help" ]]; then
  echo "Usage: ./run_prepare_demo.sh"
  echo "카메라·RViz·제어를 준비하고 턴테이블 토크를 켭니다. 분류 시연은 별도 명령으로 시작합니다."
  exit 0
fi
if [[ $# != 0 ]]; then
  echo "Usage: ./run_prepare_demo.sh" >&2
  exit 2
fi
cd "$workspace"
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
export ROS_DOMAIN_ID=73
export SOARM_SAM2_DEVICE=cuda
export PYTHONPATH="$workspace/automation/reconstruction${PYTHONPATH:+:$PYTHONPATH}"
runtime="${SOARM_SAM2_PYTHON:-$workspace/automation/reconstruction/.sam2_gpu/bin/python}"
if [[ ! -x "$runtime" ]]; then
  echo "CUDA Python을 찾을 수 없습니다. .sam2_gpu 또는 SOARM_SAM2_PYTHON을 확인하세요." >&2
  exit 1
fi
exec "$runtime" "$workspace/automation/reconstruction/prepare_demo.py"
