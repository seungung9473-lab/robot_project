#!/usr/bin/env bash
set -e
SOARM_WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$SOARM_WORKSPACE_DIR/install/setup.bash"
export ROS_DOMAIN_ID=73
exec /usr/bin/python3 "$SOARM_WORKSPACE_DIR/automation/reconstruction/run_qr_expiry_cycle.py" "$@"
