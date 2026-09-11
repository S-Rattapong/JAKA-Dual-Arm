#!/usr/bin/env bash
set -eo pipefail
WORKSPACE="${HOME}/jaka_ws"
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export JAKA_OPERATOR_MODE=1
cd "${WORKSPACE}"
exec python3 -m uvicorn \
  dual_arm_app.backend.dual_jaka_web_backend:app \
  --host 127.0.0.1 \
  --port 8000
