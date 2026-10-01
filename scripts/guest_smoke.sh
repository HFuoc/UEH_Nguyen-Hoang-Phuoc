#!/usr/bin/env bash
set -euo pipefail
cd /home/fish/crc_ws
source_cmd='source /opt/ros/humble/setup.bash && source /ws_build/install/setup.bash'
docker exec crc bash -lc "$source_cmd && python3 /ws/scripts/verify_sensors.py"
docker run --rm --net=host -e ROS_DOMAIN_ID=97 -v "$PWD":/workspace \
  crc_sim:humble bash -lc 'source /opt/ros/humble/setup.bash && python3 /workspace/tests/ros_runtime_check.py'
