#!/usr/bin/env bash
set -euo pipefail
cd /home/fish/crc_ws
docker run --rm -e ROS_DOMAIN_ID=97 -v "$PWD":/workspace crc_sim:humble \
  bash -lc 'source /opt/ros/humble/setup.bash; cd /workspace; python3 -m unittest discover -s tests -v; python3 tests/ros_runtime_check.py'
bash scripts/guest_clean_check.sh
