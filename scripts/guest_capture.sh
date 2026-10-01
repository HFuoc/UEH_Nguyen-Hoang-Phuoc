#!/usr/bin/env bash
set -euo pipefail
docker exec crc bash -lc 'source /opt/ros/humble/setup.bash; python3 /ws/scripts/capture_sensors.py'
mkdir -p /tmp/crc_live_capture
docker cp crc:/tmp/crc_capture/. /tmp/crc_live_capture/
tar -czf /tmp/crc-capture.tar.gz -C /tmp crc_live_capture
