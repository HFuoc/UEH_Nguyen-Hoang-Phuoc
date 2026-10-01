#!/usr/bin/env bash
set -euo pipefail
cd /home/fish/crc_ws
mkdir -p results/pre_trial
docker cp crc:/tmp/crc_results/. results/pre_trial/ 2>/dev/null || true
docker restart crc
docker exec crc bash -lc 'source /opt/ros/humble/setup.bash && python3 /ws/scripts/verify_sensors.py'
docker exec crc bash /ws/scripts/evaluate.sh 60
mkdir -p results/trial
docker cp crc:/tmp/crc_results results/trial/
docker cp crc:/tmp/crc-driver.log results/trial/driver.log
tar -czf /tmp/crc-trial.tar.gz results/trial
tail -15 results/trial/driver.log
