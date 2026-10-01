#!/usr/bin/env bash
# Development harness only. Start-pose perturbations never enter the driver.
set -euo pipefail
cd /home/fish/crc_ws
mkdir -p results
source_cmd='source /opt/ros/humble/setup.bash && source /ws_build/install/setup.bash'
run_case() {
  local label="$1"; shift
  bash scripts/run_docker.sh down || true
  docker run -d --name crc --net=host --ipc=host -e DISPLAY=:99 \
    -v "$PWD/src":/ws/src -v "$PWD/scripts":/ws/scripts -v crc_build:/ws_build \
    crc_sim:humble bash /ws/scripts/headless_entrypoint.sh "$@"
  docker exec crc bash -lc "$source_cmd && python3 /ws/scripts/verify_sensors.py"
  docker exec crc bash -lc "$source_cmd && bash /ws/scripts/evaluate.sh ${CRC_DURATION:-300}"
  mkdir -p "results/$label"
  docker cp crc:/tmp/crc_results "results/$label/"
  docker cp crc:/tmp/crc-driver.log "results/$label/driver.log"
  docker logs crc > "results/$label/simulator.log" 2>&1
}
run_case default_1
run_case default_2
run_case default_3
run_case shifted_a x:=-4.55 y:=-2.022 yaw:=0.03
run_case shifted_b x:=-4.65 y:=-2.022 yaw:=-0.03
tar -czf /tmp/crc-results.tar.gz results
