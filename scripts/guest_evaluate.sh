#!/usr/bin/env bash
# Development harness only. Start-pose perturbations never enter the driver.
set -euo pipefail
cd /home/fish/crc_ws
mkdir -p results
batch="${CRC_BATCH:-$(date -u +%Y%m%dT%H%M%SZ)}"
result_root="results/$batch"
mkdir -p "$result_root"
python3 - <<'PY' > "$result_root/source_sha256.json"
import hashlib,json
from pathlib import Path
print(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in Path('src/crc_solution').rglob('*.py')},indent=2))
PY
source_cmd='source /opt/ros/humble/setup.bash && source /ws_build/install/setup.bash'
run_case() {
  local label="$1"; shift
  bash scripts/run_docker.sh down || true
  docker run -d --name crc --net=host --ipc=host -e DISPLAY=:99 -e LIBGL_ALWAYS_SOFTWARE=1 \
    -v "$PWD/src":/ws/src -v "$PWD/scripts":/ws/scripts -v crc_build:/ws_build \
    crc_sim:humble bash /ws/scripts/headless_entrypoint.sh skycam:=false "$@"
  if ! docker exec crc bash -lc "$source_cmd && python3 /ws/scripts/verify_sensors.py"; then
    docker logs crc > "$result_root/${label}_startup_failure.log" 2>&1
    docker restart crc
    docker exec crc bash -lc "$source_cmd && python3 /ws/scripts/verify_sensors.py"
  fi
  docker exec crc bash -lc "$source_cmd && bash /ws/scripts/evaluate.sh ${CRC_DURATION:-300}"
  mkdir -p "$result_root/$label"
  docker cp crc:/tmp/crc_results "$result_root/$label/"
  docker cp crc:/tmp/crc-driver.log "$result_root/$label/driver.log"
  docker cp crc:/tmp/crc-node-info.txt "$result_root/$label/node-info.txt"
  docker logs crc > "$result_root/$label/simulator.log" 2>&1
}
run_case default_1
if [ "${CRC_PILOT:-0}" = 1 ]; then
  tar -czf /tmp/crc-results.tar.gz "$result_root"
  exit 0
fi
run_case default_2
run_case default_3
run_case shifted_a x:=-4.55 y:=-2.022 yaw:=0.03
run_case shifted_b x:=-4.65 y:=-2.022 yaw:=-0.03
tar -czf /tmp/crc-results.tar.gz "$result_root"
