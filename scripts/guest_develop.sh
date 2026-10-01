#!/usr/bin/env bash
# Isolated development cases; launch poses belong only to this test harness.
set -euo pipefail
cd /home/fish/crc_ws
label="${CRC_LABEL:-development_$(date -u +%Y%m%dT%H%M%SZ)}"
out="results/$label"
if [ -e "$out" ]; then
  echo "Refusing to overwrite existing evidence: $out" >&2
  exit 1
fi
mkdir -p "$out/crc_results"
python3 - <<'PY' > "$out/case.json"
import json,os
print(json.dumps({key:os.environ.get(key,default) for key,default in
    [('CRC_LABEL','development'),('CRC_X','auto'),('CRC_Y','auto'),('CRC_YAW','0.0'),
     ('CRC_DURATION','120'),('CRC_SEED','0'),('CRC_PED_TRIGGER','0.0')]},indent=2))
PY
python3 - <<'PY' > "$out/source_sha256.json"
import hashlib,json
from pathlib import Path
print(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in Path('src/crc_solution').rglob('*') if p.is_file() and '__pycache__' not in str(p)},indent=2))
PY
if docker container inspect crc >/dev/null 2>&1; then
  bash scripts/run_docker.sh down
fi
collect_evidence() {
  status=$?
  trap - EXIT
  set +e
  docker cp crc:/tmp/crc-driver.log "$out/driver.log"
  docker cp crc:/tmp/crc-node-info.txt "$out/node-info.txt"
  docker cp crc:/tmp/crc-evaluation.json "$out/evaluation.json"
  docker logs crc > "$out/simulator.log" 2>&1
  printf '{"exit_code":%s}\n' "$status" > "$out/completion.json"
  tar -czf /tmp/crc-develop.tar.gz "$out"
  tail -12 "$out/driver.log"
  exit "$status"
}
trap collect_evidence EXIT
docker run -d --name crc --net=host --ipc=host -e DISPLAY=:99 -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v "$PWD/src":/ws/src -v "$PWD/scripts":/ws/scripts -v crc_build:/ws_build \
  -v "$PWD/$out/crc_results":/tmp/crc_results \
  crc_sim:humble bash /ws/scripts/headless_entrypoint.sh skycam:=false \
  x:="${CRC_X:-auto}" y:="${CRC_Y:-auto}" yaw:="${CRC_YAW:-0.0}" \
  traffic_seed:="${CRC_SEED:-0}" ped_trigger:="${CRC_PED_TRIGGER:-0.0}"
docker exec crc bash -lc 'source /opt/ros/humble/setup.bash; python3 /ws/scripts/verify_sensors.py'
docker exec crc bash /ws/scripts/evaluate.sh "${CRC_DURATION:-120}"
docker exec crc bash -lc 'source /opt/ros/humble/setup.bash; python3 /ws/scripts/capture_sensors.py'
docker cp crc:/tmp/crc_capture "$out/"
