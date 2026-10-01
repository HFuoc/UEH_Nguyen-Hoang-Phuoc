#!/usr/bin/env bash
set -euo pipefail
out="/home/fish/crc_ws/results/development_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$out"
docker cp crc:/tmp/crc_results "$out/"
docker cp crc:/tmp/crc-driver.log "$out/driver.log"
docker logs crc > "$out/simulator.log" 2>&1
docker stop crc
