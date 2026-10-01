#!/usr/bin/env bash
set -euo pipefail
latest=$(docker exec crc bash -c 'find /tmp/crc_results -name "*_camera.jpg" | sort | tail -1')
docker cp "crc:$latest" /tmp/crc-latest.jpg
docker exec crc bash -c 'for file in /tmp/crc_results/*/telemetry.csv; do tail -1 "$file"; done'
