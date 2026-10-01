#!/usr/bin/env bash
set -euo pipefail
docker exec crc tail -15 /tmp/crc-driver.log
mkdir -p /tmp/crc-live-results
docker cp crc:/tmp/crc_results/. /tmp/crc-live-results/
tar -czf /tmp/crc-live.tar.gz -C /tmp crc-live-results
