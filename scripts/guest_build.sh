#!/usr/bin/env bash
set -euo pipefail
mkdir -p /home/fish/crc_ws
tar -xzf /tmp/crc-workspace.tar.gz -C /home/fish/crc_ws
chown -R fish:fish /home/fish/crc_ws
cd /home/fish/crc_ws
bash scripts/run_docker.sh build
bash scripts/run_docker.sh compile
bash scripts/run_docker.sh up-headless
docker logs crc --tail 30
