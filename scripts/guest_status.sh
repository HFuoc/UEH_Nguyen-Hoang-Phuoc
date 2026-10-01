#!/usr/bin/env bash
set -u
df -h /
docker ps -a --format '{{.Names}} {{.Status}}'
docker images --format '{{.Repository}}:{{.Tag}} {{.Size}}'
docker logs crc --tail 15 2>/dev/null || true
