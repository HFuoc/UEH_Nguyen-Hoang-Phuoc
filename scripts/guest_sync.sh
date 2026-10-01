#!/usr/bin/env bash
set -euo pipefail
tar -xzf /tmp/crc-workspace.tar.gz -C /home/fish/crc_ws
chown -R fish:fish /home/fish/crc_ws
