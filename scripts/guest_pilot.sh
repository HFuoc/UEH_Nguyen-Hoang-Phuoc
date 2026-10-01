#!/usr/bin/env bash
set -euo pipefail
export CRC_PILOT=1 CRC_DURATION=120
bash /home/fish/crc_ws/scripts/guest_evaluate.sh
