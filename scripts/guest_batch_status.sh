#!/usr/bin/env bash
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
import csv,json
batches=sorted(Path('/home/fish/crc_ws/results').glob('20*'))
batch=batches[-1]
done=[]
for source in sorted(batch.rglob('telemetry.csv')):
    rows=list(csv.DictReader(source.open()))
    last=rows[-1]
    done.append({'case':source.parents[2].name,'distance_m':round(float(last['distance_odom_m']),3),'state':last['state']})
print(json.dumps({'batch':batch.name,'completed':done}),flush=True)
PY
docker exec crc bash -c 'for file in /tmp/crc_results/*/telemetry.csv; do [ -f "$file" ] && tail -1 "$file"; done' || true
