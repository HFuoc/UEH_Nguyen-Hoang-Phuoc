#!/usr/bin/env bash
set -euo pipefail
# End only this task's evaluation harnesses; retain the partial evidence.
python3 - <<'PY'
import os,signal
from pathlib import Path
for process in Path('/proc').iterdir():
    if not process.name.isdigit() or int(process.name) in (os.getpid(),os.getppid()):continue
    try:
        args=(process/'cmdline').read_bytes().split(b'\0')
        if args[0] not in (b'/bin/bash',b'bash'):continue
        for arg in args[1:]:
            if arg.startswith(b'/tmp/crc-') and arg.endswith(b'.sh'):
                script=Path(os.fsdecode(arg)).read_text()
                if 'run_case default_1' in script and 'run_case shifted_b' in script:
                    os.kill(int(process.name),signal.SIGTERM)
                    print('Stopped evaluation harness PID',process.name)
    except (FileNotFoundError,ProcessLookupError,PermissionError):pass
PY
out="/home/fish/crc_ws/results/interrupted_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$out"
docker cp crc:/tmp/crc_results "$out/" || true
docker cp crc:/tmp/crc-driver.log "$out/driver.log" || true
docker logs crc > "$out/simulator.log" 2>&1
docker stop crc
