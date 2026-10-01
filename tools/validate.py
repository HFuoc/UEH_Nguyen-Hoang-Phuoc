"""Record actual local checks with a source fingerprint for the report."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone

root=Path(__file__).resolve().parents[1]
checks=[]
for args in (['-m','unittest','discover','-s','tests','-v'],['tools/check_rules.py']):
    result=subprocess.run([sys.executable,*args],cwd=root,capture_output=True,text=True)
    checks.append({'command':'python '+' '.join(args),'exit_code':result.returncode,
                   'output':result.stdout+result.stderr})
    print(checks[-1]['output'])
sources={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
         for p in sorted((root/'src/crc_solution').rglob('*.py'))}
out=root/'analysis/generated';out.mkdir(exist_ok=True,parents=True)
(out/'validation.json').write_text(json.dumps({'utc':datetime.now(timezone.utc).isoformat(),
                                             'checks':checks,'sources':sources},indent=2)+'\n')
sys.exit(any(c['exit_code'] for c in checks))
