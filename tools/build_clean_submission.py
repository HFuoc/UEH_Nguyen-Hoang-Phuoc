"""Package only the deliverable and its reproducible evidence."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[1]
NAME='UEH_Nguyen-Hoang-Phuoc'
files=[]
for directory in ('src/crc_solution','tests','analysis/evidence/full_v15','analysis/evidence/video_v15'):
    files.extend(p for p in (ROOT/directory).rglob('*') if p.is_file()
                 and '__pycache__' not in p.parts and p.suffix!='.pyc')
files.extend(ROOT/name for name in ('README.md','REPORT.pdf','VIDEO.md','SIMULATION_DEMO.mp4',
                                  'analysis/submission_figures.py'))
missing=[str(p) for p in files if not p.exists()]
if missing: raise FileNotFoundError(missing)
out=ROOT/(NAME+'_submission.zip')
manifest={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(files))}
with zipfile.ZipFile(out,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for name in manifest: z.write(ROOT/name,NAME+'/'+name)
    z.writestr(NAME+'/MANIFEST.sha256.json',json.dumps(manifest,indent=2)+'\n')
with zipfile.ZipFile(out) as z:
    assert z.testzip() is None
    for name,digest in manifest.items():
        assert hashlib.sha256(z.read(NAME+'/'+name)).hexdigest()==digest
digest=hashlib.sha256(out.read_bytes()).hexdigest()
out.with_suffix('.zip.sha256').write_text(digest+'  '+out.name+'\n')
print(json.dumps({'file':str(out),'files':len(manifest),'bytes':out.stat().st_size,'sha256':digest}))
