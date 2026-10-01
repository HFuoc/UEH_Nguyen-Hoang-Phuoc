"""Build and verify a portable submission ZIP without local credentials/runs."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

root=Path(__file__).resolve().parents[1]
name='UEH_Nguyen-Hoang-Phuoc'
output=root/(name+'_runtime.zip')
temporary=output.with_suffix('.zip.tmp')
files=[]
for directory in ('src','scripts','docker','tests','tools','analysis/evidence'):
    files.extend(p for p in (root/directory).rglob('*') if p.is_file()
                 and '__pycache__' not in p.parts and p.suffix!='.pyc')
files.extend(p for p in (root/'analysis').glob('*.py'))
files.extend(root/f for f in ('README.md','HUONG_DAN.md','PROGRESS.md','VIDEO.md',
                             'REPORT.pdf','official_checksums.json','.gitignore','.gitattributes'))
files=sorted(set(p for p in files if p.exists()))
subprocess.run([sys.executable,'tools/check_rules.py'],cwd=root,check=True)
manifest={p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
with zipfile.ZipFile(temporary,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
    for p in files:
        archive.write(p,name+'/'+p.relative_to(root).as_posix())
    archive.writestr(name+'/MANIFEST.sha256.json',json.dumps(manifest,indent=2)+'\n')
with zipfile.ZipFile(temporary) as archive:
    assert archive.testzip() is None
    for relative,digest in manifest.items():
        assert hashlib.sha256(archive.read(name+'/'+relative)).hexdigest()==digest
temporary.replace(output)
digest=hashlib.sha256(output.read_bytes()).hexdigest()
output.with_suffix('.zip.sha256').write_text(digest+'  '+output.name+'\n')
print(json.dumps({'zip':str(output),'files':len(files),'bytes':output.stat().st_size,'sha256':digest}))
