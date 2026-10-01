"""Copy a completed evaluation batch's small reproducibility set into Git."""
import argparse
import json
from pathlib import Path
import shutil
import hashlib

parser=argparse.ArgumentParser()
parser.add_argument('batch',type=Path)
args=parser.parse_args()
out=Path(__file__).resolve().parent/'evidence'/args.batch.name
out.mkdir(parents=True,exist_ok=True)
for source in args.batch.rglob('telemetry.csv'):
    target=out/source.parent.name
    target.mkdir(exist_ok=True)
    shutil.copy2(source,target/source.name)
    frames=sorted(source.parent.glob('*_FOLLOW.jpg'))
    if frames:
        chosen=frames[-1]
        shutil.copy2(chosen,target/chosen.name)
        raw=chosen.with_name(chosen.name.replace('_FOLLOW.jpg','_camera.jpg'))
        if raw.exists(): shutil.copy2(raw,target/raw.name)
    case=source.parents[2]
    for name in ['node-info.txt','driver.log','simulator.log','case.json','completion.json','evaluation.json']:
        if (case/name).exists(): shutil.copy2(case/name,target/name)
    (target/'provenance.json').write_text(json.dumps({'case':case.name,'raw_source':source.as_posix(),
        'note':'CSV is complete; image set is a small selected subset. No official score is inferred.'},indent=2)+'\n')
fingerprint=args.batch/'source_sha256.json'
if fingerprint.exists():
    shutil.copy2(fingerprint,out/fingerprint.name)
    recorded=json.loads(fingerprint.read_text())
    root=Path(__file__).resolve().parents[1]
    mismatches=[path for path,digest in recorded.items()
                if not (root/path).exists() or hashlib.sha256((root/path).read_bytes()).hexdigest()!=digest]
    if mismatches: raise SystemExit('Source mismatch for evaluated batch: '+', '.join(mismatches))
audit=[]
for path in out.rglob('node-info.txt'):
    graph=path.read_text()
    forbidden=['/model_states','/link_states','/sky_cam/','/traffic_lights','/traffic_light/',
               '/automobile/semaphores','/get_entity_state','/set_entity_state','/spawn_entity','/delete_entity']
    passed=all(x in graph for x in ['/camera/image_raw','/scan','/odom','/cmd_vel']) and not any(x in graph for x in forbidden)
    audit.append({'file':path.as_posix(),'pass':passed})
(out/'graph_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
if audit and not all(a['pass'] for a in audit): raise SystemExit('Live graph audit failed')
print(out)
