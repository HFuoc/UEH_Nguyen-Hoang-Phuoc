"""Copy a completed evaluation batch's small reproducibility set into Git."""
import argparse
import json
from pathlib import Path
import shutil

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
    for name in ['node-info.txt','driver.log','simulator.log']:
        if (case/name).exists(): shutil.copy2(case/name,target/name)
    (target/'provenance.json').write_text(json.dumps({'case':case.name,'raw_source':str(source),
        'note':'CSV is complete; image set is a small selected subset. No official score is inferred.'},indent=2)+'\n')
fingerprint=args.batch/'source_sha256.json'
if fingerprint.exists(): shutil.copy2(fingerprint,out/fingerprint.name)
print(out)
