"""Retrieve a completed development run without dumping frame/log collections."""
import csv
import json
from pathlib import Path
import tarfile
from vm import call

root=Path(__file__).resolve().parents[1]
archive=root/'.local/development-latest.tar.gz'
call('copyFileFromGuestToHost','/tmp/crc-develop.tar.gz',str(archive))
with tarfile.open(archive) as tar:
    for name in tar.getnames():
        parts=Path(name).parts
        assert len(parts)>=2 and parts[0]=='results' and '..' not in parts and not Path(name).is_absolute()
    names=tar.getnames()
    tar.extractall(root,filter='data')
batch=root/Path(names[0])
for path in batch.rglob('telemetry.csv'):
    with path.open() as stream:
        reader=csv.DictReader(stream)
        records=list(reader)
        # Keep the original file untouched if a VM power loss leaves a
        # partial trailing record. Report how much evidence is unusable.
        rows=[r for r in records if None not in r and all(r.get(k) is not None
              for k in reader.fieldnames) and not any('\x00' in v for v in r.values())]
    if not rows:
        print(json.dumps({'batch':str(batch),'run':path.parent.name,'error':'No complete telemetry rows'}))
        continue
    print(json.dumps({'batch':str(batch),'run':path.parent.name,'last':rows[-1],
                       'incomplete_rows':len(records)-len(rows),
                       'states':dict((s,sum(r['state']==s for r in rows)) for s in sorted({r['state'] for r in rows}))}))
