"""Create a compact workspace transfer; excludes credentials and run outputs."""
from pathlib import Path
import tarfile

root = Path(__file__).resolve().parents[1]
out = root/'.local/crc-workspace.tar.gz'
out.parent.mkdir(exist_ok=True)
with tarfile.open(out, 'w:gz') as tar:
    for name in ['src','scripts','docker','tests','analysis','tools','.gitignore','.gitattributes','official_checksums.json',
                 'README.md','PROGRESS.md','REPORT.pdf','VIDEO.md','HUONG_DAN.md']:
        path = root/name
        if path.exists():
            tar.add(path, arcname=name, filter=lambda info: None if '__pycache__' in info.name else info)
print(out)
