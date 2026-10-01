"""Verify protected official assets and audit the submitted controller source."""
import ast
import hashlib
import json
from pathlib import Path
import re
import sys

root=Path(__file__).resolve().parents[1]
failures=[]
checks=json.loads((root/'official_checksums.json').read_text())
for relative,expected in checks.items():
    path=root/relative
    if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
        failures.append('Protected asset changed: '+relative)
banned=re.compile(r'/(?:traffic_lights|traffic_light/|automobile/semaphores|sky_cam/|model_states|link_states|get_entity_state|set_entity_state|spawn_entity|delete_entity)')
for path in (root/'src/crc_solution').rglob('*.py'):
    text=path.read_text()
    tree=ast.parse(text)
    if banned.search(text): failures.append('Forbidden interface in '+str(path))
    for node in ast.walk(tree):
        if isinstance(node,ast.Constant) and isinstance(node.value,str):
            if any(marker in node.value for marker in ('crc_sim/worlds','crc_sim/models','crc_sim/config')):
                failures.append('Runtime simulator asset access in '+str(path))
print(f'Protected files checked: {len(checks)}')
print('\n'.join(failures) if failures else 'Static rules audit: PASS (also inspect the live ROS graph).')
sys.exit(bool(failures))
