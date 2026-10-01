"""Create the participant's private repository using the local Git credential helper.

Tokens are never printed, written to disk, or passed on a shell command line.
"""
import json
import subprocess
import urllib.request
import urllib.error

REPO='UEH_Nguyen-Hoang-Phuoc'
credential=subprocess.run(['git','credential','fill'],input='protocol=https\nhost=github.com\n\n',
                          text=True,capture_output=True,check=True)
fields=dict(line.split('=',1) for line in credential.stdout.splitlines() if '=' in line)
token=fields['password']
def api(path, data=None):
    req=urllib.request.Request('https://api.github.com'+path,
        data=json.dumps(data).encode() if data is not None else None,
        headers={'Authorization':'Bearer '+token,'Accept':'application/vnd.github+json',
                 'X-GitHub-Api-Version':'2022-11-28','User-Agent':'CRC-local-setup'})
    with urllib.request.urlopen(req,timeout=30) as response: return json.load(response)
user=api('/user')['login']
try:
    repo=api('/repos/'+user+'/'+REPO)
except urllib.error.HTTPError as exc:
    if exc.code!=404: raise
    repo=api('/user/repos',{'name':REPO,'private':True,'description':'UEH CRC 2026 sensor-only driving solution','auto_init':False})
if not repo['private']:
    raise SystemExit('Existing repository is public. No push performed.')
print(json.dumps({'url':repo['html_url'],'private':repo['private'],'owner':user}))
remotes=subprocess.run(['git','remote'],capture_output=True,text=True,check=True).stdout.split()
if 'origin' not in remotes:
    subprocess.run(['git','remote','add','origin',repo['clone_url']],check=True)
else:
    actual=subprocess.run(['git','remote','get-url','origin'],capture_output=True,text=True,check=True).stdout.strip()
    if actual!=repo['clone_url']: raise SystemExit('Existing origin differs; retained unchanged.')
