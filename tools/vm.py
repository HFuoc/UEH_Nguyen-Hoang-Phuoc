"""Small VMware guest bridge. Credentials are read only from the environment."""
import argparse
import base64
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import uuid

VMRUN = r'D:\Robot 2\vmrun.exe'
VMX = r'D:\MAY AO\Ubuntu 64-bit.vmx'


def call(action, *args):
    command = [VMRUN, '-gu', os.environ.get('CRC_VM_USER', 'fish'),
               '-gp', os.environ['CRC_VM_PASSWORD'], action, VMX, *args]
    p = subprocess.run(command, capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError(p.stdout + p.stderr)
    return p.stdout


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['run', 'push', 'pull'])
    parser.add_argument('source')
    parser.add_argument('destination', nargs='?')
    parser.add_argument('--sudo', action='store_true')
    parser.add_argument('--background', action='store_true')
    parser.add_argument('--log', default='/tmp/crc-task.log')
    args = parser.parse_args()
    if args.action in ('push', 'pull'):
        action = 'copyFileFromHostToGuest' if args.action == 'push' else 'copyFileFromGuestToHost'
        print(call(action, args.source, args.destination))
        return
    local = Path('.local'); local.mkdir(exist_ok=True)
    task = '/tmp/crc-' + uuid.uuid4().hex
    call('copyFileFromHostToGuest', str(Path(args.source).resolve()), task+'.sh')
    invoke = '/bin/bash ' + task + '.sh'
    if args.sudo:
        invoke = "printf '%s\\n' \"$1\" | sudo -S -p '' /bin/bash " + task + '.sh'
    runner = local/'guest-runner.sh'
    if not args.log.startswith('/tmp/') or any(c not in '/-_.0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ' for c in args.log):
        raise ValueError('Log must be a simple /tmp/ path')
    runner.write_text('#!/bin/bash\n(' + invoke + ') > ' + args.log + ' 2>&1\nprintf "\\nEXIT_CODE=%s\\n" "$?" >> ' + args.log + '\n', newline='\n')
    call('copyFileFromHostToGuest', str(runner.resolve()), task+'-runner.sh')
    options = ['-noWait'] if args.background else []
    options += ['/bin/bash', task+'-runner.sh']
    if args.sudo:
        options += [os.environ['CRC_VM_PASSWORD']]
    call('runProgramInGuest', *options)
    if args.background:
        print('Started; log: '+args.log)
    else:
        call('copyFileFromGuestToHost', args.log, str((local/'guest.log').resolve()))
        print((local/'guest.log').read_text(errors='replace'))


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
