#!/usr/bin/env python3
"""Verify the exact 2026.2.13 Windows Release artifact from the bounded run."""
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import urllib.request

RUN = 41966076720
WORKFLOW_SHA = '66f8dee2910338768db226772b986a493117416a'
TAG_SHA = 'e73610d0fa4eea36bd2051de5d269b68a3bdde15'
SOURCE_HASH = '82724028ecc956fa01e3c262af5a21145bd82b0e897405b6ea8d91a1bf41e81b'
LIB_HASH = '000ce6a7e191375b1ac0f0ea3eb5f89315fdde9deb1e0374eaf37bd531054710'
EXPECTED = {
    'meco-base-windowsx86-64-release': 'sha256:8f80d5ab3ea03a33467c1b92078d5ba3e78835eda57eeb4ed8558ddd54ef2d71',
    'recovery-windows-Release-0': 'sha256:6f58571aead6074e36f9eff7e86c46181778bd91fd8706227ec3ed4a42a0b4c8',
    'recovery-windows-Release-1': 'sha256:86aa8106a5d3163a7b27dbd16ddc073c0aa6484790b5a6e7e1777039a41da6f9',
    'recovery-windows-Release-2': 'sha256:f02387ecc61d033df15997203449cb8610754e5f93bdca240d22bdb4bd77ec64',
    'recovery-windows-Release-3': 'sha256:a479c1f585d9f6a729c794e28db2c3c7d42a97029ad612d4a0f36d3f28a83dc2',
    'recovery-windows-Release-4': 'sha256:872cc545deaae3b58fe51db089eba883590f5d98a636c561da005bfcba30dfaa',
    'recovery-windows-Release-5': 'sha256:9b0e9d13520ccd45b0a9484370335dcbc7690d8be1f617c21e8677494f1a62b6',
    'recovery-windows-Release-6': 'sha256:bca09223f72ea7b36d393f9b3719836ebff4684d141d63b4281088180ad3fcbc',
    'recovery-windows-Release-7': 'sha256:0ae8cb1029cff5a827aef1937537857a5c77b13b1798a58c52ca9736e4d86acd',
}

def api(path):
    request = urllib.request.Request(f'https://api.github.com/repos/MECO-Robotics/MecoBase-cpp/{path}', headers={
        'Accept': 'application/vnd.github+json', 'Authorization': f"Bearer {os.environ['GITHUB_TOKEN']}",
        'X-GitHub-Api-Version': '2022-11-28'})
    with urllib.request.urlopen(request, timeout=20) as response:
        return json.load(response)

def time(value):
    return datetime.fromisoformat(value.replace('Z', '+00:00'))

if subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip() != TAG_SHA:
    raise SystemExit('Wrong tagged source checkout')
run = api(f'actions/runs/{RUN}')
if (run['event'], run['workflow_id'], run['head_sha'], run['status'], run['conclusion']) != ('workflow_dispatch', 405037436, WORKFLOW_SHA, 'completed', 'failure'):
    raise SystemExit('Release recovery run metadata mismatch')
created = time(run['created_at'])
jobs = api(f'actions/runs/{RUN}/jobs?per_page=100')['jobs']
needed = {f'Recovery Windows Release shard {i}' for i in range(8)} | {'Assemble recovery Windows Release'}
selected = {j['name']: j for j in jobs if j['name'] in needed}
if set(selected) != needed:
    raise SystemExit('Release job matrix incomplete')
for name, job in selected.items():
    if job['conclusion'] != 'success' or (time(job['completed_at']) - created).total_seconds() > 180:
        raise SystemExit(f'Release job failed or exceeded the total budget: {name}')
artifacts = {a['name']: a['digest'] for a in api(f'actions/runs/{RUN}/artifacts?per_page=100')['artifacts'] if not a['expired']}
for name, digest in EXPECTED.items():
    if artifacts.get(name) != digest:
        raise SystemExit(f'Release artifact digest mismatch: {name}')
if len(os.sys.argv) == 2:
    data = Path(os.sys.argv[1]).read_bytes()
    if hashlib.sha256(data).hexdigest() != LIB_HASH or TAG_SHA.encode() not in data or SOURCE_HASH.encode() not in data:
        raise SystemExit('Downloaded Release archive does not match pinned bytes and embedded provenance')
print(f'Verified bounded Windows Release run {RUN}, all shard digests, and exact tag')
