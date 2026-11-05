#!/usr/bin/env python3
"""Verify and download one bounded exact-source Windows recovery archive."""
from datetime import datetime
import hashlib
import io
import json
import os
from pathlib import Path
import re
import sys
import urllib.request
import urllib.parse
import zipfile

API = 'https://api.github.com/repos/MECO-Robotics/MecoBase-cpp/'
TAG = '447b84371ec0f5ce26754756c5b35fd97a43d302'
WORKFLOW_HEAD = '79ab13e8c39f71acd8b6f20c95a430a939e50dab'
HEADERS = {'Accept': 'application/vnd.github+json', 'Authorization': f"Bearer {os.environ['GITHUB_TOKEN']}", 'X-GitHub-Api-Version': '2022-11-28'}

class SafeRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, request, fp, code, message, headers, newurl):
        origin = urllib.parse.urlparse(request.full_url)
        destination = urllib.parse.urlparse(newurl)
        if origin.scheme == 'https' and destination.scheme != 'https':
            raise ValueError('Refusing HTTPS downgrade redirect')
        redirected = super().redirect_request(request, fp, code, message, headers, newurl)
        if redirected and origin.hostname != destination.hostname:
            redirected.remove_header('Authorization')
            redirected.remove_header('authorization')
        return redirected

OPENER = urllib.request.build_opener(SafeRedirect())

def request(path):
    return OPENER.open(urllib.request.Request(API + path, headers=HEADERS), timeout=30)

def api(path):
    with request(path) as response:
        return json.load(response)

def timestamp(value):
    return datetime.fromisoformat(value.replace('Z', '+00:00'))

run_id, config = sys.argv[1:]
if not run_id.isdecimal() or config not in ('Debug', 'Release'):
    raise SystemExit('Invalid recovery run or configuration')
run = api(f'actions/runs/{run_id}')
if (run['event'] != 'workflow_dispatch' or run['head_sha'] != WORKFLOW_HEAD or
        run['path'] != '.github/workflows/recovery-2026-2-14-windows.yml' or
        run['status'] != 'completed' or run['conclusion'] != 'success'):
    raise SystemExit('Recovery run does not match pinned workflow or successful completion')
jobs = api(f'actions/runs/{run_id}/jobs?per_page=100')['jobs']
expected = {f'Recovery Windows {config} shard {i}' for i in range(8)} | {f'Assemble recovery Windows {config}'}
if {j['name'] for j in jobs} != expected:
    raise SystemExit('Recovery jobs do not match expected configuration and shards')
start = timestamp(run['created_at'])
for job in jobs:
    elapsed = (timestamp(job['completed_at']) - start).total_seconds()
    if job['conclusion'] != 'success' or not 0 <= elapsed <= 180:
        raise SystemExit(f"Recovery job failed or exceeded total deadline: {job['name']} {elapsed}")
artifacts = api(f'actions/runs/{run_id}/artifacts?per_page=100')['artifacts']
name = f'meco-base-windowsx86-64-{config.lower()}'
selected = [x for x in artifacts if x['name'] == name and not x['expired']]
if len(selected) != 1 or not re.fullmatch(r'sha256:[0-9a-f]{64}', selected[0].get('digest', '')):
    raise SystemExit('Missing or ambiguous archive digest')
with request(f"actions/artifacts/{selected[0]['id']}/zip") as response:
    data = response.read()
if 'sha256:' + hashlib.sha256(data).hexdigest() != selected[0]['digest']:
    raise SystemExit('Archive artifact digest mismatch')
with zipfile.ZipFile(io.BytesIO(data)) as z:
    if z.namelist() != ['frcBase.lib']:
        raise SystemExit('Unexpected Windows archive artifact contents')
    lib = z.read('frcBase.lib')
if TAG.encode() not in lib:
    raise SystemExit('Windows archive missing tagged revision')
out = Path('build/libs/frcBase/static/windowsx86-64') / config.lower() / 'frcBase.lib'
out.parent.mkdir(parents=True, exist_ok=True)
out.write_bytes(lib)
print(f'Verified {config} run {run_id}, artifact {selected[0]["digest"]}, lib sha256:{hashlib.sha256(lib).hexdigest()}')
