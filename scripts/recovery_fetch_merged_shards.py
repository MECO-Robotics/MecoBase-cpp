#!/usr/bin/env python3
"""Fetch previously successful exact-tool shards with independently bounded assembly."""
from concurrent.futures import FIRST_COMPLETED, ThreadPoolExecutor, wait
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import time
import urllib.parse
import urllib.request
import zipfile

API = 'https://api.github.com/repos/MECO-Robotics/MecoBase-cpp'
RUN_IDS = os.environ['SHARD_RUN_IDS'].split(',')
if not 1 <= len(RUN_IDS) <= 16 or len(set(RUN_IDS)) != len(RUN_IDS) or any(not x.isdecimal() for x in RUN_IDS):
    raise ValueError('Expected one to sixteen unique comma-separated shard run IDs')
CONFIGURATION = os.environ['CONFIGURATION']
if CONFIGURATION not in ('Debug', 'Release'):
    raise SystemExit('Invalid Windows configuration')
HEADERS = {'Accept': 'application/vnd.github+json',
           'Authorization': f"Bearer {os.environ['GITHUB_TOKEN']}",
           'X-GitHub-Api-Version': '2022-11-28'}

class SafeRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, request, fp, code, message, headers, newurl):
        origin = urllib.parse.urlparse(request.full_url)
        destination = urllib.parse.urlparse(newurl)
        if origin.scheme == 'https' and destination.scheme != 'https':
            raise ValueError('Refusing HTTPS downgrade redirect')
        redirected = super().redirect_request(request, fp, code, message, headers, newurl)
        if redirected and urllib.parse.urlparse(request.full_url).hostname != urllib.parse.urlparse(newurl).hostname:
            redirected.remove_header('Authorization')
            redirected.remove_header('authorization')
        return redirected

OPENER = urllib.request.build_opener(SafeRedirect())

def get_json(path):
    with OPENER.open(urllib.request.Request(API + path, headers=HEADERS), timeout=15) as response:
        return json.load(response)

assembly_run = get_json(f"/actions/runs/{os.environ['GITHUB_RUN_ID']}")
created = datetime.fromisoformat(assembly_run['created_at'].replace('Z', '+00:00'))

def require_time():
    elapsed = (datetime.now(timezone.utc) - created).total_seconds()
    if elapsed < 0 or elapsed > 180:
        raise RuntimeError('Windows recovery exceeded the three-minute total limit')

def download(index, artifact):
    expected = artifact.get('digest', '')
    if not re.fullmatch(r'sha256:[0-9a-f]{64}', expected):
        raise ValueError(f'Shard {index} has no valid GitHub artifact digest')
    request = urllib.request.Request(f"{API}/actions/artifacts/{artifact['id']}/zip", headers=HEADERS)
    output = Path('build/recovery-shards') / str(index)
    output.mkdir(parents=True, exist_ok=False)
    tmp = output / 'download.zip'
    h = hashlib.sha256()
    with OPENER.open(request, timeout=30) as source, tmp.open('wb') as target:
        while chunk := source.read(1024 * 1024):
            require_time()
            h.update(chunk)
            target.write(chunk)
    if 'sha256:' + h.hexdigest() != expected:
        raise ValueError(f'Shard {index} artifact digest mismatch')
    with zipfile.ZipFile(tmp) as archive:
        seen = set()
        for item in archive.infolist():
            name = item.filename
            folded = name.casefold()
            base = name.split('.', 1)[0].upper()
            reserved = {'CON', 'PRN', 'AUX', 'NUL', *(f'COM{i}' for i in range(1, 10)), *(f'LPT{i}' for i in range(1, 10))}
            if (item.is_dir() or not re.fullmatch(r'[A-Za-z0-9_-]+(?:\.[A-Za-z0-9_-]+)+', name)
                    or folded in seen or base in reserved
                    or not (name == 'manifest.json' or folded.endswith('.obj'))):
                raise ValueError(f'Unsafe or unexpected shard member: {name!r}')
            seen.add(folded)
            with archive.open(item) as source, (output / name).open('wb') as target:
                while chunk := source.read(1024 * 1024):
                    target.write(chunk)
        if 'manifest.json' not in seen:
            raise ValueError(f'Shard {index} is missing its manifest')
    tmp.unlink()
    print(f'Verified downloaded shard {CONFIGURATION}/{index}: {expected}', flush=True)

selected = {}
for run_id in RUN_IDS:
    require_time()
    run = get_json(f'/actions/runs/{run_id}')
    if (run['status'] != 'completed' or run['event'] != 'workflow_dispatch' or
        run['head_sha'] != 'e6125fb135fa91ae92fb501944bba06923f07928' or
        run['path'] != '.github/workflows/recovery-2026-2-15-windows-v2.yml'):
        raise ValueError('Shard run does not match pinned recovery tools: '+run_id)
    started = datetime.fromisoformat(run['created_at'].replace('Z', '+00:00'))
    jobs = get_json(f'/actions/runs/{run_id}/jobs?per_page=100')['jobs']
    names = {f'Recovery Windows {CONFIGURATION} shard {i}' for i in range(16)}
    if len(jobs) != 17 or {j['name'] for j in jobs} != names | {f'Assemble recovery Windows {CONFIGURATION}'}:
        raise ValueError('Shard job set does not match pinned workflow: '+run_id)
    by_name = {j['name']: j for j in jobs}
    result = get_json(f'/actions/runs/{run_id}/artifacts?per_page=100')
    live = [item for item in result['artifacts'] if not item['expired']]
    artifacts = {item['name']: item for item in live}
    if len(artifacts) != len(live): raise ValueError('Ambiguous artifact names')
    for index in range(16):
        if index in selected: continue
        name = f'Recovery Windows {CONFIGURATION} shard {index}'
        job = by_name[name]
        if job['status'] != 'completed' or job['conclusion'] != 'success' or not job['completed_at']:
            continue
        elapsed = (datetime.fromisoformat(job['completed_at'].replace('Z', '+00:00')) - started).total_seconds()
        if not 0 <= elapsed <= 180: continue
        artifact = artifacts.get(f'recovery-windows-{CONFIGURATION}-{index}')
        if artifact and re.fullmatch(r'sha256:[0-9a-f]{64}', artifact.get('digest', '')):
            selected[index] = (run_id, artifact)
if set(selected) != set(range(16)):
    raise ValueError('Missing successful, timely shard indices: '+str(sorted(set(range(16))-set(selected))))
with ThreadPoolExecutor(max_workers=4) as executor:
    pending = [executor.submit(download, index, selected[index][1]) for index in range(16)]
    remaining = set(pending)
    while remaining:
        require_time()
        completed, remaining = wait(remaining, timeout=1, return_when=FIRST_COMPLETED)
        for future in completed: future.result()
require_time()
provenance = {str(index): {'run': selected[index][0], 'digest': selected[index][1]['digest']} for index in range(16)}
Path('build/recovery-shards/verified.json').write_text(json.dumps(provenance, indent=2)+'\n')
print(f'All {CONFIGURATION} shard artifacts verified from individually successful jobs')
