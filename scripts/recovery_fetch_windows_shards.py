#!/usr/bin/env python3
"""Fetch same-run recovery shards as they appear, verifying GitHub ZIP digests."""
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
RUN_ID = os.environ['GITHUB_RUN_ID']
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

run = get_json(f'/actions/runs/{RUN_ID}')
created = datetime.fromisoformat(run['created_at'].replace('Z', '+00:00'))

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

pending = {}
with ThreadPoolExecutor(max_workers=4) as executor:
    while len(pending) != 8:
        require_time()
        result = get_json(f'/actions/runs/{RUN_ID}/artifacts?per_page=100')
        artifacts = {item['name']: item for item in result['artifacts'] if not item['expired']}
        for index in range(8):
            name = f'recovery-windows-{CONFIGURATION}-{index}'
            if index not in pending and name in artifacts:
                pending[index] = executor.submit(download, index, artifacts[name])
        for future in pending.values():
            if future.done():
                future.result()
        if len(pending) != 8:
            time.sleep(2)
    remaining = set(pending.values())
    while remaining:
        require_time()
        completed, remaining = wait(remaining, timeout=1, return_when=FIRST_COMPLETED)
        for future in completed:
            future.result()
require_time()
print(f'All {CONFIGURATION} shard artifacts verified')
