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
# The packager supplies and independently resolves the immutable release tag.
TAG = os.environ['RELEASE_COMMIT']
if not re.fullmatch(r'[0-9a-f]{40}', TAG):
    raise ValueError('RELEASE_COMMIT must be an immutable commit SHA')
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

ref = api('git/ref/tags/v2026.2.15')['object']
for _ in range(4):
    if ref['type'] == 'commit': break
    if ref['type'] != 'tag': raise SystemExit('Release tag does not identify a commit')
    ref = api('git/tags/' + ref['sha'])['object']
if ref['type'] != 'commit' or ref['sha'] != TAG:
    raise SystemExit('Recovery release commit does not match the immutable tag')

run_id, config = sys.argv[1:]
if not run_id.isdecimal() or config not in ('Debug', 'Release'):
    raise SystemExit('Invalid recovery run or configuration')
run = api(f'actions/runs/{run_id}')
if run['path'] == '.github/workflows/recovery-2026-2-15-assemble.yml':
    assembly_tools = 'be730b50616f66dba1c2ac4750a363d9d6ad6c53'
    shard_tools = 'e6125fb135fa91ae92fb501944bba06923f07928'
    if (run['event'] != 'workflow_dispatch' or run['head_sha'] != assembly_tools or
            run['status'] != 'completed' or run['conclusion'] != 'success'):
        raise SystemExit('Separate assembly run is not pinned and successful')
    jobs = api(f'actions/runs/{run_id}/jobs?per_page=100')['jobs']
    if len(jobs) != 1 or jobs[0]['name'] != f'Assemble verified Windows {config} shards' or jobs[0]['conclusion'] != 'success':
        raise SystemExit('Unexpected assembly job set or conclusion')
    elapsed = (timestamp(jobs[0]['completed_at']) - timestamp(run['created_at'])).total_seconds()
    if not 0 <= elapsed <= 180:
        raise SystemExit('Separate assembly exceeded queue plus execution deadline')
    artifacts = api(f'actions/runs/{run_id}/artifacts?per_page=100')['artifacts']
    selected = [x for x in artifacts if x['name'] == f'verified-windows-assembly-{config}' and not x['expired']]
    if len(selected) != 1 or not re.fullmatch(r'sha256:[0-9a-f]{64}', selected[0].get('digest', '')):
        raise SystemExit('Missing or ambiguous assembly artifact')
    with request(f"actions/artifacts/{selected[0]['id']}/zip") as response:
        data = response.read()
    if 'sha256:' + hashlib.sha256(data).hexdigest() != selected[0]['digest']:
        raise SystemExit('Assembly artifact digest mismatch')
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        if sorted(z.namelist()) != ['assembly.json', 'frcBase.lib']:
            raise SystemExit('Unexpected separate assembly artifact contents')
        metadata = json.loads(z.read('assembly.json'))
        lib = z.read('frcBase.lib')
    if (metadata.get('assemblyTools') != assembly_tools or metadata.get('shardTools') != shard_tools or
            metadata.get('releaseCommit') != TAG or metadata.get('configuration') != config or
            metadata.get('archiveSha256') != hashlib.sha256(lib).hexdigest() or TAG.encode() not in lib):
        raise SystemExit('Assembly archive or provenance does not match release')
    shard_run_id = metadata.get('shardRun', '')
    if not isinstance(shard_run_id, str) or not shard_run_id.isdecimal():
        raise SystemExit('Invalid source shard run')
    source_run = api(f'actions/runs/{shard_run_id}')
    if (source_run['event'] != 'workflow_dispatch' or source_run['head_sha'] != shard_tools or
            source_run['path'] != '.github/workflows/recovery-2026-2-15-windows-v2.yml' or
            source_run['status'] != 'completed'):
        raise SystemExit('Source shard run does not match pinned tools')
    source_jobs = api(f'actions/runs/{shard_run_id}/jobs?per_page=100')['jobs']
    expected_shards = {f'Recovery Windows {config} shard {i}' for i in range(16)}
    if len(source_jobs) != 17 or {j['name'] for j in source_jobs} != expected_shards | {f'Assemble recovery Windows {config}'}:
        raise SystemExit('Unexpected source shard job set')
    start = timestamp(source_run['created_at'])
    for job in source_jobs:
        if job['name'] not in expected_shards:
            continue  # Its concurrent assembly result is never counted as evidence.
        if job['status'] != 'completed' or job['conclusion'] != 'success' or not job['completed_at']:
            raise SystemExit(f"Source shard failed: {job['name']}")
        elapsed = (timestamp(job['completed_at']) - start).total_seconds()
        if not 0 <= elapsed <= 180:
            raise SystemExit(f"Source shard exceeded deadline: {job['name']}")
    source_artifacts = api(f'actions/runs/{shard_run_id}/artifacts?per_page=100')['artifacts']
    actual = {}
    for i in range(16):
        matches = [a for a in source_artifacts if a['name'] == f'recovery-windows-{config}-{i}' and not a['expired']]
        if len(matches) != 1 or not re.fullmatch(r'sha256:[0-9a-f]{64}', matches[0].get('digest', '')):
            raise SystemExit('Missing or ambiguous source shard artifact')
        actual[str(i)] = matches[0]['digest']
    if metadata.get('shardDigests') != actual:
        raise SystemExit('Source artifact digests differ from verified assembly')
    out = Path('build/libs/frcBase/static/windowsx86-64') / config.lower() / 'frcBase.lib'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(lib)
    print(f'Verified {config} source shards {shard_run_id}, separate assembly {run_id}, archive sha256:{metadata["archiveSha256"]}')
    raise SystemExit(0)
# Recovery v2 is external tooling; the release tag itself remains unchanged.
# This immutable SHA is the reviewed workflow and source-verification tooling.
tools_sha = 'e6125fb135fa91ae92fb501944bba06923f07928'
paths = {'.github/workflows/recovery-2026-2-15-windows.yml': (TAG, 8),
         '.github/workflows/recovery-2026-2-15-windows-v2.yml': (tools_sha, 16)}
policy = paths.get(run['path'])
if (not policy or run['event'] != 'workflow_dispatch' or run['head_sha'] != policy[0] or
        run['status'] != 'completed' or run['conclusion'] != 'success'):
    raise SystemExit('Recovery run does not match pinned workflow or successful completion')
jobs = api(f'actions/runs/{run_id}/jobs?per_page=100')['jobs']
expected = {f'Recovery Windows {config} shard {i}' for i in range(policy[1])} | {f'Assemble recovery Windows {config}'}
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
