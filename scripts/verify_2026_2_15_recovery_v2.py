#!/usr/bin/env python3
"""Verify exact 2026.2.15 tag, clean checkout and pinned Windows cache artifact."""
import json
import os
import re
import subprocess
import urllib.request
from pathlib import Path

REPOSITORY = 'https://api.github.com/repos/MECO-Robotics/MecoBase-cpp/'
CACHE_RUN = '42036443481'
CACHE_SHA = 'sha256:0605c4985e400aa236b2d5b649c3351369ec0327f723e575a40acd474ade22f7'
HEADERS = {'Accept':'application/vnd.github+json', 'Authorization':f"Bearer {os.environ['GITHUB_TOKEN']}",
           'X-GitHub-Api-Version':'2022-11-28'}

def api(path):
    with urllib.request.urlopen(urllib.request.Request(REPOSITORY + path, headers=HEADERS), timeout=10) as response:
        return json.load(response)

commit = os.environ['RELEASE_COMMIT']
if not re.fullmatch(r'[0-9a-f]{40}', commit):
    raise SystemExit('Expected a full immutable workflow commit')
checkout = subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
if checkout != commit or subprocess.check_output(['git','status','--porcelain'],text=True).strip():
    raise SystemExit('Recovery must use a clean checkout of the workflow commit')
ref = api('git/ref/tags/v2026.2.15')['object']
# Follow an annotated tag to its commit; never trust its name alone.
for _ in range(4):
    if ref['type'] == 'commit': break
    if ref['type'] != 'tag': raise SystemExit('Release tag does not identify a commit')
    ref = api('git/tags/' + ref['sha'])['object']
if ref['type'] != 'commit' or ref['sha'] != commit:
    raise SystemExit('Workflow commit does not match the published v2026.2.15 tag')
if json.loads(Path('vendor/MecoBase.json').read_text())['version'] != '2026.2.15':
    raise SystemExit('Tagged descriptor is not version 2026.2.15')
if os.environ['REUSE_RUN_ID'] != CACHE_RUN:
    raise SystemExit('Unexpected Windows dependency cache source')
items = api('actions/runs/' + CACHE_RUN + '/artifacts?per_page=100')['artifacts']
matching = [a for a in items if a['name'] == 'studica-windows-maven-2026.0.0' and not a['expired']]
if len(matching) != 1 or matching[0].get('digest') != CACHE_SHA:
    raise SystemExit('Pinned Windows dependency cache missing or changed')
print(f'Verified release tag and source {commit}; Windows dependency cache {CACHE_SHA}')
