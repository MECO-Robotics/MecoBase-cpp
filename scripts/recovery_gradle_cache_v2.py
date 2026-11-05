#!/usr/bin/env python3
"""Validate and restore a same-tool-SHA, exact-tag Windows Gradle cache."""
from datetime import datetime
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import urllib.parse
import urllib.request
import zipfile

api = 'https://api.github.com/repos/MECO-Robotics/MecoBase-cpp/'
headers = {'Accept': 'application/vnd.github+json', 'Authorization': 'Bearer '+os.environ['GITHUB_TOKEN'],
           'X-GitHub-Api-Version': '2022-11-28'}
class SafeRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, request, fp, code, message, headers, newurl):
        old, new = urllib.parse.urlparse(request.full_url), urllib.parse.urlparse(newurl)
        if new.scheme != 'https': raise ValueError('Refusing non-HTTPS redirect')
        result = super().redirect_request(request, fp, code, message, headers, newurl)
        if result and old.hostname != new.hostname:
            result.remove_header('Authorization'); result.remove_header('authorization')
        return result
opener = urllib.request.build_opener(SafeRedirect())
def get(path):
    with opener.open(urllib.request.Request(api+path, headers=headers), timeout=40) as r: return r.read()
def obj(path): return json.loads(get(path))
def timestamp(s): return datetime.fromisoformat(s.replace('Z', '+00:00'))
runid=os.environ['PREWARM_RUN_ID']; tools=os.environ['GITHUB_SHA']
if not runid.isdecimal() or not re.fullmatch('[0-9a-f]{40}',tools): raise ValueError('Invalid prewarm run or tool SHA')
run=obj('actions/runs/'+runid)
if (run['head_sha'] != tools or run['event'] != 'workflow_dispatch' or run['status'] != 'completed' or
    run['conclusion'] != 'success' or run['path'] != '.github/workflows/recovery-2026-2-15-gradle-prewarm.yml'):
    raise ValueError('Prewarm run does not match this immutable recovery tooling')
jobs=obj('actions/runs/'+runid+'/jobs?per_page=100')['jobs']
if len(jobs)!=1 or jobs[0]['name']!='Prepare exact-tag Windows Gradle cache' or jobs[0]['conclusion']!='success':
    raise ValueError('Unexpected prewarm job set or status')
elapsed=(timestamp(jobs[0]['completed_at'])-timestamp(run['created_at'])).total_seconds()
if not 0<=elapsed<=180: raise ValueError('Prewarm exceeded queue plus execution limit')
artifacts=obj('actions/runs/'+runid+'/artifacts?per_page=100')['artifacts']
selected=[a for a in artifacts if a['name']=='recovery-gradle-cache-2026-2-15' and not a['expired']]
if len(selected)!=1 or not re.fullmatch('sha256:[0-9a-f]{64}',selected[0].get('digest','')): raise ValueError('Invalid prewarm artifact')
raw=get('actions/artifacts/'+str(selected[0]['id'])+'/zip')
if 'sha256:'+hashlib.sha256(raw).hexdigest()!=selected[0]['digest']: raise ValueError('Prewarm artifact digest mismatch')
with zipfile.ZipFile(io.BytesIO(raw)) as outer:
    if outer.namelist()!=['gradle-cache.zip']: raise ValueError('Unexpected prewarm artifact members')
    nested=outer.read('gradle-cache.zip')
root=Path(os.environ['GRADLE_USER_HOME']); root.mkdir(parents=True,exist_ok=True)
with zipfile.ZipFile(io.BytesIO(nested)) as cache:
    total=0; seen=set()
    for entry in cache.infolist():
        name=entry.filename; path=PurePosixPath(name)
        if (name.startswith('/') or '\\' in name or ':' in name or any(part in ('.','..') for part in path.parts)
            or name.casefold() in seen or (entry.external_attr >> 16) & 0o170000 == 0o120000):
            raise ValueError('Unsafe cache member '+name)
        seen.add(name.casefold());total+=entry.file_size
        if total>2_000_000_000: raise ValueError('Cache too large')
        target=root.joinpath(*path.parts)
        if entry.is_dir(): target.mkdir(parents=True,exist_ok=True); continue
        target.parent.mkdir(parents=True,exist_ok=True)
        with cache.open(entry) as src,target.open('wb') as dst:
            while data:=src.read(1024*1024): dst.write(data)
print('Verified exact-tool prewarm artifact',selected[0]['digest'])
