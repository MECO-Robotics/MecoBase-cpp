#!/usr/bin/env python3
"""Bind a validated archive to its tagged source, tooling and shard run."""
import hashlib
import json
import os
from pathlib import Path
import re

archive = Path('build/recovery-archive/frcBase.lib')
tools_sha = os.environ['GITHUB_SHA']
release_sha = os.environ['RELEASE_COMMIT']
configuration = os.environ['CONFIGURATION']
if (not re.fullmatch('[0-9a-f]{40}', tools_sha) or
        not re.fullmatch('[0-9a-f]{40}', release_sha) or configuration not in ('Debug', 'Release')):
    raise ValueError('Invalid assembly provenance')
shards = json.loads(Path('build/recovery-shards/verified.json').read_text())
if set(shards) != {str(i) for i in range(16)}:
    raise ValueError('Missing verified shards')
for item in shards.values():
    if (not isinstance(item, dict) or not str(item.get('run', '')).isdecimal() or
        not re.fullmatch('sha256:[0-9a-f]{64}', item.get('digest', ''))):
        raise ValueError('Invalid verified shard provenance')
metadata = {'shards': shards, 'shardTools': 'e6125fb135fa91ae92fb501944bba06923f07928',
            'assemblyTools': tools_sha, 'releaseCommit': release_sha, 'configuration': configuration,
            'archiveSha256': hashlib.sha256(archive.read_bytes()).hexdigest()}
Path('build/recovery-archive/assembly.json').write_text(json.dumps(metadata, indent=2)+'\n')
