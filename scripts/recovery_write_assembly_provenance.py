#!/usr/bin/env python3
"""Bind a validated archive to its tagged source, tooling and shard run."""
import hashlib
import json
import os
from pathlib import Path
import re

archive = Path('build/recovery-archive/frcBase.lib')
shard_run = os.environ['SHARD_RUN_ID']
tools_sha = os.environ['GITHUB_SHA']
release_sha = os.environ['RELEASE_COMMIT']
configuration = os.environ['CONFIGURATION']
if (not shard_run.isdecimal() or not re.fullmatch('[0-9a-f]{40}', tools_sha) or
        not re.fullmatch('[0-9a-f]{40}', release_sha) or configuration not in ('Debug', 'Release')):
    raise ValueError('Invalid assembly provenance')
digests = json.loads(Path('build/recovery-shards/verified.json').read_text())
if set(digests) != {str(i) for i in range(16)} or any(not re.fullmatch('sha256:[0-9a-f]{64}', x) for x in digests.values()):
    raise ValueError('Missing verified shard digests')
metadata = {'shardDigests': digests, 'shardRun': shard_run, 'shardTools': 'e6125fb135fa91ae92fb501944bba06923f07928',
            'assemblyTools': tools_sha, 'releaseCommit': release_sha, 'configuration': configuration,
            'archiveSha256': hashlib.sha256(archive.read_bytes()).hexdigest()}
Path('build/recovery-archive/assembly.json').write_text(json.dumps(metadata, indent=2)+'\n')
