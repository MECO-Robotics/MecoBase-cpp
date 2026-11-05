#!/usr/bin/env python3
"""Verify tagged revision and source digest embedded in reused Linux archives."""
import hashlib
from pathlib import Path
import subprocess

TAG_SHA = 'e73610d0fa4eea36bd2051de5d269b68a3bdde15'
if subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip() != TAG_SHA:
    raise SystemExit('Wrong source checkout')
h = hashlib.sha256()
files = sorted((p for root in (Path('src/main'), Path('vendordeps'))
                for p in root.rglob('*') if p.is_file()), key=lambda p: str(p.absolute()))
for path in files:
    h.update(path.as_posix().encode())
    h.update(path.read_bytes())
h.update(Path('build.gradle').read_bytes())
source_hash = h.hexdigest().encode()
root = Path('build/libs/frcBase/static')
for platform in ('linuxathena', 'linuxx86-64'):
    for config in ('debug', 'release'):
        path = root / platform / config / 'libfrcBase.a'
        archive = path.read_bytes()
        if TAG_SHA.encode() not in archive or source_hash not in archive:
            raise SystemExit(f'Archive lacks exact-tag provenance: {path}')
        print(f'Verified embedded revision and source hash: {path}')
