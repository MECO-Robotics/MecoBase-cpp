#!/usr/bin/env python3
"""Validate release refs and bind staged platform archives to their build commit."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

PLATFORMS = {
    'linux': ['linuxathena/debug/libfrcBase.a', 'linuxathena/release/libfrcBase.a',
              'linuxx86-64/debug/libfrcBase.a', 'linuxx86-64/release/libfrcBase.a'],
    'windows': ['windowsx86-64/debug/frcBase.lib', 'windowsx86-64/release/frcBase.lib'],
}


def validate_tag(ref, descriptor):
    version = json.loads(descriptor.read_text(encoding='utf-8'))['version']
    if ref != f'refs/tags/v{version}':
        raise ValueError(f'Release ref {ref!r} does not match descriptor version {version!r}')


def provenance(platform, static_root, commit):
    return {'platform': platform, 'sourceCommit': commit, 'archives': {
        name: hashlib.sha256((static_root / name).read_bytes()).hexdigest()
        for name in PLATFORMS[platform]}}


def verify(platform, static_root, commit, path):
    if json.loads(path.read_text(encoding='utf-8')) != provenance(platform, static_root, commit):
        raise ValueError(f'{platform} archive provenance does not match staged files and source commit')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('operation', choices=['tag', 'record', 'verify'])
    parser.add_argument('--ref')
    parser.add_argument('--descriptor', type=Path, default=Path('vendor/MecoBase.json'))
    parser.add_argument('--platform', choices=PLATFORMS)
    parser.add_argument('--static-root', type=Path, default=Path('build/libs/frcBase/static'))
    parser.add_argument('--file', type=Path)
    args = parser.parse_args()
    if args.operation == 'tag':
        validate_tag(args.ref, args.descriptor)
        return
    if not args.platform or not args.file:
        parser.error('--platform and --file are required')
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
    if args.operation == 'record':
        args.file.parent.mkdir(parents=True, exist_ok=True)
        args.file.write_text(json.dumps(provenance(args.platform, args.static_root, commit), indent=2) + '\n', encoding='utf-8')
    else:
        verify(args.platform, args.static_root, commit, args.file)


if __name__ == '__main__':
    main()
