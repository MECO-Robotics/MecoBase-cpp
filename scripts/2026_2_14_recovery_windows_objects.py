#!/usr/bin/env python3
"""Record and merge exact-tag Windows recovery object shards."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

TAG_SHA = '447b84371ec0f5ce26754756c5b35fd97a43d302'

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def shard_for(name):
    value = 0
    for char in name:
        value = (31 * value + ord(char)) & 0xffffffff
    return (value if value < 2**31 else value - 2**32) % 8

def sources():
    root = Path('src/main/cpp/base')
    result = sorted(p for p in root.rglob('*') if p.is_file() and p.suffix in ('.cpp', '.cc'))
    if len({p.stem.lower() for p in result}) != len(result):
        raise ValueError('Source basenames must be unique to verify object coverage')
    return result

def revision():
    result = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
    if result != TAG_SHA or subprocess.check_output(['git', 'status', '--porcelain'], text=True).strip():
        raise ValueError('Recovery must compile a clean exact-tag checkout')
    return result

def record(index, objroot, options, dest, configuration):
    revision()
    selected = [p for p in sources() if shard_for(p.relative_to('src/main/cpp/base').as_posix()) == index]
    objects = sorted(objroot.rglob('*.obj'))
    if {o.stem.lower() for o in objects} != {p.stem.lower() for p in selected} or len(objects) != len(selected):
        raise ValueError('Compiled object members do not match shard sources')
    header = Path('build/generated/replay/BuildProvenance.h').read_text()
    if f'FRC_BUILD_REVISION "{TAG_SHA}"' not in header or 'FRC_BUILD_DIRTY "false"' not in header:
        raise ValueError('Generated build provenance does not identify the clean tag')
    raw_flags = options.read_bytes()
    # MSVC response files may use a UTF-16 BOM or UTF-16LE without a BOM.
    if raw_flags.startswith((b'\xff\xfe', b'\xfe\xff')):
        flags = raw_flags.decode('utf-16')
    elif b'\x00' in raw_flags[:128]:
        flags = raw_flags.decode('utf-16-le')
    else:
        flags = raw_flags.decode('utf-8', errors='replace')
    # Only compare complete switches; include paths can contain "/Zi" text.
    symbols = re.findall(r'(?<![\w/\\])[/\-]Z(?:7|i|I)(?![\w])', flags)
    print(f'Debug-information switches in response-file order: {symbols}', flush=True)
    if configuration == 'Debug' and (not symbols or symbols[-1].lower() not in ('/z7', '-z7')):
        raise ValueError('Debug compilation must end with embedded-symbol /Z7')
    source_hash = re.search(r'FRC_BUILD_SOURCE_HASH "([0-9a-f]{64})"', header).group(1)
    compiler = subprocess.run(['cl'], capture_output=True, text=True)
    version = (compiler.stderr + compiler.stdout).strip().splitlines()[0]
    dest.mkdir(parents=True, exist_ok=True)
    files = {}
    for obj in objects:
        target = dest / obj.name
        target.write_bytes(obj.read_bytes())
        files[obj.name] = digest(target)
    manifest = {'revision': TAG_SHA, 'shard': index, 'configuration': configuration, 'sourceHash': source_hash,
                'compiler': version, 'optionsHash': digest(options),
                'sources': {p.relative_to('src/main/cpp/base').as_posix(): digest(p) for p in selected},
                'objects': files}
    (dest / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')

def assemble(shards, output, configuration):
    revision()
    expected = {p.relative_to('src/main/cpp/base').as_posix(): digest(p) for p in sources()}
    seen = {}
    all_objects = []
    fingerprints = set()
    for index in range(8):
        folder = shards / str(index)
        data = json.loads((folder / 'manifest.json').read_text())
        if data['revision'] != TAG_SHA or data['shard'] != index or data['configuration'] != configuration:
            raise ValueError('Wrong tag or shard')
        fingerprints.add((data['sourceHash'], data['compiler'], data['optionsHash']))
        for src, content in data['sources'].items():
            if src in seen or shard_for(src) != index or expected.get(src) != content:
                raise ValueError('Overlapping or unexpected source')
            seen[src] = content
        for name, expected_hash in data['objects'].items():
            obj = folder / name
            if digest(obj) != expected_hash:
                raise ValueError('Object digest mismatch')
            all_objects.append(obj.resolve())
        if {Path(p).stem.lower() for p in data['sources']} != {Path(p).stem.lower() for p in data['objects']}:
            raise ValueError('Object coverage does not match sources')
    if seen != expected or len(fingerprints) != 1 or len({p.name.lower() for p in all_objects}) != len(all_objects):
        raise ValueError('Incomplete source coverage, duplicate archive member, or inconsistent flags/toolchain')
    output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(['lib.exe', '/nologo', f'/OUT:{output.resolve()}', *map(str, all_objects)], check=True)
    members = subprocess.check_output(['lib.exe', '/nologo', '/LIST', str(output.resolve())], text=True).splitlines()
    if sorted(Path(p.replace('\\', '/')).name.lower() for p in members if p.strip()) != sorted(p.name.lower() for p in all_objects):
        raise ValueError('Archive member list does not match validated objects')
    data = output.read_bytes()
    source_hash = next(iter(fingerprints))[0]
    if TAG_SHA.encode() not in data or source_hash.encode() not in data:
        raise ValueError('Archive lacks the tagged build revision or source hash')
    if configuration == 'Debug' and any(b'.debug$S' not in p.read_bytes() for p in all_objects):
        raise ValueError('A Debug object lacks embedded symbol sections')
    print(f'Validated {len(all_objects)} unique archive members from the clean tagged source')

parser = argparse.ArgumentParser()
sub = parser.add_subparsers(dest='operation', required=True)
r = sub.add_parser('record'); r.add_argument('--shard', type=int, required=True, choices=range(8)); r.add_argument('--objects', type=Path, required=True); r.add_argument('--options', type=Path, required=True); r.add_argument('--dest', type=Path, required=True); r.add_argument('--configuration', choices=['Debug', 'Release'], required=True)
a = sub.add_parser('assemble'); a.add_argument('--shards', type=Path, required=True); a.add_argument('--output', type=Path, required=True); a.add_argument('--configuration', choices=['Debug', 'Release'], required=True)
args = parser.parse_args()
if args.operation == 'record': record(args.shard, args.objects, args.options, args.dest, args.configuration)
else: assemble(args.shards, args.output, args.configuration)
