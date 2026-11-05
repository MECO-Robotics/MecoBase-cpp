#!/usr/bin/env python3
"""Verify the live Base descriptor and all checksummed Maven files concurrently."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import urllib.request

SITE = 'https://meco-robotics.github.io/MecoBase-cpp'
def fetch(url):
    with urllib.request.urlopen(url, timeout=30) as response:
        return response.read()

expected = json.loads(Path('vendor/MecoBase.json').read_text())
descriptor_bytes = fetch(SITE + '/MecoBase.json')
descriptor = json.loads(descriptor_bytes)
if descriptor != expected or json.loads(fetch(descriptor['jsonUrl'])) != expected:
    raise ValueError('Public, tagged and checkout descriptors differ')
manifest = json.loads(fetch(SITE + '/SHA256SUMS.json'))['artifacts']
if hashlib.sha256(descriptor_bytes).hexdigest() != manifest.get('MecoBase.json'):
    raise ValueError('Public descriptor checksum mismatch')
version = descriptor['version']
dependency = next(d for d in descriptor['cppDependencies'] if d['artifactId'] == 'meco-base-cpp')
if dependency['version'] != version:
    raise ValueError('Maven coordinate does not match descriptor version')
prefix = f'maven/com/mecorobotics/frc/meco-base-cpp/{version}/meco-base-cpp-{version}'
required = {prefix + '.pom', prefix + '-headers.zip', prefix + '-sources.zip'}
required.update(f'{prefix}-{platform}{variant}.zip' for platform in dependency['binaryPlatforms'] for variant in ('static', 'staticdebug'))
if not required <= manifest.keys():
    raise ValueError('Required current Maven files are missing from checksum manifest')
items = [(p, h) for p, h in manifest.items() if p.startswith('maven/')]
def verify(item):
    path, digest = item
    if hashlib.sha256(fetch(SITE + '/' + path)).hexdigest() != digest:
        raise ValueError(f'Checksum mismatch: {path}')
with ThreadPoolExecutor(max_workers=12) as pool:
    list(pool.map(verify, items))
print(f'Verified Base {version}, descriptor, and {len(items)} Maven artifact checksums')
