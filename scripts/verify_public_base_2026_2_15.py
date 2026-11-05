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
manifest_document = json.loads(fetch(SITE + '/SHA256SUMS.json'))
if manifest_document.get('descriptorVersion') != '2026.2.15' or manifest_document.get('sourceCommit') != '8a75c1a6ae46630a67eb209da06136502fe10607':
    raise ValueError('Live manifest does not identify the exact 2026.2.15 release source')
manifest = manifest_document['artifacts']
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
# Every previously published Base 2026.2.x coordinate must remain available,
# including source/header archives and both configurations for every platform.
for release in range(1, 15):
    previous = f'2026.2.{release}'
    retained_prefix = f'maven/com/mecorobotics/frc/meco-base-cpp/{previous}/meco-base-cpp-{previous}'
    retained = {retained_prefix + '.pom', retained_prefix + '-headers.zip', retained_prefix + '-sources.zip'}
    retained.update(f'{retained_prefix}-{platform}{variant}.zip' for platform in dependency['binaryPlatforms'] for variant in ('static', 'staticdebug'))
    if not retained <= manifest.keys():
        raise ValueError(f'Retained Base {previous} Maven files are missing from checksum manifest')
items = [(p, h) for p, h in manifest.items() if p.startswith('maven/')]
def verify(item):
    path, digest = item
    if hashlib.sha256(fetch(SITE + '/' + path)).hexdigest() != digest:
        raise ValueError(f'Checksum mismatch: {path}')
with ThreadPoolExecutor(max_workers=12) as pool:
    list(pool.map(verify, items))
print(f'Verified Base {version}, descriptor, and {len(items)} Maven artifact checksums')
