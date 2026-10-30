#!/usr/bin/env python3
"""Snapshot published MECO Maven coordinates and add the candidate patch."""
import hashlib
import json
from pathlib import Path
import sys
import urllib.request

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root / 'scripts'))
from preserve_published_maven import preserve
from stage import stage

outputs, site = map(Path, sys.argv[1:])
url = 'https://meco-robotics.github.io/MecoBase-cpp'
site.mkdir(parents=True, exist_ok=True)
for name in ('MecoBase.json', 'SHA256SUMS.json', 'build-provenance.json', 'LICENSE.md'):
    with urllib.request.urlopen(f'{url}/{name}', timeout=30) as response:
        (site / name).write_bytes(response.read())
preserve(url, site)
new = stage(outputs, site)
manifest_file = site / 'SHA256SUMS.json'
manifest = json.loads(manifest_file.read_text())
for path, digest in new.items():
    if path in manifest['artifacts'] and manifest['artifacts'][path] != digest:
        raise ValueError(f'Existing published coordinate conflicts: {path}')
    manifest['artifacts'][path] = digest
manifest_file.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
provenance = {'sourceCommit': 'e02bbf3176588166e8fe5192ab8dea85f6d62f7a',
              'patchSha256': hashlib.sha256((root / 'packaging/pathplanner/sample-index.patch.gz').read_bytes()).hexdigest(),
              'wpilibVersion': '2026.2.1', 'artifacts': new}
(site / 'pathplanner-provenance.json').write_text(json.dumps(provenance, indent=2, sort_keys=True) + '\n')
descriptor = json.loads((site / 'MecoBase.json').read_text())
next(dep for dep in descriptor['cppDependencies'] if dep['artifactId'] == 'PathplannerLib-cpp')['version'] = '2026.1.2-meco1'
(site / 'PathplannerCandidate.json').write_text(json.dumps(descriptor, indent=2) + '\n')
