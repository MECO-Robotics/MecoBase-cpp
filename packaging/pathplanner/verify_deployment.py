#!/usr/bin/env python3
"""Verify deployed files against the exact serialized deployment snapshot."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
from verify_deployed_vendordep import fetch

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--expected', type=Path, required=True)
parser.add_argument('--site-url', required=True)
args = parser.parse_args()
site = args.site_url.rstrip('/')
expected = json.loads((args.expected / 'SHA256SUMS.json').read_text())
for attempt in range(12):
    actual = json.loads(fetch(f'{site}/SHA256SUMS.json', 12, 10))
    if actual == expected:
        break
    if attempt < 11:
        time.sleep(10)
else:
    raise SystemExit('Deployed checksum manifest differs from the serialized snapshot')
if fetch(f'{site}/MecoBase.json', 12, 10) != (args.expected / 'MecoBase.json').read_bytes():
    raise SystemExit('Deployed Base descriptor differs from the serialized snapshot')
for path, digest in expected['artifacts'].items():
    if path.startswith('maven/'):
        if hashlib.sha256(fetch(f'{site}/{path}', 12, 10)).hexdigest() != digest:
            raise SystemExit(f'Deployed artifact checksum mismatch: {path}')
print(f"Verified {sum(p.startswith('maven/') for p in expected['artifacts'])} published Maven files")
