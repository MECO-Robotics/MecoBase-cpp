#!/usr/bin/env python3
"""Retain checksummed published Maven artifacts when replacing the Pages site."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import urllib.request


def fetch(url):
    with urllib.request.urlopen(url, timeout=30) as response:
        return response.read()


def preserve(site_url, repository):
    repository = Path(repository).resolve()
    manifest_path = repository / 'SHA256SUMS.json'
    current = json.loads(manifest_path.read_text())
    published = json.loads(fetch(site_url.rstrip('/') + '/SHA256SUMS.json'))
    artifacts = published['artifacts']
    if not isinstance(artifacts, dict) or not any(path.startswith('maven/') for path in artifacts):
        raise ValueError('Published checksum manifest has no Maven artifacts')
    retained = 0
    for relative, expected in artifacts.items():
        if not relative.startswith('maven/'):
            continue
        if (not re.fullmatch(r'maven/[A-Za-z0-9_./+\-]+', relative)
                or any(part in ('', '.', '..') for part in relative.split('/'))
                or not isinstance(expected, str)
                or not re.fullmatch(r'[0-9a-f]{64}', expected)):
            raise ValueError(f'Invalid published Maven entry: {relative!r}')
        destination = repository / relative
        if not destination.resolve().is_relative_to(repository):
            raise ValueError(f'Maven entry escapes repository: {relative!r}')
        data = destination.read_bytes() if destination.exists() else fetch(site_url.rstrip('/') + '/' + relative)
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError(f'Checksum mismatch or immutable artifact conflict: {relative}')
        if not destination.exists():
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(data)
            retained += 1
        current['artifacts'][relative] = expected
    manifest_path.write_text(json.dumps(current, indent=2, sort_keys=True) + '\n')
    return retained


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--site-url', required=True)
    parser.add_argument('--repository', required=True)
    args = parser.parse_args()
    print(f'Retained {preserve(args.site_url, args.repository)} published Maven artifacts')
