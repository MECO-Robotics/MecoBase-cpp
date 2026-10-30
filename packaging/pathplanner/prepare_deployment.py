#!/usr/bin/env python3
"""Build a fresh Pages deployment with exact previously tested candidate bytes."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import urllib.request

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root / 'scripts'))
from preserve_published_maven import preserve

PREFIX = 'maven/com/pathplanner/lib/PathplannerLib-cpp/2026.1.2-meco1/'


def prepare(candidate, destination, url):
    candidate = candidate.resolve()
    destination = destination.resolve()
    if destination.exists() and any(destination.iterdir()):
        raise ValueError('Deployment destination must be empty')
    destination.mkdir(parents=True, exist_ok=True)
    # These are the root files produced by the existing MecoBase publisher.
    for name in ('MecoBase.json', 'SHA256SUMS.json', 'build-provenance.json', 'LICENSE.md'):
        with urllib.request.urlopen(f'{url.rstrip("/")}/{name}', timeout=30) as response:
            (destination / name).write_bytes(response.read())
    preserve(url, destination)
    fresh_file = destination / 'SHA256SUMS.json'
    fresh = json.loads(fresh_file.read_text())
    staged = json.loads((candidate / 'SHA256SUMS.json').read_text())
    paths = {name: digest for name, digest in staged['artifacts'].items() if name.startswith(PREFIX)}
    if len(paths) != 9:
        raise ValueError(f'Expected nine candidate artifacts, found {len(paths)}')
    for name, expected in paths.items():
        source = candidate / name
        if not source.resolve().is_relative_to(candidate):
            raise ValueError(f'Candidate path escapes site: {name}')
        data = source.read_bytes()
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError(f'Candidate checksum mismatch: {name}')
        old = fresh['artifacts'].get(name)
        if old is not None and old != expected:
            raise ValueError(f'Published immutable coordinate conflict: {name}')
        target = destination / name
        if target.exists() and target.read_bytes() != data:
            raise ValueError(f'Existing artifact conflicts: {name}')
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        fresh['artifacts'][name] = expected
    # Put provenance in the retained Maven namespace for later releases.
    provenance = candidate / 'pathplanner-provenance.json'
    content = provenance.read_bytes()
    provenance_path = PREFIX + 'PathplannerLib-cpp-2026.1.2-meco1-provenance.json'
    expected = hashlib.sha256(content).hexdigest()
    if provenance_path in fresh['artifacts'] and fresh['artifacts'][provenance_path] != expected:
        raise ValueError('Published provenance conflicts with candidate')
    output = destination / provenance_path
    if output.exists() and output.read_bytes() != content:
        raise ValueError('Existing provenance conflicts with candidate')
    output.write_bytes(content)
    fresh['artifacts'][provenance_path] = expected
    fresh_file.write_text(json.dumps(fresh, indent=2, sort_keys=True) + '\n')
    # Keep the expected snapshot outside Pages for post-deploy comparisons.
    return fresh


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    parser.add_argument('--site-url', required=True)
    args = parser.parse_args()
    result = prepare(args.candidate, args.destination, args.site_url)
    print(f"Prepared {len(result['artifacts'])} checksummed files")
