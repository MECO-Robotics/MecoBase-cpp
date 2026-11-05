#!/usr/bin/env python3
"""Fetch/verify exact official Studica Windows artifacts for a local Maven cache.

Linux CI can reach the vendor server when Windows runners cannot. The transferred
cache contains unmodified official artifacts checked against a committed manifest.
"""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import urllib.request
import xml.etree.ElementTree as ET
import zipfile


def verify(path, artifact):
    if hashlib.sha256(path.read_bytes()).hexdigest() != artifact['sha256']:
        raise ValueError(f'SHA256 mismatch: {artifact["path"]}')
    if path.suffix == '.zip':
        with zipfile.ZipFile(path) as archive:
            if not archive.namelist() or archive.testzip() is not None:
                raise ValueError(f'Invalid ZIP archive: {artifact["path"]}')
    else:
        project = ET.parse(path).getroot()
        namespace = {'m': 'http://maven.apache.org/POM/4.0.0'}
        expected = ('com.studica.frc', path.parent.parent.name, '2026.0.0')
        actual = tuple(project.findtext('m:' + key, namespaces=namespace)
                       for key in ('groupId', 'artifactId', 'version'))
        if actual != expected:
            raise ValueError(f'Unexpected Maven POM coordinates: {actual}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--verify-only', action='store_true')
    parser.add_argument('--manifest', type=Path, default=Path(__file__).with_name('vendor-cache-sha256.json'))
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    for artifact in manifest['artifacts']:
        relative = Path(artifact['path'])
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('Manifest paths must remain inside the Maven cache')
        destination = args.output / relative
        if not args.verify_only:
            destination.parent.mkdir(parents=True, exist_ok=True)
            with urllib.request.urlopen(artifact['url'], timeout=60) as response:
                data = response.read()
            with tempfile.NamedTemporaryFile(dir=destination.parent, suffix=destination.suffix, delete=False) as temporary:
                temporary.write(data)
                temporary_path = Path(temporary.name)
            try:
                verify(temporary_path, artifact)
                temporary_path.replace(destination)
            finally:
                temporary_path.unlink(missing_ok=True)
        verify(destination, artifact)
        print(f'Verified {artifact["path"]}')


if __name__ == '__main__':
    main()
