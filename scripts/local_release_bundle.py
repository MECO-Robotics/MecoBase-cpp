#!/usr/bin/env python3
"""Create or unpack a locally assembled release package with checked contents."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTOR = 'MecoBase.json'
SOURCE = ROOT / 'vendor/MecoBase.json'
ARTIFACT = 'meco-base-cpp'
GROUP = 'com/mecorobotics/frc'
PLATFORMS = ('linuxathena', 'linuxx86-64')


def validate(directory):
    manifest = json.loads((directory / 'SHA256SUMS.json').read_text(encoding='utf-8'))
    descriptor = json.loads((directory / DESCRIPTOR).read_text(encoding='utf-8'))
    if descriptor != json.loads(SOURCE.read_text(encoding='utf-8')):
        raise ValueError('Package descriptor does not match the source checkout')
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    if manifest.get('sourceCommit') != commit:
        raise ValueError('Package was assembled from a different source commit')
    version = descriptor['version']
    dependency = next((item for item in descriptor.get('cppDependencies', []) if item.get('artifactId') == ARTIFACT), {})
    if set(dependency.get('binaryPlatforms', [])) != set(PLATFORMS):
        raise ValueError('Descriptor platforms do not match the local package')
    prefix = f'maven/{GROUP}/{ARTIFACT}/{version}/{ARTIFACT}-{version}'
    required = {prefix + '.pom', prefix + '-headers.zip', prefix + '-sources.zip'}
    required.update(f'{prefix}-{platform}{suffix}.zip' for platform in PLATFORMS
                    for suffix in ('static', 'staticdebug'))
    artifacts = manifest.get('artifacts', {})
    if (not isinstance(artifacts, dict) or not required.issubset(artifacts)
            or 'local-build-provenance.json' not in artifacts):
        raise ValueError('Package manifest does not cover every required platform artifact')
    for name, digest in artifacts.items():
        path = PurePosixPath(name)
        if path.is_absolute() or not path.parts or any(p in ('.', '..') for p in path.parts):
            raise ValueError(f'Unsafe package path: {name!r}')
        file = directory.joinpath(*path.parts)
        if not file.is_file() or hashlib.sha256(file.read_bytes()).hexdigest() != digest:
            raise ValueError(f'Missing or corrupt package file: {name}')
    provenance = json.loads((directory / 'local-build-provenance.json').read_text(encoding='utf-8'))
    if provenance.get('buildOrigin') != 'local' or set(provenance.get('platforms', {})) != {'linux'}:
        raise ValueError('Package does not contain the local Linux build record')
    for platform in PLATFORMS:
        host = 'windows' if platform.startswith('windows') else 'linux'
        record = provenance['platforms'][host]
        if record.get('sourceCommit') != commit or record.get('platform') != host:
            raise ValueError('Platform build source differs from the package source')
        for configuration, suffix in (('release', 'static'), ('debug', 'staticdebug')):
            source_name = 'frcBase.lib' if platform.startswith('windows') else 'libfrcBase.a'
            published_name = 'mecoBase.lib' if platform.startswith('windows') else 'libmecoBase.a'
            key = f'{platform}/{configuration}/{source_name}'
            with zipfile.ZipFile(directory / f'{prefix}-{platform}{suffix}.zip') as archive:
                members = [name for name in archive.namelist() if name.endswith('/' + published_name)]
                if len(members) != 1 or hashlib.sha256(archive.read(members[0])).hexdigest() != record.get('archives', {}).get(key):
                    raise ValueError(f'Packaged library does not match local build record: {key}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('pack', 'unpack'))
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--bundle', type=Path, required=True)
    args = parser.parse_args()
    if args.operation == 'pack':
        if args.bundle.resolve().is_relative_to(args.package.resolve()):
            raise ValueError('Bundle must be outside the package directory')
        validate(args.package)
        args.bundle.parent.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(args.bundle, 'w', compression=zipfile.ZIP_DEFLATED) as output:
            for file in sorted(args.package.rglob('*')):
                if file.is_file():
                    output.write(file, file.relative_to(args.package).as_posix())
    else:
        if args.package.exists() and any(args.package.iterdir()):
            raise ValueError('Unpack destination must be empty')
        with zipfile.ZipFile(args.bundle) as archive:
            names = archive.namelist()
            if len(names) != len(set(names)):
                raise ValueError('Duplicate paths in package bundle')
            for name in names:
                path = PurePosixPath(name)
                if path.is_absolute() or not path.parts or any(p in ('.', '..') for p in path.parts) or '\\' in name:
                    raise ValueError(f'Unsafe bundle path: {name!r}')
            archive.extractall(args.package)
        validate(args.package)


if __name__ == '__main__':
    main()
