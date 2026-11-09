#!/usr/bin/env python3
"""Copy this package's Maven artifacts into a checked-out raw Maven branch."""

import argparse
from pathlib import Path
import shutil


def copy_package(package: Path, destination: Path) -> int:
    source_maven = package.resolve() / 'maven'
    destination = destination.resolve()
    coordinate_root = Path('com/mecorobotics/frc/meco-base-cpp')
    sources = sorted((source_maven / coordinate_root).rglob('*'))
    files = [path for path in sources if path.is_file()]
    if not files:
        raise FileNotFoundError(f'No MecoBase Maven artifacts under {source_maven / coordinate_root}')
    for source in files:
        relative = source.relative_to(source_maven)
        if source.stat().st_size >= 100_000_000:
            raise ValueError(f'Raw GitHub cannot serve this Maven artifact reliably: {relative}')
        target = destination / relative
        if target.exists():
            if target.read_bytes() != source.read_bytes():
                raise ValueError(f'Published Maven artifact is immutable: {relative}')
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    return len(files)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    args = parser.parse_args()
    print(f'Copied or verified {copy_package(args.package, args.destination)} MecoBase Maven artifacts')


if __name__ == '__main__':
    main()
