#!/usr/bin/env python3
"""Build and assemble local Linux desktop and roboRIO release archives."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
os.chdir(ROOT)


def run(*args):
    print('+', *map(str, args), flush=True)
    subprocess.run([str(arg) for arg in args], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('build', 'assemble'))
    args = parser.parse_args()
    if subprocess.check_output(['git', 'status', '--porcelain', '--untracked-files=normal'], text=True).strip():
        raise SystemExit('Release builds require a clean committed checkout')
    if not sys.platform.startswith('linux'):
        raise SystemExit('The release tool requires a Linux host with the WPILib roboRIO toolchain')
    gradle = ROOT / 'gradlew'
    if args.operation == 'build':
        tasks = ['frcBaseLinuxathenaDebugStaticLibrary', 'frcBaseLinuxathenaReleaseStaticLibrary',
                 'frcBaseLinuxx86-64DebugStaticLibrary', 'frcBaseLinuxx86-64ReleaseStaticLibrary']
        run(gradle, *tasks, '-Ptoolchain-optional-roboRio', '--max-workers=2', '--console=plain')
        run(sys.executable, 'scripts/release_guard.py', 'record', '--platform', 'linux',
            '--file', 'build/provenance/linux.json')
        print('Linux and roboRIO archives and source provenance are in build/')
    else:
        for name in ('linux',):
            if not (ROOT / f'build/provenance/{name}.json').is_file():
                raise SystemExit(f'Missing {name} provenance; build that platform and copy its archives and provenance first')
            run(sys.executable, 'scripts/release_guard.py', 'verify', '--platform', name,
                '--file', f'build/provenance/{name}.json')
        run(gradle, 'packageVendorMaven', '-Ptoolchain-optional-roboRio', '--max-workers=2', '--console=plain')
        records = {name: json.loads((ROOT / f'build/provenance/{name}.json').read_text(encoding='utf-8'))
                   for name in ('linux',)}
        package = ROOT / 'build/vendor-maven'
        (package / 'local-build-provenance.json').write_text(
            json.dumps({'buildOrigin': 'local', 'platforms': records}, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
        run(sys.executable, 'scripts/finalize_vendordep.py', package)
        print('Complete local package: build/vendor-maven')


if __name__ == '__main__':
    main()
