#!/usr/bin/env python3
"""Stage an immutable MECO PathPlanner coordinate from upstream build outputs."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

VERSION = '2026.1.2-meco1'
ARTIFACT = 'PathplannerLib-cpp'
PREFIX = '_GROUP_com_pathplanner_lib_ID_PathplannerLib-cpp_CLS-'
PLATFORMS = {
    'linuxathenastatic': 'linux/athena/static/libPathplannerLib.a',
    'linuxathenastaticdebug': 'linux/athena/static/libPathplannerLib.a',
    'linuxx86-64static': 'linux/x86-64/static/libPathplannerLib.a',
    'linuxx86-64staticdebug': 'linux/x86-64/static/libPathplannerLib.a',
    'windowsx86-64static': 'windows/x86-64/static/PathplannerLib.lib',
    'windowsx86-64staticdebug': 'windows/x86-64/static/PathplannerLib.lib',
}


def stage(outputs: Path, site: Path):
    target = site / 'maven/com/pathplanner/lib' / ARTIFACT / VERSION
    candidates = {'headers': (outputs / (PREFIX + 'headers.zip'), 'pathplanner/lib/path/PathPlannerPath.h')}
    candidates.update({key: (outputs / ('_M_' + PREFIX + key + '.zip'), value) for key, value in PLATFORMS.items()})
    files = {}
    for classifier, (source, expected) in candidates.items():
        with zipfile.ZipFile(source) as archive:
            if expected not in archive.namelist() or archive.getinfo(expected).file_size == 0:
                raise ValueError(f'{source}: missing or empty {expected}')
        files[f'{ARTIFACT}-{VERSION}-{classifier}.zip'] = source.read_bytes()
    files[f'{ARTIFACT}-{VERSION}.pom'] = (f'<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<project xmlns="http://maven.apache.org/POM/4.0.0"><modelVersion>4.0.0</modelVersion>'
        f'<groupId>com.pathplanner.lib</groupId><artifactId>{ARTIFACT}</artifactId>'
        f'<version>{VERSION}</version><packaging>pom</packaging></project>\n').encode()
    for name, data in files.items():
        path = target / name
        if path.exists() and path.read_bytes() != data:
            raise ValueError(f'immutable coordinate conflict: {path}')
    target.mkdir(parents=True, exist_ok=True)
    for name, data in files.items():
        (target / name).write_bytes(data)
    return {str((target / name).relative_to(site)): hashlib.sha256(data).hexdigest() for name, data in files.items()}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--outputs', type=Path, required=True)
    parser.add_argument('--site', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(stage(args.outputs, args.site), indent=2, sort_keys=True))
