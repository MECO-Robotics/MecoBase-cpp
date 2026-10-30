#!/usr/bin/env python3
"""Pin PathPlanner's dynamic WPILib build inputs for the 2026 MECO package."""
import pathlib
import subprocess
import sys

root = pathlib.Path(sys.argv[1]).resolve()
expected = 'e02bbf3176588166e8fe5192ab8dea85f6d62f7a'
commit = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
if commit != expected:
    raise SystemExit(f'Unexpected PathPlanner revision {commit}, expected {expected}')
patch = pathlib.Path(__file__).with_name('sample-index.patch.gz')
import gzip
subprocess.run(['git', '-C', str(root), 'apply', '--check', '-'], input=gzip.decompress(patch.read_bytes()), check=True)
subprocess.run(['git', '-C', str(root), 'apply', '-'], input=gzip.decompress(patch.read_bytes()), check=True)
for name, old, new in [
    ('pathplannerlib/build.gradle', 'ext.wpilibVersion = "2026.+"', 'ext.wpilibVersion = "2026.2.1"'),
    ('pathplannerlib/config.gradle', 'wpiVersion = "2026.1.1"', 'wpiVersion = "2026.2.1"'),
    ('pathplannerlib/vendordeps/WPILibNewCommands.json', '"2026.+"', '"2026.2.1"'),
]:
    path = root / name
    contents = path.read_text(encoding='utf-8')
    if contents.count(old) != (2 if name.endswith('.json') else 1):
        raise SystemExit(f'Unexpected pin source in {name}')
    path.write_text(contents.replace(old, new), encoding='utf-8')
