#!/usr/bin/env python3
"""Normalize vendordep archives and record checksums/build provenance."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

root = Path(sys.argv[1]).resolve()
repo = Path(__file__).resolve().parents[1]
epoch = (1980, 1, 1, 0, 0, 0)
for path in sorted(root.rglob('*.zip')):
    with zipfile.ZipFile(path) as source:
        entries = [(item.filename, source.read(item.filename), item.external_attr) for item in source.infolist() if not item.is_dir()]
    temporary = path.with_suffix('.normalized.zip')
    with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=9) as target:
        for name, data, attributes in sorted(entries):
            info = zipfile.ZipInfo(name, epoch)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = attributes or (0o100644 << 16)
            target.writestr(info, data)
    temporary.replace(path)

commit = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=repo, check=True, text=True, capture_output=True).stdout.strip()
manifest = {"sourceCommit": commit, "descriptorVersion": json.loads((root / 'MecoBase.json').read_text())['version'], "artifacts": {}}
for path in sorted(item for item in root.rglob('*') if item.is_file() and item.name not in {'SHA256SUMS.json', 'build-provenance.json'}):
    manifest['artifacts'][path.relative_to(root).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
(root / 'SHA256SUMS.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
(root / 'build-provenance.json').write_text(json.dumps({"sourceCommit": commit, "descriptorVersion": manifest['descriptorVersion']}, indent=2, sort_keys=True) + '\n')
