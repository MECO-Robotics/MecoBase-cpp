import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from release_guard import PLATFORMS, provenance, validate_tag, verify


class ReleaseGuardTest(unittest.TestCase):
    def test_release_tag_must_match_descriptor(self):
        with tempfile.TemporaryDirectory() as temporary:
            descriptor = Path(temporary) / 'descriptor.json'
            descriptor.write_text(json.dumps({'version': '2026.2.4'}))
            validate_tag('refs/tags/v2026.2.4', descriptor)
            with self.assertRaises(ValueError):
                validate_tag('refs/tags/v2026.2.3', descriptor)

    def test_rejects_stale_commit_or_modified_archive(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name in PLATFORMS['linux']:
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b'archive')
            record = root / 'provenance.json'
            record.write_text(json.dumps(provenance('linux', root, 'old')))
            verify('linux', root, 'old', record)
            with self.assertRaises(ValueError):
                verify('linux', root, 'new', record)
            (root / PLATFORMS['linux'][0]).write_bytes(b'modified')
            with self.assertRaises(ValueError):
                verify('linux', root, 'old', record)


if __name__ == '__main__':
    unittest.main()
