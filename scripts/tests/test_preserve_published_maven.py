import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('preserve', Path(__file__).parents[1] / 'preserve_published_maven.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PreservePublishedMavenTest(unittest.TestCase):
    def test_retains_old_coordinates_without_replacing_current_metadata(self):
        old = 'maven/com/meco/2026.2.1/base.zip'
        data = b'published binary'
        digest = hashlib.sha256(data).hexdigest()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest = {'sourceCommit': 'new', 'descriptorVersion': '2026.2.2', 'artifacts': {'MecoBase.json': 'current'}}
            (root / 'SHA256SUMS.json').write_text(json.dumps(manifest))
            (root / 'MecoBase.json').write_bytes(b'current descriptor')
            def fetch(url):
                if url.endswith('/SHA256SUMS.json'):
                    return json.dumps({'artifacts': {old: digest, 'MecoBase.json': 'old'}}).encode()
                self.assertEqual(url, 'https://example.test/' + old)
                return data
            with patch.object(module, 'fetch', side_effect=fetch):
                self.assertEqual(module.preserve('https://example.test/', root), 1)
                self.assertEqual(module.preserve('https://example.test/', root), 0)
            self.assertEqual((root / old).read_bytes(), data)
            self.assertEqual((root / 'MecoBase.json').read_bytes(), b'current descriptor')
            manifest['artifacts'][old] = digest
            self.assertEqual(json.loads((root / 'SHA256SUMS.json').read_text()), manifest)

    def test_rejects_corruption_conflicts_and_unsafe_paths(self):
        valid = 'maven/com/meco/old.zip'
        for relative, existing in [(valid, None), (valid, b'new conflicting binary'),
                                   ('maven/../outside', None), ('maven/%2e%2e/outside', None),
                                   ('maven//outside', None), ('MecoBase.json', None)]:
            with self.subTest(relative=relative, existing=existing), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                original = json.dumps({'sourceCommit': 'new', 'artifacts': {}})
                (root / 'SHA256SUMS.json').write_text(original)
                if existing is not None:
                    destination = root / relative
                    destination.parent.mkdir(parents=True)
                    destination.write_bytes(existing)
                published = json.dumps({'artifacts': {relative: hashlib.sha256(b'old binary').hexdigest()}}).encode()
                with patch.object(module, 'fetch', side_effect=[published, b'corrupt download']):
                    with self.assertRaises(ValueError):
                        module.preserve('https://example.test', root)
                self.assertEqual((root / 'SHA256SUMS.json').read_text(), original)
                if existing is not None:
                    self.assertEqual((root / relative).read_bytes(), existing)


if __name__ == '__main__':
    unittest.main()
