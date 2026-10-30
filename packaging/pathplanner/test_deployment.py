import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('deployment', Path(__file__).with_name('prepare_deployment.py'))
deployment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(deployment)


class Response(io.BytesIO):
    pass


class DeploymentTest(unittest.TestCase):
    def test_refresh_preserves_newer_coordinates_and_rejects_conflicts(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as temp:
            root = Path(temp)
            candidate = root / 'candidate'
            candidate.mkdir()
            prefix = deployment.PREFIX
            artifacts = {}
            for index in range(9):
                name = f'{prefix}candidate-{index}.zip'
                data = f'candidate {index}'.encode()
                path = candidate / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(data)
                artifacts[name] = hashlib.sha256(data).hexdigest()
            (candidate / 'SHA256SUMS.json').write_text(json.dumps({'artifacts': artifacts}))
            (candidate / 'pathplanner-provenance.json').write_text('{}')
            latest = 'maven/com/example/later/1.0/later.pom'
            latest_data = b'published after candidate staging'
            manifest = {'artifacts': {latest: hashlib.sha256(latest_data).hexdigest()}}
            site = {'MecoBase.json': b'{}', 'SHA256SUMS.json': json.dumps(manifest).encode(),
                    'build-provenance.json': b'{}', 'LICENSE.md': b'license', latest: latest_data}
            def urlopen(url, timeout=30):
                return Response(site[url.removeprefix('https://example.test/')])
            with patch('urllib.request.urlopen', side_effect=urlopen):
                result = deployment.prepare(candidate, root / 'deployment', 'https://example.test')
            self.assertEqual((root / 'deployment' / latest).read_bytes(), latest_data)
            self.assertEqual(result['artifacts'][latest], manifest['artifacts'][latest])
            self.assertEqual(len(result['artifacts']), 11)
            first = next(iter(artifacts))
            conflict_manifest = {'artifacts': {first: hashlib.sha256(b'other').hexdigest()}}
            site['SHA256SUMS.json'] = json.dumps(conflict_manifest).encode()
            site[first] = b'other'
            with patch('urllib.request.urlopen', side_effect=urlopen):
                with self.assertRaisesRegex(ValueError, 'immutable coordinate conflict'):
                    deployment.prepare(candidate, root / 'conflict', 'https://example.test')


if __name__ == '__main__':
    unittest.main()
