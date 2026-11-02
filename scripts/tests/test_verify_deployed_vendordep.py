import hashlib
import importlib.util
import json
from pathlib import Path
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('verify_deployed', Path(__file__).parents[1] / 'verify_deployed_vendordep.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class VerifyDeployedVendordepTest(unittest.TestCase):
    def test_verifies_release_checkout_and_requires_each_published_archive(self):
        site = 'https://example.test'
        source = 'https://example.test/tag/MecoBase.json'
        version = '2026.2.12'
        descriptor = {'version': version, 'jsonUrl': source, 'mavenUrls': [site + '/maven', 'https://other.test'],
                      'cppDependencies': [{'groupId': 'com.mecorobotics.frc', 'artifactId': 'meco-base-cpp',
                                           'version': version, 'binaryPlatforms': ['linuxathena', 'linuxx86-64', 'windowsx86-64']}]}
        prefix = f'maven/com/mecorobotics/frc/meco-base-cpp/{version}/meco-base-cpp-{version}'
        suffixes = ['.pom', '-headers.zip', '-sources.zip'] + [
            f'-{platform}{variant}.zip' for platform in descriptor['cppDependencies'][0]['binaryPlatforms']
            for variant in ('static', 'staticdebug')]
        content = b'published archive'
        checksums = {prefix + suffix: hashlib.sha256(content).hexdigest() for suffix in suffixes}
        def fetch(url, attempts, delay):
            if url in (site + '/MecoBase.json', source):
                return json.dumps(descriptor).encode()
            if url == site + '/SHA256SUMS.json':
                return json.dumps({'artifacts': checksums}).encode()
            self.assertTrue(url.startswith(site + '/maven/'))
            return content
        with patch.object(module, 'fetch', side_effect=fetch):
            self.assertEqual(module.verify(site, descriptor), (version, len(suffixes)))
            del checksums[prefix + '-windowsx86-64static.zip']
            with self.assertRaisesRegex(ValueError, 'lacks required Base artifacts'):
                module.verify(site, descriptor)


if __name__ == '__main__':
    unittest.main()
