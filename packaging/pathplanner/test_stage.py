import importlib.util
from pathlib import Path
import tempfile
import unittest
import zipfile

spec = importlib.util.spec_from_file_location('stage', Path(__file__).with_name('stage.py'))
staging = importlib.util.module_from_spec(spec)
spec.loader.exec_module(staging)


class StageTest(unittest.TestCase):
    def test_missing_archive_leaves_coordinate_absent_and_conflicts_are_rejected(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            root = Path(directory)
            outputs, site = root / 'outputs', root / 'site'
            outputs.mkdir()
            (outputs / 'PathPlanner-LICENSE.txt').write_text('test license')
            candidates = {'headers': 'pathplanner/lib/path/PathPlannerPath.h', **staging.PLATFORMS}
            for classifier, member in candidates.items():
                prefix = '' if classifier == 'headers' else '_M_'
                with zipfile.ZipFile(outputs / (prefix + staging.PREFIX + classifier + '.zip'), 'w') as archive:
                    archive.writestr(member, b'test')
            missing = outputs / ('_M_' + staging.PREFIX + 'windowsx86-64static.zip')
            data = missing.read_bytes()
            missing.unlink()
            with self.assertRaises(FileNotFoundError):
                staging.stage(outputs, site)
            self.assertFalse(site.exists())
            missing.write_bytes(data)
            manifest = staging.stage(outputs, site)
            self.assertEqual(len(manifest), 9)
            path = site / next(iter(manifest))
            path.write_bytes(b'conflict')
            with self.assertRaisesRegex(ValueError, 'immutable coordinate conflict'):
                staging.stage(outputs, site)
            self.assertEqual(path.read_bytes(), b'conflict')


if __name__ == '__main__':
    unittest.main()
