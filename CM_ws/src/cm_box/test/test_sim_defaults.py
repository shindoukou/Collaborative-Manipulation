#!/usr/bin/env python3
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import cm_short_entry as entry


class PackagePaths:
    def __init__(self, root):
        self.root = root

    def get_path(self, name):
        return str(self.root / name)


class SimulationDefaultsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.packages = PackagePaths(self.root)
        self.profile = self.root / 'cm_box/config/sim.json'
        self.profile.parent.mkdir(parents=True)
        self.profile.write_text(json.dumps({'simulation_reference': {
            'package': 'cm_kirara_box', 'directory': 'generated/verified'}}))
        self.references = self.root / 'cm_kirara_box/generated/verified'
        for phase in ('catch', 'move'):
            (self.references / phase).mkdir(parents=True)
            for side in ('left', 'right'):
                (self.references / phase / '{}_{}_smoothed_path.txt'.format(phase, side)).write_text('0\n')

    def test_both_sides_use_portable_package_paths(self):
        for side in ('left', 'right'):
            argv = ['sim_{}.py'.format(side)]
            entry.configure_sim_defaults(side, argv, self.packages)
            self.assertEqual(argv[argv.index('--config') + 1], str(self.profile))
            for phase in ('catch', 'move'):
                self.assertEqual(argv[argv.index('--{}-reference'.format(phase)) + 1],
                    str(self.references / phase / '{}_{}_smoothed_path.txt'.format(phase, side)))

    def test_explicit_config_is_unchanged(self):
        for options in (['--config', 'custom.json'], ['--config=custom.json']):
            argv = ['sim_left.py'] + options
            before = list(argv)
            entry.configure_sim_defaults('left', argv, self.packages)
            self.assertEqual(argv, before)

    def test_explicit_reference_is_kept(self):
        argv = ['sim_left.py', '--move-reference=custom.txt']
        entry.configure_sim_defaults('left', argv, self.packages)
        self.assertIn('--move-reference=custom.txt', argv)
        self.assertNotIn('--move-reference', argv)
        self.assertIn('--catch-reference', argv)

    def test_missing_reference_fails_without_partial_argv_changes(self):
        (self.references / 'move/move_left_smoothed_path.txt').unlink()
        argv = ['sim_left.py']
        with self.assertRaises(IOError):
            entry.configure_sim_defaults('left', argv, self.packages)
        self.assertEqual(argv, ['sim_left.py'])

    def test_real_entry_does_not_load_sim_defaults(self):
        from types import SimpleNamespace
        calls = []
        with patch.object(entry, 'configure_sim_defaults') as defaults, \
             patch.object(entry.rospkg, 'RosPack', return_value=self.packages), \
             patch.dict(sys.modules, {'cm_dnmpc_arm_node': SimpleNamespace(main=lambda **kw: calls.append(kw))}):
            entry.run_dnmpc('box', 'left', 'real', object_profile=True)
        defaults.assert_not_called()
        self.assertTrue(calls[0]['auto_real_profile'])
        self.assertEqual(calls[0]['forced_runtime'], 'real')


if __name__ == '__main__':
    unittest.main()
