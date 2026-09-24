#!/usr/bin/env python3
"""Run after building CM_SOCKET_BOX_VALIDATOR; no ROS or Gazebo is started."""
import copy
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import pinocchio as pin
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
from cm_box_attachment import BoxAttachment


class AttachmentTests(unittest.TestCase):
    def setUp(self):
        self.path = PACKAGE / 'config/move_kirara_box.yaml'
        self.config = yaml.safe_load(self.path.read_text())
        self.model = pin.buildModelFromUrdf(self.config['file_path']['pinocchio_file'])
        self.exe = WORKSPACE / 'devel/lib/cm_kirara_box/CM_SOCKET_BOX_VALIDATOR'

    def test_native_python_agreement(self):
        attachment = BoxAttachment(self.config, self.model)
        data = self.model.createData()
        initial = np.array([self.config['Joint_init']['joint_' + str(i + 1)] for i in range(12)])
        rng = np.random.RandomState(20260905)
        pairs = np.vstack([initial, initial + rng.uniform(-.2, .2, (32, 12))])
        request = ''.join('P {} {}\n'.format(i, ' '.join('%.17g' % v for v in q)) for i, q in enumerate(pairs)) + 'X\n'
        result = subprocess.run([str(self.exe), 'move-fcl', str(self.path)], input=request,
                                capture_output=True, text=True, timeout=15, check=True)
        lines = result.stdout.splitlines()
        self.assertTrue(lines[0].startswith('READY'))
        self.assertEqual(len(lines), len(pairs) + 1)
        for pair, line in zip(pairs, lines[1:]):
            values = np.array([float(v) for v in line.split()[2:]])
            q = pin.neutral(self.model)
            q[:12] = pair
            pose = attachment.apply(self.model, data, q)
            np.testing.assert_allclose(pose.translation, values[:3], atol=1e-12)
            np.testing.assert_allclose(pose.rotation, pin.Quaternion(values[3:]).matrix(), atol=1e-12)

    def test_bad_mapping_is_rejected(self):
        for field, value in [('quaternion_xyzw', [0, 0, 0, 0]),
                             ('translation_m', [0, 0]),
                             ('anchor_frame', 'missing_frame')]:
            config = copy.deepcopy(self.config)
            config['carried_box'][field] = value
            with self.assertRaises((ValueError, IndexError)):
                BoxAttachment(config, self.model)
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'invalid.yaml'
                path.write_text(yaml.safe_dump(config))
                result = subprocess.run([str(self.exe), 'move-fcl', str(path)], input='X\n',
                                        capture_output=True, text=True, timeout=15)
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn('READY', result.stdout)

    def test_missing_mapping_is_rejected(self):
        config = copy.deepcopy(self.config)
        del config['carried_box']
        with self.assertRaises(ValueError):
            BoxAttachment(config, self.model)

    def test_catch_does_not_require_attachment(self):
        result = subprocess.run([str(self.exe), 'catch', str(PACKAGE / 'config/catch_kirara_box.yaml')],
                                input='X\n', capture_output=True, text=True, timeout=15, check=True)
        self.assertTrue(result.stdout.startswith('READY catch'))


if __name__ == '__main__':
    unittest.main()
