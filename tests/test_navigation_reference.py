import ast
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'navigation'))
from navigation_reference import select_reference, reference_ready, reference_status


class NavigationReferenceTests(unittest.TestCase):
    def test_default_without_transform_stays_preview(self):
        self.assertEqual(select_reference('auto', False), 'preview')
        self.assertFalse(reference_ready(reference_status('preview')))

    def test_explicit_lidar_does_not_invent_body_or_joint_feedback(self):
        mode = select_reference('lidar_projection', False)
        report = dict(reference_status(mode), velocity_ready=True)
        self.assertTrue(reference_ready(report))
        self.assertFalse(report['body_reference_ready'])
        self.assertFalse(report['kinematic_feedback_ready'])
        report['velocity_ready'] = False
        self.assertFalse(reference_ready(report))

    def test_transform_and_reference_must_agree(self):
        for mode, body in [('body_kinematics', False), ('lidar_projection', True),
                           ('preview', True), ('anything', False)]:
            with self.subTest(mode=mode, body=body), self.assertRaises(ValueError):
                select_reference(mode, body)

    def test_body_and_legacy_reports_still_require_real_body_inputs(self):
        for verified in (False, True):
            for feedback in (False, True):
                report = reference_status('body_kinematics', body_verified=verified,
                                          body_feedback=feedback)
                self.assertEqual(reference_ready(report), verified and feedback)
        self.assertTrue(reference_ready(dict(body_reference_ready=True, kinematic_feedback_ready=True)))
        for report in (None, [], {'navigation_reference': 'preview', 'navigation_reference_ready': True},
                       {'navigation_reference': 'invalid', 'body_reference_ready': True, 'kinematic_feedback_ready': True}):
            self.assertFalse(reference_ready(report))

    def test_actual_gate_and_buffer_callbacks_use_explicit_reference(self):
        for file, name, callback, field in [('gate_node.py', 'GateNode', 'on_terrain', 'calibration_ready'),
                                           ('motion_buffer_node.py', 'MotionBufferNode', 'terrain', 'reference')]:
            tree = ast.parse((ROOT / 'navigation' / file).read_text())
            cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
            method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == callback)
            scope = dict(json=json, time=time, reference_ready=reference_ready)
            exec(compile(ast.Module(body=[method], type_ignores=[]), file, 'exec'), scope)
            node = NS()
            for ready in (True, False):
                report = dict(reference_status('lidar_projection'), velocity_ready=ready)
                scope[callback](node, NS(data=json.dumps(report)))
                self.assertIs(getattr(node, field), ready)
            scope[callback](node, NS(data='[]'))
            self.assertFalse(getattr(node, field))


if __name__ == '__main__':
    unittest.main()
