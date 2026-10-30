import importlib.util
import json
import math
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
EXPERIMENT=ROOT/'experiments/gait_space'

class GaitSpaceTests(unittest.TestCase):
    def test_optional_profile_preserves_controller_and_health_contracts(self):
        spec=importlib.util.spec_from_file_location('gait_space_config',EXPERIMENT/'configure.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        value=module.parameters();baseline=module.baseline_config(gait_profile='p2-flat-model')
        for key in baseline:
            if key not in ('planner_server','global_costmap'):self.assertEqual(value[key],baseline[key],key)
        planner=value['planner_server']['ros__parameters']['GridBased']
        self.assertEqual(planner['gait_max_midroute_rotation'],math.pi/2)
        self.assertEqual(planner['gait_min_forward_leg'],.5)
        self.assertEqual(planner['gait_terminal_region'],.3)
        self.assertFalse(planner['allow_reverse_expansion'])
        grid=value['global_costmap']['global_costmap']['ros__parameters']
        self.assertEqual(grid['footprint'],baseline['global_costmap']['global_costmap']['ros__parameters']['footprint'])
        self.assertEqual(grid['footprint_padding'],.05)
        self.assertEqual(grid['inflation_layer']['inflation_radius'],1.4)
        self.assertTrue(grid['obstacle_layer']['enabled'])
        frozen=module.parameters(frozen_replay=True)
        self.assertFalse(frozen['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['enabled'])
        with self.assertRaises(ValueError):module.parameters(headings=8)

    def test_forward_primitives_match_independently_checked_experiment(self):
        self.assertEqual((EXPERIMENT/'biped-straight-pivot32.json').read_bytes(),(ROOT/'experiments/continuous_forward/biped-straight-pivot32.json').read_bytes())
        proof=json.loads((EXPERIMENT/'validation-2026-10-30.json').read_text())
        self.assertFalse(proof['whole_navigation_accepted'])
        audit=proof['independent_audit']
        self.assertEqual(len(audit['checks']),8)
        self.assertTrue(all(row['samples']>1 and row['filled_violations']==row['nav2_cpp_violations']==0 for row in audit['checks']))

if __name__=='__main__':unittest.main()
