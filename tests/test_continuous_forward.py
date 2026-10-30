import importlib.util
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[0]
# This file is installed under tests/ in the repository by the staging helper.
if not (ROOT/'experiments').is_dir():ROOT=ROOT.parent
EXPERIMENT=ROOT/'experiments/continuous_forward'

class ContinuousForwardTests(unittest.TestCase):
    def test_frozen_footprint_accepts_winding_changes_and_rejects_crossed_or_changed_shapes(self):
        spec=importlib.util.spec_from_file_location('replay_contract',EXPERIMENT/'replay_contract.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        points=[[-.58,-.72],[1.,-.72],[1.,.72],[-.58,.72]]
        for order in [points,list(reversed(points))]:
            for shift in range(4):self.assertTrue(module.nominal_stop_footprint(order[shift:]+order[:shift]))
        self.assertFalse(module.nominal_stop_footprint([points[i] for i in [0,2,1,3]]))
        self.assertFalse(module.nominal_stop_footprint([[-.57,-.72],[1.,-.72],[1.,.72],[-.57,.72]]))
        self.assertFalse(module.nominal_stop_footprint([[float('nan'),-.72],*points[1:]]))

    def test_primitives_are_exact_forward_rays_or_pure_pivots(self):
        data=json.loads((EXPERIMENT/'biped-straight-pivot32.json').read_text())
        metadata=data['lattice_metadata'];angles=metadata['heading_angles']
        self.assertEqual(metadata['num_of_headings'],32)
        self.assertEqual(metadata['number_of_trajectories'],96)
        self.assertEqual(metadata['grid_resolution'],.05)
        self.assertEqual(angles,sorted(angles))
        for primitive in data['primitives']:
            before=[0.,0.,angles[primitive['start_angle_index']]]
            moving=primitive['trajectory_length']>0.
            for pose in primitive['poses']:
                dx,dy=pose[0]-before[0],pose[1]-before[1]
                self.assertLessEqual(math.hypot(dx,dy),.01250001)
                self.assertLessEqual(abs(math.remainder(pose[2]-before[2],2*math.pi)),.02000001)
                if moving:
                    self.assertGreater(dx*math.cos(pose[2])+dy*math.sin(pose[2]),0.)
                    self.assertAlmostEqual(-dx*math.sin(pose[2])+dy*math.cos(pose[2]),0.,places=12)
                else:self.assertEqual(pose[:2],[0.,0.])
                before=pose
            self.assertAlmostEqual(math.remainder(before[2]-angles[primitive['end_angle_index']],2*math.pi),0.,places=12)
            for coordinate in before[:2]:self.assertAlmostEqual(coordinate/.05,round(coordinate/.05),places=10)

    def test_primitive_generation_is_reproducible(self):
        with tempfile.TemporaryDirectory() as name:
            directory=Path(name);script=directory/'generate_primitives.py';shutil.copy2(EXPERIMENT/script.name,script)
            subprocess.run([sys.executable,str(script)],check=True,stdout=subprocess.PIPE)
            self.assertEqual((directory/'biped-straight-pivot32.json').read_bytes(),(EXPERIMENT/'biped-straight-pivot32.json').read_bytes())

    def test_inflation_covers_padded_stopping_rectangle_and_gates_are_preserved(self):
        spec=importlib.util.spec_from_file_location('continuous_forward_config',EXPERIMENT/'configure.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        candidate=module.parameters();baseline=module.baseline_config(gait_profile='p2-flat-model')
        grid=candidate['global_costmap']['global_costmap']['ros__parameters']
        footprint=json.loads(grid['footprint']);padding=grid['footprint_padding']
        radius=max(math.hypot(x+math.copysign(padding,x),y+math.copysign(padding,y)) for x,y in footprint)
        self.assertGreaterEqual(grid['inflation_layer']['inflation_radius'],radius+grid['resolution'])
        self.assertEqual(grid['footprint'],baseline['global_costmap']['global_costmap']['ros__parameters']['footprint'])
        self.assertEqual(grid['footprint_padding'],.05)
        for key in baseline:
            if key not in ['planner_server','global_costmap']:self.assertEqual(candidate[key],baseline[key],key)
        self.assertEqual(candidate['planner_server']['ros__parameters']['GridBased']['lookup_table_size'],.05)

if __name__=='__main__':unittest.main()
