import json,pathlib,sys,tempfile,unittest,yaml
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'navigation'))
from configure import config,apply_directional_stop

class DirectionalStopTests(unittest.TestCase):
    def test_uses_same_body_padding_threshold_and_preserves_other_nodes(self):
        body=[[.275,.275],[.275,-.275],[-.275,-.275],[-.275,.275]]
        for padding in (0.,.05):
            with tempfile.TemporaryDirectory()as folder:
                path=pathlib.Path(folder)/'params.yaml';before=config(gait_profile='p2-flat-model',footprint_vertices=body,footprint_padding=padding)
                path.write_text(yaml.safe_dump(before));apply_directional_stop(path);after=yaml.safe_load(path.read_text())
                for node in before:
                    if node!='collision_monitor':self.assertEqual(before[node],after[node])
                m=after['collision_monitor']['ros__parameters'];points=m['StopZone']['points']
                self.assertAlmostEqual(max(points[::2])-min(points[::2]),.55+2*padding)
                self.assertEqual(m['StopZone']['max_points'],m['SweptZone']['max_points'])
                self.assertEqual(m['StopZone']['max_points'],before['collision_monitor']['ros__parameters']['StopZone']['max_points'])
                self.assertTrue(m['stop_on_predicted_collision'])
                self.assertGreater(.275+.75*m['SweptZone']['time_before_collision'],.805)
                self.assertEqual(m['source_timeout'],before['collision_monitor']['ros__parameters']['source_timeout'])
                apply_directional_stop(path);self.assertEqual(yaml.safe_load(path.read_text()),after)

if __name__=='__main__':unittest.main()
