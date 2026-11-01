import json,sys,unittest,time
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from path_view import encode_view,decode_view
from test_console_cancellation import fixture

class PathViewTests(unittest.TestCase):
    def test_every_point_and_original_source_time_survive_roundtrip(self):
        points=[[i*.05,(i%17)*.01] for i in range(812)]
        result,stamp=decode_view(encode_view(points,123456789000))
        self.assertEqual(points,result);self.assertEqual(stamp,123456789000)
    def test_invalid_source_or_nonfinite_coordinates_cannot_become_fresh_plan(self):
        for value in [{'version':1,'source_ros_ns':0,'points':[]},
                      {'version':1,'source_ros_ns':True,'points':[]},
                      {'version':1,'source_ros_ns':1,'points':[[1,float('nan')]]},
                      {'version':1,'source_ros_ns':1,'points':[[1,True]]}]:
            with self.assertRaises((ValueError,TypeError)):decode_view(json.dumps(value))
    def test_console_uses_source_age_not_relay_receipt_to_resume(self):
        c,g=fixture();now=time.monotonic()
        c.session.planned(g,8,now);c.session.start(g,now,True,True);c.session.started(g,now)
        c.session.obstacle(now);c.path_revision=c.obstacle_path_revision=2
        c.session.clear_since=now-.8  # Existing clearance dwell already met.
        c.collision_stop=False;c.health_ready=lambda:True
        c.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=2_000_000_000))
        stale=encode_view([[0.,0.],[1.,0.]],1_400_000_000)
        c.on_path_view(SimpleNamespace(data=stale));self.assertEqual(c.session.phase,'replanning')
        fresh=encode_view([[0.,0.],[1.,0.]],1_900_000_000)
        c.on_path_view(SimpleNamespace(data=fresh));self.assertEqual(c.session.phase,'executing')
        self.assertEqual(c.path,[[0.,0.],[1.,0.]])

if __name__=='__main__':unittest.main()
