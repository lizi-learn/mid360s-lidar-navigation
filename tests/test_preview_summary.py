"""Health polling must not render the parallel LiDAR preview map."""
import ast,json,math,pathlib,sys,threading,time,unittest
from collections import deque
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from nav_geometry import FloorMap
from sensor_watchdog import data_stale

tree=ast.parse((ROOT/'nav_server.py').read_text())
scope={'Node':object,'time':time,'math':math,'np':np,'json':json,'RUNTIME':ROOT/'tests/nonexistent-runtime',
       'ROBOT_IDENTITY':{'name':'synthetic-fixture'}}
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef)and n.name=='NavigationView')
exec(compile(ast.Module(body=[cls],type_ignores=[]),'nav_server.py','exec'),scope)

class PreviewSummaryTests(unittest.TestCase):
    def fixture(self):
        n=scope['NavigationView'].__new__(scope['NavigationView']);n.lock=threading.RLock()
        n.points=np.array([[1.,2.,3.]]);n.recent_acc=deque();n.odom_time=time.monotonic()
        n.imu_count=41;n.cloud_count=7;n.initial_tilt=0.;n.height=1.5;n.sensor_z=0.
        n.localization_error=n.floor_error='';n.sensor_xy=[0.,0.];n.sensor_yaw=n.sensor_pitch=0.
        n.path=[];n.goal=None;n.path_state='';n.floor_map=FloorMap(size=20)
        n.floor_map.layers=Mock(wraps=n.floor_map.layers);n.survey=NS(status=Mock(return_value={'active':False}))
        n.message_age=lambda kind:{'cloud':.12,'imu':.03,'odom':.15}[kind]
        return n
    def test_summary_retains_source_health_without_map_or_survey_work(self):
        n=self.fixture();d=n.status(include_cells=False)
        self.assertEqual(d['counts'],{'imu':41,'map_frames':7});self.assertEqual(d['raw_age'],.12)
        self.assertEqual(d['imu_age'],.03);self.assertEqual(d['odom_age'],.15)
        self.assertFalse(data_stale(d));n.floor_map.layers.assert_not_called();n.survey.status.assert_not_called()
        for key in ['points','free_cells','obstacle_cells','survey']:self.assertNotIn(key,d)
    def test_full_web_preview_keeps_its_existing_data(self):
        n=self.fixture();d=n.status()
        self.assertEqual(d['points'],[[1.,2.,3.]]);self.assertIn('free_cells',d);self.assertIn('obstacle_cells',d)
        self.assertEqual(d['survey'],{'active':False});n.survey.status.assert_called_once();n.floor_map.layers.assert_called_once()
    def test_stalled_source_remains_stalled_in_summary(self):
        n=self.fixture();n.message_age=lambda _:3.1
        self.assertTrue(data_stale(n.status(include_cells=False)))
    def test_unobserved_ros_preview_does_not_construct_map(self):
        n=self.fixture();n.map_pub=NS(get_subscription_count=lambda:0,publish=Mock())
        n.publish_map();n.floor_map.layers.assert_not_called();n.map_pub.publish.assert_not_called()

if __name__=='__main__':unittest.main()
