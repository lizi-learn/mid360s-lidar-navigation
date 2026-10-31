"""Run actual adapter callback code without ROS or actuator I/O.
Fresh source poses retain their original stamps; polling a cached pose is not
new evidence, and expiration/invalid data must suppress TF output.
"""
import ast,json,math,sys,threading,unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from odometry_velocity import PlanarVelocity

def vector():return NS(x=0.,y=0.,z=0.)
def quaternion():return NS(x=0.,y=0.,z=0.,w=1.)
def header(stamp=0.):
    ns=round(stamp*1e9);return NS(stamp=NS(sec=ns//10**9,nanosec=ns%10**9),frame_id='')
def transform():return NS(header=header(),child_frame_id='',transform=NS(translation=vector(),rotation=quaternion()))
def odometry(stamp=0.,x=0.):
    position=vector();position.x=x
    return NS(header=header(stamp),child_frame_id='',pose=NS(pose=NS(position=position,orientation=quaternion())),twist=NS(twist=NS(linear=vector(),angular=vector())))
class Bool:
    def __init__(self):self.data=False
class String:
    def __init__(self):self.data=''
p=Path(__file__).resolve().parents[1]/'navigation/live_adapter.py'
cls=next(n for n in ast.parse(p.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='Adapter')
scope={'np':np,'math':math,'json':json,'Rotation':Rotation,'Bool':Bool,'String':String,'TransformStamped':transform,'Odometry':odometry}
exec(compile(ast.Module(body=[cls],type_ignores=[]),str(p),'exec'),scope)

class AdapterTimingTests(unittest.TestCase):
    def fixture(self):
        n=scope['Adapter'].__new__(scope['Adapter']);n.lock=threading.Lock();n.body_reference=None;n.reference_fault='';n.previous=None
        n.rotation=np.eye(3);n.initial_height=0.;n.velocity=PlanarVelocity();n.published_odom_stamp=None
        n.alignment_mtime=1;n.alignment_path=NS(stat=lambda:NS(st_mtime_ns=1));n.now=1.01
        n.sink=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=round(n.now*1e9))))
        n.transforms=[];n.odometry=[];n.ground=[];n.status=[]
        n.tf=NS(sendTransform=n.transforms.append);n.odom_pub=NS(publish=n.odometry.append);n.ground_pub=NS(publish=n.ground.append);n.status_pub=NS(publish=n.status.append)
        self.sample(n,1.,0.);n.tick();return n
    def sample(self,n,stamp,x):
        n.cloud=NS(header=header(stamp));n.processed_stamp=stamp
        n.on_odom(odometry(stamp,x));n.now=stamp+.01
    def test_next_poll_forwards_fresh_pose_with_exact_source_stamp(self):
        n=self.fixture();self.sample(n,1.2,.02);n.tick()
        self.assertEqual(len(n.transforms),1);self.assertEqual(len(n.odometry),1)
        self.assertEqual(n.transforms[0].header.stamp,n.odom.header.stamp)
        self.assertEqual(n.odometry[0].header.stamp,n.odom.header.stamp)
        self.assertAlmostEqual(n.odometry[0].twist.twist.linear.x,.1)
    def test_repeated_polls_never_renew_cached_tf_or_odometry(self):
        n=self.fixture();self.sample(n,1.2,.02);n.tick()
        for i in range(1,10):n.now=1.21+i*.02;n.tick()
        self.assertEqual(len(n.transforms),1);self.assertEqual(len(n.odometry),1)
    def test_expired_or_invalid_pose_stops_output(self):
        n=self.fixture();self.sample(n,1.2,.02);n.tick();n.now=1.8;n.tick()
        self.assertEqual(len(n.transforms),1);self.assertFalse(n.ground[-1].data)
        self.sample(n,1.9,float('nan'));n.tick()
        self.assertEqual(len(n.transforms),1);self.assertFalse(n.ground[-1].data)
        self.assertEqual(n.reference_fault,'invalid odometry pose')
    def test_second_new_source_pose_is_forwarded_without_waiting_for_new_cloud(self):
        n=self.fixture();self.sample(n,1.2,.02);n.tick()
        n.on_odom(odometry(1.3,.03));n.now=1.31;n.tick()
        self.assertEqual(len(n.transforms),2);self.assertEqual(n.transforms[-1].header.stamp,n.odom.header.stamp)
        self.assertEqual(n.processed_stamp,1.2)

if __name__=='__main__':unittest.main()
