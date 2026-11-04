import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from pose_history import PoseHistory
from scan_ranges import polar_ranges


def q(yaw):return (0.,0.,math.sin(yaw/2),math.cos(yaw/2))
def yaw(quaternion):
    x,y,z,w=quaternion
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def project(point,pose):
    dx=np.asarray(point)-pose.position[:2]
    scan=polar_ranges(np.array([dx]),yaw(pose.quaternion))
    i=int(np.flatnonzero(np.isfinite(scan))[0]);a=-math.pi+i*2*math.pi/len(scan)+yaw(pose.quaternion)
    return np.array(pose.position[:2])+scan[i]*np.array([math.cos(a),math.sin(a)])

class ProjectionTimeTest(unittest.TestCase):
    def test_cloud_time_interpolation_restores_static_wall(self):
        h=PoseHistory();t=1_793_600_000_000_000_000
        h.add(t,(0.,0.,0.),q(0.));h.add(t+200_000_000,(.2,0.,0.),q(.3))
        sample=h.at(t+100_000_000);latest=h.at(t+200_000_000)
        point=np.array([8.,1.]);good=project(point,sample)
        # Old code generated bearings with latest pose, then consumers used TF
        # at the cloud's unchanged earlier stamp. Reproduce that defect explicitly.
        delta=point-latest.position[:2];a=math.atan2(delta[1],delta[0])-yaw(latest.quaternion)
        bad=np.asarray(sample.position[:2])+np.linalg.norm(delta)*np.array([math.cos(a+yaw(sample.quaternion)),math.sin(a+yaw(sample.quaternion))])
        self.assertGreater(np.linalg.norm(bad-point),1.)
        self.assertLess(np.linalg.norm(good-point),.04)
        self.assertEqual(sample.stamp_ns,t+100_000_000)
        self.assertEqual(sample.bracket_ns,(t,t+200_000_000))
    def test_no_extrapolation_waits_for_real_bracket(self):
        h=PoseHistory();h.add(1_000_000_000,(0.,0.,0.),q(0))
        self.assertIsNone(h.at(1_050_000_000));self.assertIsNone(h.at(999_999_999))
        h.add(1_100_000_000,(.1,0.,0.),q(.1))
        self.assertAlmostEqual(h.at(1_050_000_000).position[0],.05)
        self.assertIsNone(h.at(1_200_000_000))
    def test_wrap_and_quaternion_sign_use_short_rotation(self):
        h=PoseHistory();h.add(1,(0.,0.,0.),q(math.radians(179)))
        h.add(101,(1.,0.,0.),tuple(-v for v in q(math.radians(-179))))
        self.assertAlmostEqual(abs(yaw(h.at(51).quaternion)),math.pi)
    def test_original_span_and_invalid_values(self):
        h=PoseHistory();h.add(1,(0.,0.,0.),q(0));h.add(500_000_002,(1.,0.,0.),q(0))
        self.assertIsNone(h.at(100))
        for stamp,p,orientation in [(0,(0,0,0),q(0)),(2,(float('nan'),0,0),q(0)),(2,(0,0,0),(0,0,0,0))]:
            with self.assertRaises(ValueError):h.add(stamp,p,orientation)
    def test_out_of_order_and_bounded_retention(self):
        h=PoseHistory(duration_ns=1_000,max_span_ns=1_000,limit=6)
        for i in [5,1,3,2,4,6,7,8,9,10]:h.add(i,(float(i),0.,0.),q(0))
        self.assertLessEqual(len(h.times),6);self.assertEqual(h.at(10).position[0],10.)
        self.assertIsNone(h.at(1))
    def test_nearest_bins_have_no_systematic_clockwise_shift(self):
        errors=[]
        for angle in np.linspace(-math.pi,math.pi,721,endpoint=False)+.0017:
            point=11*np.array([math.cos(angle),math.sin(angle)])
            scan=polar_ranges(np.array([point]),0.);i=int(np.flatnonzero(np.isfinite(scan))[0])
            error=math.remainder(-math.pi+i*2*math.pi/len(scan)-angle,2*math.pi);errors.append(error)
            self.assertLessEqual(abs(error),math.pi/720+1e-12)
        self.assertLess(abs(np.mean(errors)),1e-5)
    def test_empty_unknown_wrap_nearest_and_measured_beyond_range(self):
        self.assertTrue(np.isnan(polar_ranges(np.empty((0,2)),0.)).all())
        scan=polar_ranges(np.array([[-2.,.00001],[-1.,-.00001]]),0.)
        self.assertEqual(np.isfinite(scan).sum(),1);self.assertAlmostEqual(scan[0],1.)
        self.assertTrue(np.isinf(polar_ranges(np.array([[20.,0.]]),0.)[360]))
        self.assertTrue(np.isnan(polar_ranges(np.array([[.01,0.]]),0.)).all())
