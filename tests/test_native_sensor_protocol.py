import unittest,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from native_backend import sensor_sample,PACKET
class ModelSensorProtocol(unittest.TestCase):
 def setUp(self):self.sample=[1.,0.,0.,.9,1.,0.,0.,0.,0.,0.,.9,1.,0.,0.,0.,.1]
 def line(self,values):return 'SENSOR '+' '.join(map(str,values))
 def test_pose_pair_and_joint_sample_and_existing_lease_contract(self):
  self.assertEqual(sensor_sample(self.line(self.sample)),tuple(self.sample));self.assertEqual(PACKET.size,48)
 def test_truncation_nan_bad_quaternion_and_negative_time_rejected(self):
  for values in [self.sample[:-1],[-1.]+self.sample[1:],self.sample[:4]+[0.]*4+self.sample[8:],self.sample[:8]+[float('nan')]+self.sample[9:]]:
   with self.assertRaises(ValueError):sensor_sample(self.line(values))
if __name__=='__main__':unittest.main()
