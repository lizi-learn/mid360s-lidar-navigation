import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from operator_authority import ReceiverAuthority

class AuthorityTests(unittest.TestCase):
    def sample(self,raw=282):
        a=[0.]*37;a[0]=100_000_000;a[1:17]=[1002.]*16;a[5]=raw;return a
    def test_raw_three_positions_and_unmasked_intervention(self):
        for raw,mode in [(282,'navigation_allowed'),(1002,'paused'),(1722,'manual')]:
            d=ReceiverAuthority(True);a=self.sample(raw)
            self.assertEqual(d.decode(a,b'gid',100.001,10.,7)['switch_mode'],mode)
            a[0]+=10000;a[3]=1100 # mapped axes remain zero in E-left
            self.assertTrue(d.decode(a,b'gid',100.011,10.01,7)['manual_activity'])
    def test_invalid_raw_cannot_look_like_center(self):
        o=ReceiverAuthority(True).decode(self.sample(666),b'g',100.001,10.,1)
        self.assertFalse(o['switch_mapping_verified']);self.assertEqual(o['switch_mode'],'unknown')
    def test_stale_replay_original_time_and_clock_jump(self):
        d=ReceiverAuthority(True);a=self.sample()
        o=d.decode(a,b'g',100.001,10.,1)
        self.assertAlmostEqual(o['source_monotonic_ns']/1e9,9.999,places=5)
        self.assertFalse(d.decode(a,b'g',100.01,10.009,1)['receiver_connected'])
        a[0]+=10000
        self.assertFalse(d.decode(a,b'g',100.5,10.499,1)['receiver_connected'])
        a[0]=101_000_000
        self.assertEqual(d.decode(a,b'g',101.,10.5,1)['fault'],'host_clock_changed')
        a[0]+=10000
        self.assertEqual(d.decode(a,b'g',101.01,11.009,1)['fault'],'host_clock_changed')
    def test_flags_and_uncommissioned(self):
        for i in (17,18,32,36):
            a=self.sample();a[i]=1
            self.assertFalse(ReceiverAuthority(True).decode(a,b'g',100.001,10.,1)['receiver_connected'])
        self.assertFalse(ReceiverAuthority(False).decode(self.sample(),b'g',100.001,10.,1)['switch_mapping_verified'])
        a=self.sample();a[0]=float('nan')
        self.assertFalse(ReceiverAuthority(True).decode(a,b'g',100.,10.,1)['receiver_connected'])
if __name__=='__main__':unittest.main()
