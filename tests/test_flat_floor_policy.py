from pathlib import Path
import sys,unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from terrain import ground_decision,forward_lower_surface,local_floor

class FlatFloorPolicyTests(unittest.TestCase):
    def test_explicit_flat_floor_assumption_does_not_relabel_missing_support(self):
        missing=False
        self.assertFalse(ground_decision('observed_support',missing,False))
        self.assertTrue(ground_decision('flat_floor',missing,False))
        self.assertFalse(missing)
        with self.assertRaises(ValueError):ground_decision('other',missing,False)

    def test_actual_lower_return_stops_both_modes_including_between_old_patches(self):
        for x,y in ((.7,0.),(.3,0.),(.9,.3)):
            lower=forward_lower_surface(np.array([[x,y,-.2]]),[0,0],0,[0,0,0])
            self.assertTrue(lower)
            for mode in ('observed_support','flat_floor'):
                self.assertFalse(ground_decision(mode,True,lower))
        self.assertFalse(forward_lower_surface(np.array([[.7,0.,0.]]),[0,0],0,[0,0,0]))

    def test_detection_uses_current_heading_and_local_plane(self):
        p=np.array([[0.,.7,-.2]])
        self.assertTrue(forward_lower_surface(p,[0,0],np.pi/2,[0,0,0]))
        self.assertFalse(forward_lower_surface(p,[0,0],0,[0,0,0]))
        self.assertFalse(forward_lower_surface(np.array([[.7,0.,1.]]),[0,0],0,[0,0,1.]))

    def test_missing_plane_cannot_be_used_as_flat_floor_evidence(self):
        with self.assertRaises(ValueError):local_floor(np.empty((0,3)),[0,0])

if __name__=='__main__':unittest.main()
