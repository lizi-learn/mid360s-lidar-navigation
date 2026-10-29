import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from survey import shifted_grid,wrap,distance_step
from nav_geometry import FloorMap
class SurveyTests(unittest.TestCase):
    def test_export_keeps_unknown_and_obstacle_overrides_floor(self):
        p=np.array([[-2.,1.,0.],[-2.,1.,1.],[-1.,2.,0.]])
        origin,pgm=shifted_grid(p);grid=pgm[::-1]
        cells=np.floor((p[:,:2]-origin)/.1+1e-8).astype(int)
        self.assertEqual(grid[cells[0,1],cells[0,0]],0)
        self.assertEqual(grid[cells[2,1],cells[2,0]],254)
        self.assertEqual(grid[0,0],205)
    def test_stationary_jitter_does_not_inflate_path_length(self):
        anchor=[0.,0.];length=0.
        for x in [.003,-.004,.002,-.001]*100:
            d,anchor=distance_step(anchor,[x,0.]);length+=d
        self.assertEqual(length,0)
        d,anchor=distance_step(anchor,[.03,0.]);self.assertAlmostEqual(d,.03)

    def test_heading_crossing_wrap_is_small(self):
        self.assertAlmostEqual(abs(wrap(-np.pi+.01-(np.pi-.01))),.02)
    def test_rolling_far_jump_does_not_wrap_old_obstacles(self):
        m=FloorMap(size=100,resolution=.1,radius=0)
        m.update(np.array([[0.,0.,1.]]));m.update(np.array([[0.,0.,1.]]))
        m.recenter(-40.,30.)
        self.assertFalse(m.obstacles.any());self.assertIsNone(m.cell(0.,0.))
        m.update(np.array([[-40.,30.,1.]]));m.update(np.array([[-40.,30.,1.]]))
        self.assertTrue(m.obstacles[m.cell(-40.,30.)])
if __name__=='__main__':unittest.main()
