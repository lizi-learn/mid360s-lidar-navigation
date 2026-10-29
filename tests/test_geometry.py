import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import unittest
import numpy as np
from nav_geometry import level_rotation, FloorMap

class GeometryTests(unittest.TestCase):
    def test_inverted_gravity_preserves_right_handed_coordinates(self):
        a=np.array([-.02,-.01,-.9995]);r=level_rotation(a)
        np.testing.assert_allclose(r@a/np.linalg.norm(a),[0,0,1],atol=1e-10)
        self.assertAlmostEqual(np.linalg.det(r),1)
        self.assertGreater((r@np.array([1,0,0]))[0],.99)
    def test_unknown_and_inflated_obstacle_are_not_traversable(self):
        m=FloorMap(size=60,resolution=.1,radius=.2)
        m.floor[10:50,10:50]=True;m.obstacles[10:50,30]=True
        with self.assertRaises(ValueError):m.plan((-1,0),(1,0))
        with self.assertRaises(ValueError):m.plan((-1,0),(2.8,2.8))
        m.obstacles[25:35,30]=False
        path=m.plan((-1,0),(1,0));self.assertGreater(len(path),10)
    def test_no_corner_cutting(self):
        m=FloorMap(size=10,radius=0)
        m.floor[:]=True;m.obstacles[4,5]=True;m.obstacles[5,4]=True
        path=m.plan((-.05,-.05),(.05,.05))
        self.assertGreater(len(path),2)
    def test_isolated_returns_and_old_obstacles_expire(self):
        m=FloorMap(size=40,radius=0)
        obstacle=np.array([[.15,.15,.5]])
        m.update(obstacle);self.assertFalse(m.obstacles.any())
        m.update(obstacle);self.assertTrue(m.obstacles.any())
        for _ in range(20):m.update(np.empty((0,3)))
        self.assertFalse(m.obstacles.any())

    def test_rolling_map_preserves_world_location_and_expires_hits(self):
        m=FloorMap(size=100,resolution=.1,radius=0)
        pts=np.array([[2.15,.15,.5],[2.15,1.15,0.]])
        m.update(pts);m.update(pts)
        before=m.world(m.cell(2.15,.15));m.recenter(4.,2.)
        self.assertTrue(m.obstacles[m.cell(2.15,.15)])
        np.testing.assert_allclose(m.world(m.cell(2.15,.15)),before)
        self.assertFalse(m.obstacles[:,-10:].any())
        for _ in range(20):m.update(np.empty((0,3)))
        self.assertFalse(m.obstacles.any())

if __name__=='__main__':unittest.main()
