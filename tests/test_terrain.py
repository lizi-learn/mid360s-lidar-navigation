import unittest,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from terrain import local_floor,classify,observed_front_support

class TerrainTests(unittest.TestCase):
    def test_sloped_floor_and_table_remain_separate(self):
        rng=np.random.default_rng(3)
        xy=rng.uniform(-3,3,(20000,2));z=.03*xy[:,0]+.02*xy[:,1]+rng.normal(0,.005,len(xy))
        ground=np.column_stack((xy,z));furniture=np.column_stack((rng.uniform(.5,2,(3000,2)),rng.uniform(.4,1.1,3000)))
        plane,report=local_floor(np.vstack((ground,furniture)),[0,0])
        self.assertLess(abs(plane[0]-.03),.006);self.assertLess(abs(plane[1]-.02),.006)
        g,o=classify(np.vstack((ground,furniture)),[0,0],plane)
        self.assertGreater(g[:len(ground)].mean(),.99);self.assertGreater(o[len(ground):].mean(),.99)
        self.assertTrue(observed_front_support(ground,[0,0],0,plane))
    def test_missing_front_floor_is_not_filled(self):
        rng=np.random.default_rng(4);xy=rng.uniform(-3,3,(10000,2));xy=xy[xy[:,0]<.4]
        points=np.column_stack((xy,np.zeros(len(xy))))
        plane,_=local_floor(points,[0,0]);self.assertFalse(observed_front_support(points,[0,0],0,plane))
    def test_height_reference_tracks_local_floor(self):
        rng=np.random.default_rng(8);xy=rng.uniform(-3,3,(12000,2));points=np.column_stack((xy,.8+.02*xy[:,0]))
        plane,_=local_floor(points,[0,0],expected_height=.8)
        self.assertAlmostEqual(plane[2],.8,places=4)
    def test_insufficient_or_steep_floor_rejected(self):
        with self.assertRaises(ValueError):local_floor(np.zeros((10,3)),[0,0])
        rng=np.random.default_rng(5);xy=rng.uniform(-2,2,(5000,2));points=np.column_stack((xy,.5*xy[:,0]))
        with self.assertRaises(ValueError):local_floor(points,[0,0])

if __name__=='__main__':unittest.main()
