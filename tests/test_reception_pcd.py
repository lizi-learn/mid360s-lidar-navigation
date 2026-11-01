import struct,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from reception_pcd import convert,read_xyz

class PCDTests(unittest.TestCase):
    def pcd(self,root,data,mode='ascii',count=3):
        p=Path(root)/'scene.pcd';p.write_bytes(f'VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {count}\nHEIGHT 1\nPOINTS {count}\nDATA {mode}\n'.encode()+data);return p
    def test_ascii_actual_samples_finite_filter_and_deterministic_selection(self):
        with tempfile.TemporaryDirectory()as root:
            p=self.pcd(root,b'0 1 2 3\nnan 2 3 4\n4 5 6 7\n')
            a,r=convert(p,'showroom','map',maximum=1)
            self.assertEqual(a['points'],[[0.,1.,2.]]);self.assertEqual(r['finite_points'],2)
            with self.assertRaises(ValueError):convert(p,'showroom','camera_init')
    def test_binary_rigid_transform_and_no_scale_guess(self):
        with tempfile.TemporaryDirectory()as root:
            p=self.pcd(root,struct.pack('<4f',1.,2.,3.,.5),'binary',1)
            a,r=convert(p,'showroom','camera_init',[[0,-1,0,4],[1,0,0,5],[0,0,1,6],[0,0,0,1]])
            self.assertEqual(a['points'],[[2.,6.,9.]]);self.assertTrue(r['transform_applied'])
            with self.assertRaises(ValueError):convert(p,'showroom','map',[[2,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    def test_truncated_binary_wrong_count_and_unsupported_compression_are_explicit(self):
        with tempfile.TemporaryDirectory()as root:
            for data,mode,count in [(b'','binary',1),(b'1 2 3 4\n','ascii',2),(b'','binary_compressed',1)]:
                with self.assertRaises(ValueError):read_xyz(self.pcd(root,data,mode,count))
if __name__=='__main__':unittest.main()
