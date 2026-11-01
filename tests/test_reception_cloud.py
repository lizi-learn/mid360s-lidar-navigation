import sys,tempfile,unittest,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from reception_cloud import CloudStore
from reception_catalog import Catalog,blank
class CloudTest(unittest.TestCase):
    def test_private_roundtrip_no_costmap_output(self):
        with tempfile.TemporaryDirectory()as root:
            s=CloudStore(root);d={'map_id':'test','frame_id':'map','units':'m','points':[[0.,1.,2.]]}
            self.assertEqual(s.save(d)['scope'],'preview_only');self.assertEqual(s.read('test'),d);self.assertEqual(s.read('other')['points'],[])
    def test_bad_frame_units_counts_coordinates(self):
        good={'map_id':'test','frame_id':'map','units':'m','points':[[0.,1.,2.]]}
        for key,value in [('frame_id','lidar'),('units','cm'),('points',[[True,0,0]]),('points',[[float('nan'),0,0]]),('points',[[0,1]]),('points',[[0,1,2]]*20001),('map_id','../../bad')]:
            with self.assertRaises(ValueError):CloudStore.validate({**good,key:value})
    def test_catalog_file_map_identity(self):
        with tempfile.TemporaryDirectory()as root:
            Path(root,'one.json').write_text(json.dumps(blank('two')))
            with self.assertRaises(ValueError):Catalog(root).read('one')
if __name__=='__main__':unittest.main()
