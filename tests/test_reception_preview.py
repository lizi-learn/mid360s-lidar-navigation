import copy
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from reception_catalog import blank,Catalog
from reception_maps import MapLibrary
from reception_preview import preview,preview_isolated


def point(i,x,y,**kw):
    return dict(id=i,name=i,x=x,y=y,yaw=kw.get('yaw'),product_id='',script=kw.get('script',''),
                kind=kw.get('kind','visit'),dwell=kw.get('dwell',0),guest_wait=kw.get('guest_wait',False))


class SavedPreview(unittest.TestCase):
    def test_saved_raster_thresholds_row_order_identity_and_source_unchanged(self):
        with tempfile.TemporaryDirectory()as t:
            root=Path(t);mp=root/'maps/floor';mp.mkdir(parents=True)
            image=b'P5\n3 2\n255\n'+bytes([0,254,205,254,0,205]);(mp/'mapped.pgm').write_bytes(image)
            (mp/'mapped.yaml').write_text(yaml.safe_dump(dict(image='mapped.pgm',origin=[1,2,0],resolution=.05,negate=0,occupied_thresh=.65,free_thresh=.196)))
            g=MapLibrary(root/'maps',Catalog(root/'catalog')).raster('floor')
            self.assertEqual(g['cells'],[0,100,-1,100,0,-1]);self.assertEqual(g['origin'],[1,2]);self.assertEqual(g['map_id'],'floor')
            self.assertEqual((mp/'mapped.pgm').read_bytes(),image)
            with self.assertRaises(ValueError):MapLibrary(root/'maps',None).raster('../floor')

    def fixture(self):
        cells=np.zeros((100,160),int);cells[[0,-1],:]=100;cells[:,[0,-1]]=100
        grid=dict(map_id='floor',width=160,height=100,resolution=.05,origin=[-1,-1],cells=cells.ravel().tolist())
        d=blank('floor');d['points']=[point('home',0,0,kind='home'),point('a',3,0,yaw=1.,script='介绍产品',dwell=4,guest_wait=True)]
        d['routes']=[dict(id='tour',name='tour',mode='showcase',loops=1,description='',steps=[dict(point_id='a',speak=True),dict(point_id='home',speak=False)])]
        return grid,d

    def test_task_order_heading_script_wait_and_dwell_no_input_mutation(self):
        g,d=self.fixture();oldg,oldd=copy.deepcopy(g),copy.deepcopy(d);r=preview(g,d,'tour');seq=r['samples']
        speaking=next(s for s in seq if s['phase']=='presenting');self.assertAlmostEqual(speaking['pose'][2],1.,places=3)
        self.assertEqual(speaking['script'],'介绍产品');self.assertLess(next(i for i,s in enumerate(seq)if s['phase']=='guest_wait'),next(i for i,s in enumerate(seq)if s['phase']=='dwell'))
        dwell=[s for s in seq if s['phase']=='dwell'];self.assertAlmostEqual(dwell[-1]['t']-dwell[0]['t'],4.,places=2)
        self.assertEqual(seq[-1]['phase'],'completed');self.assertEqual(seq[-1]['t'],r['duration_s']);self.assertEqual(r['hardware_commands'],0);self.assertEqual(g,oldg);self.assertEqual(d,oldd)

    def test_keepout_blocks_and_speed_zone_changes_preview_duration(self):
        g,d=self.fixture();before=preview(g,d,'tour')
        d['annotations']=[dict(id='slow',name='slow',kind='speed',factor=.5,vertices=[[.7,-.7],[2.3,-.7],[2.3,.7],[.7,.7]])]
        slow=preview(g,d,'tour');self.assertGreater(slow['duration_s'],before['duration_s']+5)
        d['annotations']=[dict(id='wall',name='wall',kind='keepout',width=.1,vertices=[[1,-1],[1,4]])]
        with self.assertRaisesRegex(ValueError,'第1站'):preview(g,d,'tour')

    def test_preview_rejects_another_map(self):
        g,d=self.fixture();g['map_id']='other'
        with self.assertRaisesRegex(ValueError,'不一致'):preview(g,d,'tour')

    def test_isolated_worker_returns_result_and_propagates_invalid_route(self):
        g,d=self.fixture();self.assertEqual(preview_isolated(g,d,'tour')['samples'][-1]['phase'],'completed')
        with self.assertRaisesRegex(ValueError,'选择'):preview_isolated(g,d,'missing')


if __name__=='__main__':unittest.main()
