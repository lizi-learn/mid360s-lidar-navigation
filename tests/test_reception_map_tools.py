import base64,io,json,math,sys,tempfile,time,unittest,zipfile
from pathlib import Path
import numpy as np,yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from reception_catalog import Catalog,blank,validate
from reception_annotations import AnnotationLayer,footprint_free,nearby_candidates
from reception_maps import MapLibrary,BundleTransfer,read_pgm,local_patch,sha
from reception_history import History

BODY=[[.275,.275],[.275,-.275],[-.275,-.275],[-.275,.275]]
def map_files(path,cells=None,origin=(0.,0.,0.)):
 path=Path(path);path.mkdir(parents=True,exist_ok=True);cells=np.full((20,20),254,dtype=np.uint8)if cells is None else cells
 (path/'mapped.pgm').write_bytes(f'P5\n{cells.shape[1]} {cells.shape[0]}\n255\n'.encode()+cells.tobytes())
 (path/'mapped.yaml').write_text(yaml.safe_dump(dict(image='mapped.pgm',mode='trinary',resolution=.1,origin=list(origin),negate=0,free_thresh=.196,occupied_thresh=.65)))
 (path/'mapped.data').write_bytes(b'synthetic serialized scans, not a real graph');(path/'mapped.posegraph').write_bytes(b'synthetic graph')

class AnnotationTests(unittest.TestCase):
 def test_layers_are_reversible_full_body_and_do_not_clear_unknown(self):
  grid=dict(width=40,height=40,resolution=.1,origin=[-2.,-2.],cells=[0]*1600);grid['cells'][0]=-1;layer=AnnotationLayer()
  line=dict(id='wall',name='禁行线',kind='keepout',vertices=[[-1.,0.],[1.,0.]],width=.02)
  d=validate(dict(blank('m'),annotations=[line]));out,receipt=layer.apply(grid,d['annotations'])
  self.assertEqual(grid['cells'][800],0);self.assertEqual(out[0],-1);self.assertGreater(receipt['keepout_cells'],0)
  self.assertFalse(footprint_free(dict(grid,cells=out),0.,.1,0.,BODY));self.assertTrue(footprint_free(grid,0.,.1,0.,BODY))
  self.assertEqual(layer.apply(grid,[])[0],grid['cells'])
  grid['cells'][20*40+20]=100;self.assertFalse(footprint_free(grid,0,0,math.pi/4,BODY))
 def test_speed_body_entry_overlap_and_exit(self):
  one=dict(id='s',kind='speed',name='慢行',vertices=[[0,0],[1,0],[1,1],[0,1]],factor=.5)
  two=dict(one,id='t',factor=.3);rows=validate(dict(blank('m'),annotations=[one,two]))['annotations']
  self.assertEqual(AnnotationLayer.speed(rows,[-.2,.5,0])[0],.3)
  self.assertEqual(AnnotationLayer.speed(rows,[-.6,.5,0]),(1.,[]))
 def test_nearby_retains_unknown_and_occupied_cells(self):
  g=dict(width=40,height=40,resolution=.1,origin=[-2.,-2.],cells=[0]*1600);g['cells'][20*40+20]=100
  rows=nearby_candidates(g,dict(x=0,y=0,yaw=0),BODY,radius=1.)
  self.assertTrue(rows);self.assertTrue(all(footprint_free(g,p['x'],p['y'],p['yaw'],BODY)and not p['planner_verified']for p in rows))
  self.assertEqual(g['cells'][820],100)
  with self.assertRaises(ValueError):validate(dict(blank('m'),annotations=[dict(id='a',kind='speed',vertices=[[0,0],[1,0]],factor=.5)]))

class MapToolTests(unittest.TestCase):
 def test_export_import_new_identity_preserves_content_and_map_hash(self):
  with tempfile.TemporaryDirectory()as tmp:
   p=Path(tmp);catalog=Catalog(p/'catalog');doc=blank('office');doc['points']=[dict(id='home',name='出发点',x=1.,y=1.,kind='home')];catalog.save(doc,0)
   map_files(p/'maps/office');library=MapLibrary(p/'maps',catalog);one=library.snapshot('office');raw,manifest=library.export('office')
   result=library.import_bundle(raw,'office_1005');self.assertTrue(result['can_resume_mapping']);self.assertEqual(catalog.read('office_1005')['points'][0]['name'],'出发点')
   self.assertEqual(sha((p/'maps/office_1005/mapped.pgm').read_bytes()),manifest['files']['map/mapped.pgm']['sha256'])
   origin=json.loads((p/'maps/office_1005/site-origin.json').read_text());self.assertFalse(origin['calibration_copied']);self.assertFalse(origin['localization_copied']);self.assertNotEqual(origin['new_installation_id'],manifest['bundle_id'])
   (p/'maps/office/mapped.pgm').write_bytes(b'P5\n20 20\n255\n'+bytes(400));library.restore('office',one['id']);self.assertTrue(np.all(read_pgm(p/'maps/office/mapped.pgm')==254))
   with self.assertRaises(ValueError):library.import_bundle(raw,'office')
 def test_import_tampered_member_and_path_never_commits(self):
  with tempfile.TemporaryDirectory()as tmp:
   p=Path(tmp);c=Catalog(p/'catalog');c.save(blank('office'),0);map_files(p/'maps/office');lib=MapLibrary(p/'maps',c);raw,_=lib.export('office')
   for name,value in [('map/mapped.pgm',b'tampered'),('../escaped',b'x')]:
    memory=io.BytesIO()
    with zipfile.ZipFile(io.BytesIO(raw))as src,zipfile.ZipFile(memory,'w')as dest:
     for item in src.namelist():dest.writestr(item,value if item==name else src.read(item))
     if name not in src.namelist():dest.writestr(name,value)
    with self.assertRaises(ValueError):lib.import_bundle(memory.getvalue(),'bad')
    self.assertFalse((p/'maps/bad').exists());self.assertEqual(c.read('bad')['revision'],0)
 def test_chunks_idempotent_partial_and_conflicting_uploads(self):
  with tempfile.TemporaryDirectory()as tmp:
   p=Path(tmp);c=Catalog(p/'c');c.save(blank('a'),0);map_files(p/'maps/a');t=BundleTransfer(MapLibrary(p/'maps',c));meta=t.request(dict(operation='bundle_export',map_id='a'));parts=[base64.b64decode(t.request(dict(operation='bundle_chunk',transfer_id=meta['transfer_id'],index=i))['data'])for i in range(meta['chunks'])];raw=b''.join(parts);self.assertEqual(sha(raw),meta['sha256']);t.request(dict(operation='bundle_abort',transfer_id=meta['transfer_id']))
   begin=t.request(dict(operation='bundle_begin',new_map_id='b',bytes=len(raw),sha256=sha(raw)))
   with self.assertRaises(ValueError):t.request(dict(operation='bundle_commit',transfer_id=begin['transfer_id']))
   for i in range(begin['chunks']):
    part=dict(operation='bundle_part',transfer_id=begin['transfer_id'],index=i,data=base64.b64encode(raw[i*4096:(i+1)*4096]).decode());t.request(part);t.request(part)
   self.assertEqual(t.request(dict(operation='bundle_commit',transfer_id=begin['transfer_id']))['imported'],'b')
 def test_local_update_same_frame_preserves_outside_and_unknown(self):
  with tempfile.TemporaryDirectory()as tmp:
   p=Path(tmp);a=np.full((20,20),254,dtype=np.uint8);b=np.zeros((30,30),dtype=np.uint8);b[10:15,10:15]=205
   map_files(p/'old',a);map_files(p/'new',b,(-.5,-.5,0));r=local_patch(p/'old',p/'new',[[.5,.5],[1.5,.5],[1.5,1.5],[.5,1.5]])
   out=read_pgm(p/'new/mapped.pgm');self.assertEqual(out.shape,a.shape);self.assertEqual(r['outside_region_changed'],0);self.assertGreater(r['changed_cells'],0);self.assertEqual(out[0,0],254)
 def test_original_source_clocks_and_missing_data_survive_pagination(self):
  with tempfile.TemporaryDirectory()as tmp:
   h=History(Path(tmp)/'events.sqlite3')
   try:
    for i in range(3):h.append(dict(kind='telemetry',run_id='r',map_id='m',wall_time=100.+i,capture_monotonic_ns=20+i,pose_source_ros_ns=5,pose=None,gate={'command':None}))
    h.pending.join();first=h.trace('m','r',limit=2);last=h.trace('m','r',after=first['after'],limit=2)
    self.assertEqual(len(first['samples'])+len(last['samples']),3);self.assertTrue(all(s['pose_source_ros_ns']=='5' and s['pose']is None for s in first['samples']));self.assertEqual(h.runs('m')['runs'][0]['samples'],3);self.assertEqual(h.runs('other')['runs'],[])
   finally:h.close()

class AutomaticVoiceTests(unittest.TestCase):
 def test_busy_save_coalesces_new_revision_and_reports_real_exit(self):
  import subprocess
  from reception_voice import VoiceCache
  from reception_catalog import content_key
  with tempfile.TemporaryDirectory()as tmp:
   p=Path(tmp);script=p/'tts.py';script.write_text('import json,sys,time,wave\nd=json.load(sys.stdin)\ntime.sleep(.15)\nif d["text"]=="bad":sys.exit(7)\nw=wave.open(d["output"],"wb");w.setparams((1,2,8000,0,"NONE","none"));w.writeframes(b"\\0\\0"*1600);w.close()\n')
   v=VoiceCache(p/'cache',[sys.executable,str(script)])
   def d(texts):return dict(map_id='m',points=[dict(script=t,product_id='')for t in texts],products=[])
   v.auto_prepare(d(['first','obsolete']))
   end=time.monotonic()+2
   while not v.current and time.monotonic()<end:time.sleep(.01)
   result=v.auto_prepare(d(['latest','bad']));self.assertTrue(result['merged'])
   end=time.monotonic()+4
   while v.busy and time.monotonic()<end:time.sleep(.01)
   status=v.status(d(['latest','bad']))
   self.assertFalse(v.busy);self.assertTrue(status['clips'][content_key('latest')]['available']);self.assertEqual(status['clips'][content_key('bad')]['returncode'],7)
   self.assertFalse((p/'cache'/(content_key('obsolete')+'.wav')).exists());self.assertEqual(v.summary('m')['total'],2)
