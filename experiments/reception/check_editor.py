"""Real editor DOM actions plus actual catalog schema and storage validation."""
import json,subprocess,sys,tempfile,argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'navigation'))
from reception_ui import reception_html,embedded_reception_html
from reception_catalog import blank,Catalog,validate
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
d=blank('fixture');d['revision']=1;d['points']=[dict(id='home',name='旧点名',x=0,y=0,yaw=None,kind='home',product_id='',script='',dwell=0,guest_wait=False)];d['routes']=[dict(id='tour',name='接待路线',mode='showcase',loops=1,description='',steps=[dict(point_id='home',speak=True)])]
payload=dict(direct=reception_html(ROOT/'navigation'),embedded=embedded_reception_html(ROOT/'navigation'),fixture=d)
r=subprocess.run(['node',str(Path(__file__).with_suffix('.cjs'))],input=json.dumps(payload,ensure_ascii=False),capture_output=True,text=True,timeout=30)
if r.returncode:raise RuntimeError(r.stderr)
report=json.loads(r.stdout)
submitted=report.pop('payloads')
for body in submitted:
 value=validate(body['reception']['document'])
 with tempfile.TemporaryDirectory()as tmp:
  store=Catalog(tmp);value['revision']=0;saved=store.save(value,0);assert store.read('fixture')==saved
report['actual_catalog_save_roundtrips']=len(submitted);out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2));print(json.dumps(report))
