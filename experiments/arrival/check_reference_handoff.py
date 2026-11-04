"""Actual RPP/planner rejoin regression; modest prefixes may track as curves."""
from pathlib import Path
import argparse,csv,json,math,os,subprocess,sys,hashlib
import yaml
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'navigation'))
from configure import config
p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);a=p.parse_args()
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'and 200<=int(os.environ['ROS_DOMAIN_ID'])<=232
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
params=config(gait_profile='p2-flat-model')['controller_server']['ros__parameters'];pf=out/'params.yaml';pf.write_text(yaml.safe_dump({'preview_controller':{'ros__parameters':params}}));rows=[]
cases=[('left45',math.pi/4,0.),('right45',-math.pi/4,0.),('left45_drift',math.pi/4,-.12),('right45_drift',-math.pi/4,-.12),('reverse',math.pi,0.)]
for name,angle,drift in cases:
 w=h=260;res=.05;cells=[254 if ix in(0,w-1)or iy in(0,h-1)else 0 for iy in range(h)for ix in range(w)];mp=out/(name+'.map');mp.write_text(f'{w} {h} {res} 0 0\n'+' '.join(map(str,cells)))
 start=(6.,6.,0.);end=(start[0]+4*math.cos(angle),start[1]+4*math.sin(angle));points=[start]+[(start[0]+4*i/200*math.cos(angle),start[1]+4*i/200*math.sin(angle),angle)for i in range(1,201)];ref=out/(name+'.csv');ref.write_text('x,y,yaw\n'+''.join(','.join(map(str,q))+'\n'for q in points))
 prefix=out/(name+'-trace');env=dict(os.environ,P2_TEST_POSITION_GOAL='1',P2_TEST_REPLAY_MAP=str(mp),P2_TEST_REPLAY_CSV=str(ref),P2_TEST_LATERAL_RATIO=str(drift))
 proc=subprocess.run([a.binary,str(prefix),name,'.7','.6','--ros-args','--params-file',str(pf),'--log-level','error'],env=env,capture_output=True,text=True,timeout=40);Path(str(prefix)+'.log').write_text(proc.stdout+proc.stderr)
 assert proc.returncode==0,proc.stderr
 row=json.loads(Path(str(prefix)+'.json').read_text());trace=list(csv.DictReader(Path(str(prefix)+'.csv').open()));moving=[r for r in trace if float(r['input_x'])>.1]
 actual=json.loads(Path(str(prefix)+'.actual-parameters.json').read_text())
 assert actual['use_collision_detection']and not actual['allow_reversing']
 assert len(actual['actual_footprint'])==4 and all(abs(abs(v)-.275)<1e-8 for q in actual['actual_footprint']for v in q)
 row.update(case=name,first_forward_at=float(moving[0]['t'])if moving else None,raw_translation_zero_s=sum(float(r['raw_x'])<.01 for r in trace)*.05,reference_sha256=hashlib.sha256(ref.read_bytes()).hexdigest(),map_sha256=hashlib.sha256(mp.read_bytes()).hexdigest(),passed=row['goal_reached']and row['body_intrusions']==0)
 if name in ('left45','right45'):row['passed']=row['passed']and row['first_forward_at']is not None and row['first_forward_at']<1.
 if name=='reverse':row['passed']=row['passed']and bool(moving)and abs(float(moving[0]['yaw']))>2.7
 row['actual_parameters']=actual;rows.append(row);print(json.dumps(row),flush=True)
report=dict(passed=all(r['passed']for r in rows),cases=rows,hardware_commands=0,scope='Actual planner/controller and native input ramp; original55cm square; synthetic delayed plant and .12 rightward drift ratio, not body calibration or physical walking. Replans remain enabled.');(out/'report.json').write_text(json.dumps(report,indent=2));assert report['passed']
