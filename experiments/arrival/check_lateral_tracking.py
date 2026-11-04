"""Actual forward-only controller against a close obstacle and lateral plant drift."""
from pathlib import Path
import argparse,csv,json,math,os,subprocess,sys
import yaml
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'navigation'))
from configure import config
p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);a=p.parse_args()
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'and 200<=int(os.environ['ROS_DOMAIN_ID'])<=232
out=Path(a.output);out.mkdir(parents=True,exist_ok=True);params=config(gait_profile='p2-flat-model')['controller_server']['ros__parameters'];pf=out/'params.yaml';pf.write_text(yaml.safe_dump({'preview_controller':{'ros__parameters':params}}));rows=[]
for sign in [-1.,1.]:
 w,h,res=160,120,.05;cells=[]
 for iy in range(h):
  for ix in range(w):
   x,y=(ix+.5)*res,(iy+.5)*res
   if sign>0:y=6.-y
   cells.append(254 if ix in(0,w-1)or iy in(0,h-1)or 2.5<=x<=4. and y<=2.15 else 0)
 mp=out/(str(sign)+'.map');mp.write_text(f'{w} {h} {res} 0 0\n'+' '.join(map(str,cells)))
 points=[]
 for aa,bb in [((1.2,2.5),(2.2,2.85)),((2.2,2.85),(4.4,2.85)),((4.4,2.85),(6.,2.5))]:
  length=math.dist(aa,bb);angle=math.atan2(bb[1]-aa[1],bb[0]-aa[0]);n=math.ceil(length/.02)
  for i in range(n):
   f=i/n;x,y=aa[0]+f*(bb[0]-aa[0]),aa[1]+f*(bb[1]-aa[1]);points.append((x,6-y if sign>0 else y,-angle if sign>0 else angle))
 points.append((6.,3.5 if sign>0 else 2.5,0.));reference=out/(str(sign)+'.csv');reference.write_text('x,y,yaw\n'+''.join(','.join(map(str,q))+'\n'for q in points))
 for gain,delay in [(.7,.3),(.95,.6)]:
  prefix=out/(str(sign)+'-'+str(gain));env=dict(os.environ,P2_TEST_POSITION_GOAL='1',P2_TEST_FIXED_REFERENCE='1',P2_TEST_REPLAY_MAP=str(mp),P2_TEST_REPLAY_CSV=str(reference),P2_TEST_LATERAL_RATIO=str(sign*.16))
  proc=subprocess.run([a.binary,str(prefix),'lateral-obstacle',str(gain),str(delay),'--ros-args','--params-file',str(pf),'--log-level','error'],env=env,capture_output=True,text=True,timeout=35);Path(str(prefix)+'.log').write_text(proc.stdout+proc.stderr);assert proc.returncode==0,proc.stderr
  row=json.loads(Path(str(prefix)+'.json').read_text());assert row['lateral_ratio']==sign*.16 and row['commanded_y']==0
  row.update(obstacle_cells_verified=sum(c==254 for c in cells),passed=row['goal_reached']and row['body_intrusions']==0);rows.append(row);print(json.dumps(row),flush=True)
report=dict(passed=all(r['passed']for r in rows),cases=rows,seed=0,hardware_commands=0,scope='Actual native controller, original55cm square and input ramp; mirrored synthetic near-obstacle scenes with lateral drift. No physical D-policy calibration.');(out/'report.json').write_text(json.dumps(report,indent=2));assert report['passed']
