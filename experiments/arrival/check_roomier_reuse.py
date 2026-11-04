"""Actual planner: a new right-side obstacle should prefer available left room."""
from pathlib import Path
import argparse,csv,json,math,os,subprocess,sys
import yaml
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'navigation'))
from configure import config
from reception_annotations import footprint_free
p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);p.add_argument('--baseline',action='store_true');p.add_argument('--planner',choices=['PositionOnlyLattice','ExactHeadingLattice'],default='PositionOnlyLattice');a=p.parse_args()
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'and 200<=int(os.environ['ROS_DOMAIN_ID'])<=232
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
w,h,res=160,120,.05
cells=[254 if x in(0,w-1)or y in(0,h-1)else 0 for y in range(h)for x in range(w)]
scene=out/'scene.map';scene.write_text(f'{w} {h} {res} 0 0\n'+' '.join(map(str,cells)))
params=config(gait_profile='p2-flat-model')['planner_server']['ros__parameters']['PositionOnly' if a.planner=='PositionOnlyLattice' else 'GridBased'];pf=out/'params.yaml';pf.write_text(yaml.safe_dump({'arrival_planner':{'ros__parameters':{'GridBased':params}}}))
prefix=out/'sequence';r=subprocess.run([a.binary,str(scene),str(prefix),'p2_nav_controller::'+a.planner,'1','2.5','0','6','2.5','0','--ros-args','--params-file',str(pf),'--log-level','error'],env=dict(os.environ,P2_ARRIVAL_ROOMIER_SEQUENCE='1'),text=True,capture_output=True,timeout=30)
(out/'planner.log').write_text(r.stdout+r.stderr);assert r.returncode==0,r.stderr
body=[[.275,.275],[.275,-.275],[-.275,-.275],[-.275,.275]];reports=[]
for name in ['roomier','narrow']:
 rows=[[float(v[k])for k in ['x','y','yaw']]for v in csv.DictReader(Path(str(prefix)+'-'+name+'.csv').open())]
 current=list(cells);marked=0
 for y in range(h):
  for x in range(w):
   wx,wy=(x+.5)*res,(y+.5)*res
   if 2.5<=wx<=4. and(wy<=2.15 or(name=='narrow'and wy>=2.85)):
    current[y*w+x]=254;marked+=1
 grid=dict(width=w,height=h,resolution=res,origin=[0,0],cells=[100 if c>=254 else 0 for c in current]);bad=0
 for i,b in enumerate(rows):
  aa=rows[max(0,i-1)];da=math.remainder(b[2]-aa[2],2*math.pi);n=max(1,math.ceil(math.dist(aa[:2],b[:2])/.01),math.ceil(abs(da)/.01))
  for k in range(n+1):
   f=k/n;q=[aa[0]+f*(b[0]-aa[0]),aa[1]+f*(b[1]-aa[1]),aa[2]+f*da]
   if not footprint_free(grid,*q,body):bad+=1
 max_y=max(y for x,y,yaw in rows if 2.5<=x<=4.)
 reports.append(dict(kind=name,obstacle_cells_verified=marked,body_intrusions=bad,maximum_y_over_obstacle=max_y,roomier_left_used=max_y>2.65,seconds=json.loads(Path(str(prefix)+'-'+name+'.json').read_text())['seconds']))
closed=json.loads(Path(str(prefix)+'-closed.json').read_text());passed=all(v['body_intrusions']==0 and v['obstacle_cells_verified']>0 for v in reports)and closed['rejected']and(not a.baseline and reports[0]['roomier_left_used']or a.baseline)
report=dict(passed=passed,planner=a.planner,cases=3,results=reports,closed=closed,hardware_commands=0,seed=0,scope='Actual native planner and new-obstacle replanning with original55cm square; independent1cm filled-body checks. Synthetic map, no physical biped claim.');(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report));assert passed
