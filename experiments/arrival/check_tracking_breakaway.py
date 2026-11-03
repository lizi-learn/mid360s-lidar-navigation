#!/usr/bin/env python3
"""Native tracking-to-pivot handoff with a synthetic yaw deadzone and input lag."""
from pathlib import Path
import argparse,json,math,os,subprocess,sys
import yaml
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'navigation'))
from configure import config
p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);p.add_argument('--allow-failure',action='store_true');a=p.parse_args()
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1' and 200<=int(os.environ['ROS_DOMAIN_ID'])<=232
out=Path(a.output);out.mkdir(parents=True,exist_ok=True);rows=[]
params=config(gait_profile='p2-flat-model')['controller_server']['ros__parameters'];pf=out/'params.yaml';pf.write_text(yaml.safe_dump({'preview_controller':{'ros__parameters':params}}))
for side in (-1,1):
 for deadzone in (.3,.55):
  name=f'{side}-{deadzone}';prefix=out/name;w=h=240;r=.05;ox=oy=-3.
  cells=[254 if side*(ox+(i%w+.5)*r)>.45 else 0 for i in range(w*h)]
  mp=out/(name+'.map');mp.write_text(f'{w} {h} {r} {ox} {oy}\n'+' '.join(map(str,cells)))
  reference=out/(name+'.reference.csv');reference.write_text('x,y,yaw\n'+f'0,0,{math.pi/2-side*.18}\n'+''.join(f'0,{i*.02},{math.pi/2}\n'for i in range(1,251)))
  env=dict(os.environ,P2_TEST_POSITION_GOAL='1',P2_TEST_FIXED_REFERENCE='1',P2_TEST_YAW_DEADZONE=str(deadzone),P2_TEST_REPLAY_MAP=str(mp),P2_TEST_REPLAY_CSV=str(reference))
  proc=subprocess.run([a.binary,str(prefix),'tracking-breakaway','.7','.8','--ros-args','--params-file',str(pf),'--log-level','info'],env=env,capture_output=True,text=True,timeout=45)
  (out/(name+'.log')).write_text(proc.stdout+proc.stderr);assert proc.returncode==0,proc.stderr
  row=json.loads(Path(str(prefix)+'.json').read_text())
  row.update(side=side,synthetic_deadzone=deadzone,wall_cells_verified=sum(c==254 for c in cells),hardware_commands=0)
  assert 'Ongoing turn intersects map' in proc.stdout+proc.stderr,'Tracking brake not exercised'
  row['passed']=row['goal_reached']and row['body_intrusions']==0
  rows.append(row);print(json.dumps(row),flush=True)
ok=all(r['passed']for r in rows);(out/'report.json').write_text(json.dumps({'passed':ok,'scope':'Actual controller, native input ramp and progress checker; synthetic deadzone/delay plant, not physical D calibration','cases':rows},indent=2))
if not a.allow_failure:assert ok
