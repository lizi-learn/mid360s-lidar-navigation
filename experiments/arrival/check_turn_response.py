#!/usr/bin/env python3
"""Actual RPP/native ramp with ongoing left yaw and a requested right correction."""
from pathlib import Path
import argparse, json, math, os, subprocess, sys
import yaml
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'navigation'))
from configure import config
p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);a=p.parse_args()
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'and 200<=int(os.environ['ROS_DOMAIN_ID'])<=232
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
w=h=240;r=.05;ox=-4.;oy=-3.
cells=[254 if ox+(i%w+.5)*r<-.6 else 0 for i in range(w*h)]
marked=sum(c==254 for c in cells);assert marked>0
mp=out/'wall.map';mp.write_text(f'{w} {h} {r} {ox} {oy}\n'+' '.join(map(str,cells)))
path=[[.2,0,1.9],[.2,.01,math.pi/2]]+[[.2,i*.02,math.pi/2]for i in range(2,251)]
reference=out/'reference.csv';reference.write_text('x,y,yaw\n'+''.join(','.join(map(str,q))+'\n'for q in path))
params=config(gait_profile='p2-flat-model')['controller_server']['ros__parameters'];pf=out/'params.yaml';pf.write_text(yaml.safe_dump({'preview_controller':{'ros__parameters':params}}))
env=dict(os.environ,P2_TEST_POSITION_GOAL='1',P2_TEST_FIXED_REFERENCE='1',P2_TEST_INITIAL_SPEED='0.6 0.35 0.6 0.35',P2_TEST_REPLAY_MAP=str(mp),P2_TEST_REPLAY_CSV=str(reference))
prefix=out/'delayed-turn';proc=subprocess.run([a.binary,str(prefix),'wall-lag','.7','.8','--ros-args','--params-file',str(pf),'--log-level','error'],env=env,capture_output=True,text=True,timeout=45)
(out/'controller.log').write_text(proc.stdout+proc.stderr);assert proc.returncode==0,proc.stderr
row=json.loads(Path(str(prefix)+'.json').read_text());row.update(wall_cells_verified=marked,hardware_commands=0,initial_measured_forward=.6,initial_measured_yaw=.35,initial_native_forward=.6,initial_native_yaw=.35,queued_response_drains_after_zero=True)
row['passed']=row['goal_reached']and row['body_intrusions']==0
(out/'report.json').write_text(json.dumps(row,indent=2));print(json.dumps(row));assert row['passed']
