#!/usr/bin/env python3
"""Actual controller and position goal checker, synthetic planar tracking only."""
from pathlib import Path
import os,sys,argparse,json,subprocess,hashlib
import yaml
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'navigation'))
from configure import config
p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);rows=[]
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1' and 200<=int(os.environ['ROS_DOMAIN_ID'])<=232
for gain,delay,cohort in[(1.2,.3,'nominal'),(.8,.5,'heldout')]:
 for kind in('straight','gentle_s','corner','obstacle'):
  prefix=out.parent/(cohort+'-'+kind);params=config(gait_profile='p2-flat-model')['controller_server']['ros__parameters'];pf=prefix.with_suffix('.params.yaml');pf.write_text(yaml.safe_dump({'preview_controller':{'ros__parameters':params}}));env=dict(os.environ,P2_TEST_POSITION_GOAL='1');cmd=[a.binary,str(prefix),kind,str(gain),str(delay),'--ros-args','--params-file',str(pf),'--log-level','error'];proc=subprocess.run(cmd,env=env,capture_output=True,text=True,timeout=45);prefix.with_suffix('.log').write_text(proc.stdout+proc.stderr);assert proc.returncode==0,proc.stderr[-2000:]
  row=json.loads(prefix.with_suffix('.json').read_text());actual=json.loads(prefix.with_suffix('.actual-parameters.json').read_text());assert actual['min_lookahead_dist']==1.2 and actual['max_lookahead_dist']==1.6 and actual['use_collision_detection'] and not actual['allow_reversing'];assert len(actual['actual_footprint'])==4 and all(abs(abs(v)-.275)<1e-8 for q in actual['actual_footprint'] for v in q)
  row.update(cohort=cohort,position_goal=True,parameters=params,actual_parameters=actual)
  row['passed']=row['goal_reached']and row['goal_error_m']<.15 and row['body_intrusions']==0
  if kind=='obstacle':row['passed']=row['passed']and row['obstacle_verified_in_costmap']and row['controller_collision_stops']>0 and row['obstacle_cleared_after_two_seconds']
  rows.append(row);print(json.dumps({k:row[k]for k in('cohort','kind','passed','goal_error_m','goal_yaw_error_rad','body_intrusions','controller_collision_stops')}),flush=True);out.write_text(json.dumps({'complete':False,'cases':rows},indent=2))
ok=all(r['passed']for r in rows);out.write_text(json.dumps({'complete':True,'candidate_checks_passed':ok,'seed':0,'hardware_commands':0,'full_body_check_interval_s':.01,'identity':'Actual SettledGoalRPP and PositionGoalChecker; synthetic forward/yaw model and native input ramps, deliberately inconsistent final yaw, original0.15m XY. Confirmed dynamic obstacle injection/stop/resume. No physical gait or field perception certification.','cases':rows},indent=2));assert ok
