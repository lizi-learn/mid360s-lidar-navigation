"""Actual RPP pivot comparison; synthetic yaw plant, no hardware connection."""
import argparse
import csv
import hashlib
import itertools
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'navigation'))
from configure import config

ap=argparse.ArgumentParser()
ap.add_argument('--binary',required=True)
ap.add_argument('--output',required=True)
ap.add_argument('--baseline-prefix')
ap.add_argument('--candidate-prefix')
args=ap.parse_args()
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'and 200<=int(os.environ['ROS_DOMAIN_ID'])<=232
out=Path(args.output).resolve().parent;out.mkdir(parents=True,exist_ok=True)
assert not any(out.iterdir()),'Use a new evidence directory'
params=config(gait_profile='p2-flat-model')['controller_server']['ros__parameters']
pf=out/'parameters.yaml';pf.write_text(yaml.safe_dump({'preview_controller':{'ros__parameters':params}}))
cases=[dict(cohort='slow_large',gain=g,delay=.15,deadzone=d,goal=a,scale=1.,kind='free')
       for g,d,a in itertools.product([.25,.35],[.32,.44],[-2.8,2.8])]
cases += [dict(cohort='delayed_probe',gain=g,delay=d,deadzone=z,goal=a,scale=1.,kind='free')
          for g,d,z in [(1.25,1.95,.32),(1.9,1.45,.44),(1.9,1.95,.32),(1.9,1.95,.44)]for a in [-1.57,1.57]]
cases += [dict(cohort='small',gain=.8,delay=1.1,deadzone=.35,goal=a,scale=sc,kind='free')
          for a,sc in itertools.product([-.4,.4],[.5,1.])]
for kind in ['blocked','cancel','endpoint']:
    g,d,z,a=(.8,.1,0.,.25)if kind=='endpoint'else(1.2,1.1,.35,.8)
    cases.append(dict(cohort=kind,gain=g,delay=d,deadzone=z,goal=a,scale=1.,kind=kind))
variants=[('candidate',args.candidate_prefix)]
if args.baseline_prefix:variants.insert(0,('baseline',args.baseline_prefix))
rows=[]
for label,prefix in variants:
    env=dict(os.environ)
    if prefix:
        env['AMENT_PREFIX_PATH']=prefix+':'+env.get('AMENT_PREFIX_PATH','')
        env['LD_LIBRARY_PATH']=prefix+'/lib:'+env.get('LD_LIBRARY_PATH','')
    for i,case in enumerate(cases):
        path=out/(label+'-'+str(i));row=dict(case,variant=label,index=i)
        command=[args.binary,str(path),str(case['gain']),str(case['delay']),str(case['deadzone']),str(case['goal']),case['kind'],
                 '--ros-args','--params-file',str(pf),'--log-level','error']
        p=subprocess.run(command,env=dict(env,P2_TEST_INPUT_SCALE=str(case['scale'])),capture_output=True,text=True,timeout=12)
        path.with_suffix('.log').write_text(p.stdout+p.stderr);row['returncode']=p.returncode
        if p.returncode==0:
            row.update(json.loads(path.with_suffix('.json').read_text()))
            loaded=Path(row['controller_library']).resolve()
            row['controller_library_sha256']=hashlib.sha256(loaded.read_bytes()).hexdigest()
            if prefix:assert str(loaded).startswith(str(Path(prefix).resolve())+'/'),str(loaded)
            safe=row['body_intrusions']==0 and row['max_policy_input']<=case['scale']+1e-6
            if case['kind']=='free':row['passed']=safe and row['goal_reached']and not row['progress_failed']and row['error_rad']<.15
            elif case['kind']=='blocked':row['passed']=safe and row['obstacle_verified']and row['collision_stops']>0
            elif case['kind']=='cancel':row['passed']=safe and row['cancelled']and not row['post_cancel_nonzero']and abs(row['final_measured_rate'])<.03
            else:
                first=next(csv.DictReader(path.with_suffix('.csv').open()))
                row['passed']=safe and row['obstacle_verified']and row['goal_reached']and float(first['raw_z'])>0. and int(first['collision_exception'])==0
        else:row['passed']=False
        rows.append(row)
        if i%6==0:print(json.dumps({k:row[k]for k in ['variant','index','cohort','passed']}),flush=True)
        (out/'progress.json').write_text(json.dumps(rows,indent=2))
summary={label:dict(cases=len([r for r in rows if r['variant']==label]),failed=sum(not r['passed']for r in rows if r['variant']==label))for label,_ in variants}
comparison=[]
if args.baseline_prefix:
    for i in range(len(cases)):
        a=next(r for r in rows if r['variant']=='baseline'and r['index']==i)
        b=next(r for r in rows if r['variant']=='candidate'and r['index']==i)
        comparison.append(dict(index=i,cohort=b['cohort'],baseline_passed=a['passed'],candidate_passed=b['passed'],
                               baseline_duration=a.get('duration_s'),candidate_duration=b.get('duration_s'),
                               baseline_starts=a.get('raw_starts'),candidate_starts=b.get('raw_starts')))
report=dict(summary=summary,cases=rows,comparison=comparison,seed=0,hardware_commands=0,
            scope='Actual Nav2 controller, heading/progress checkers, complete .55m square and native input arithmetic. Synthetic pure-yaw delay/deadzone plant; not physical or biped validation.')
Path(args.output).write_text(json.dumps(report,indent=2));print(json.dumps(summary),flush=True)
raise SystemExit(0 if all(r['passed']for r in rows if r['variant']=='candidate')else 1)
