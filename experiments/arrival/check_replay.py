#!/usr/bin/env python3
"""Optional closed-loop replay of user-supplied maps and paths. Inputs are not public data."""
from pathlib import Path
import argparse,json,os,sys,subprocess,yaml
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'navigation'))
from configure import config
p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--planner-results',required=True);p.add_argument('--cases',nargs='+',required=True);p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);folder=Path(a.planner_results);rows=[]
for name in a.cases:
 prefix=out.parent/name;cfg=config(gait_profile='p2-flat-model');params=cfg['controller_server']['ros__parameters'];params['GridBased']=cfg['planner_server']['ros__parameters']['PositionOnly'];pf=prefix.with_suffix('.params.yaml');pf.write_text(yaml.safe_dump({'preview_controller':{'ros__parameters':params}}));env=dict(os.environ,P2_TEST_POSITION_GOAL='1',P2_TEST_REPLAY_CSV=str(folder/(name+'-PositionOnlyLattice.csv')),P2_TEST_REPLAY_MAP=str(folder/(name+'.map.txt')));cmd=[a.binary,str(prefix),'replay','1.2','.3','--ros-args','--params-file',str(pf),'--log-level','error'];
 try:r=subprocess.run(cmd,env=env,capture_output=True,text=True,timeout=180)
 except subprocess.TimeoutExpired as e:
  from types import SimpleNamespace
  text=lambda v:v.decode(errors='replace')if isinstance(v,bytes)else(v or '')
  r=SimpleNamespace(returncode=124,stdout=text(e.stdout),stderr=text(e.stderr)+'\n180s bounded replay timeout')
 prefix.with_suffix('.log').write_text(r.stdout+r.stderr)
 if r.returncode:row={'case':name,'returncode':r.returncode,'error':r.stderr[-2000:],'passed':False}
 else:
  row=json.loads(prefix.with_suffix('.json').read_text());row['case']=name;row['passed']=row['goal_reached']and row['goal_error_m']<.15 and row['body_intrusions']==0
 rows.append(row);print(json.dumps(row),flush=True);out.write_text(json.dumps({'complete':False,'cases':rows},indent=2))
ok=all(r['passed']for r in rows);out.write_text(json.dumps({'complete':True,'candidate_checks_passed':ok,'seed':0,'hardware_commands':0,'identity':'Actual planner+controller+position checker with1Hz replanning; supplied frozen map and native response synthetic gain1.2/delay0.3s plant. Full55cm square0.01s samples. Planning computations pause simulated time: this is functional replay, not real-time/biped/field validation.','cases':rows},indent=2));raise SystemExit(0 if ok else 1)
