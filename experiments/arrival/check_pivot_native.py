import os,json,itertools,subprocess,sys,argparse
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'navigation'))
ap=argparse.ArgumentParser();ap.add_argument('--binary',required=True);ap.add_argument('--output',required=True);args=ap.parse_args()
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'and 200<=int(os.environ['ROS_DOMAIN_ID'])<=232
from configure import config
out=Path(args.output).resolve().parent;out.mkdir(parents=True,exist_ok=True);assert not any(out.iterdir()),'Use a new evidence directory';rows=[]
cohorts=[('original_large',list(itertools.product([.8,1.2],[.3,.8,1.1],[.28,.35],[1.]))),
 ('half_scale',list(itertools.product([.55,1.2],[.3,1.1,1.95],[.28,.44],[-.4,.4,-2.8,2.8]))),
 ('sweep_a',list(itertools.product([.55,1.25,1.9],[.55,1.45,1.95],[.32,.44],[-.4,.4,-2.8,2.8]))),
 ('sweep_b',list(itertools.product([.62,1.05,1.65],[.65,1.35,1.85],[.31,.425],[-.47,.47,-2.65,2.65])))]
for candidate in [False,True]:
 params=config(gait_profile='p2-flat-model')['controller_server']['ros__parameters'];params['FollowPath']['settled_pivot_response']=candidate;params['progress_checker']['plugin']='p2_nav_controller::RotationProgressChecker'if candidate else'nav2_controller::SimpleProgressChecker';params['goal_checker']['plugin']='p2_nav_controller::SettledHeadingGoalChecker'if candidate else'nav2_controller::SimpleGoalChecker'
 pf=out/('candidate.yaml'if candidate else'baseline.yaml');pf.write_text(yaml.safe_dump({'preview_controller':{'ros__parameters':params}}))
 for cohort,cases in cohorts:
  # Paired comparisons retain every failure. These public fixed sweeps
  # are reproducible regressions, not fresh independent field evidence.
  for index,(gain,delay,dead,goal)in enumerate(cases):
   prefix=out/f'{int(candidate)}-{cohort}-{index:03d}'
   cmd=[args.binary,str(prefix),str(gain),str(delay),str(dead),str(goal),'free','--ros-args','--params-file',str(pf),'--log-level','error']
   r=subprocess.run(cmd,env=dict(os.environ,P2_TEST_INPUT_SCALE='.5'if cohort=='half_scale'else'1.'),capture_output=True,text=True,timeout=12);prefix.with_suffix('.log').write_text(r.stdout+r.stderr)
   row={'cohort':cohort,'candidate':candidate,'gain':gain,'delay':delay,'deadzone':dead,'goal':goal,'kind':'free','returncode':r.returncode}
   if r.returncode==0:
    row.update(json.loads(prefix.with_suffix('.json').read_text()));row['passed']=row['goal_reached']and row['error_rad']<.15 and row['body_intrusions']==0 and not row['progress_failed'] and row['max_policy_input']<=row['operator_scale']+1e-6
   else:row['passed']=False
   rows.append(row)
   if index%24==0:print(json.dumps({k:row[k]for k in('cohort','candidate','gain','delay','goal','passed')}),flush=True)
   (out/'progress.json').write_text(json.dumps(rows,indent=2))
 # Obstacle response and explicit deactivation are both measured at the actual
 # plugin. The synthetic transport queue drains; it is never erased magically.
 for kind in ['blocked','cancel']:
  prefix=out/f'{int(candidate)}-{kind}';cmd=[args.binary,str(prefix),'1.2','1.1','.35','.8',kind,'--ros-args','--params-file',str(pf),'--log-level','error']
  r=subprocess.run(cmd,capture_output=True,text=True,timeout=12);prefix.with_suffix('.log').write_text(r.stdout+r.stderr)
  row={'cohort':kind,'candidate':candidate,'kind':kind,'returncode':r.returncode}
  if r.returncode==0:
   row.update(json.loads(prefix.with_suffix('.json').read_text()));row['passed']=row['body_intrusions']==0 and ((kind=='blocked'and row['obstacle_verified']and row['collision_stops']>0)or(kind=='cancel'and row['cancelled']and not row['post_cancel_nonzero']and abs(row['final_measured_rate'])<.03))
  else:row['passed']=False
  rows.append(row)
summary={}
for candidate in[False,True]:
 for cohort,_ in cohorts:
  rs=[r for r in rows if r['cohort']==cohort and r['candidate']==candidate];failed=[r for r in rs if not r['passed']]
  summary[str(int(candidate))+'-'+cohort]={'cases':len(rs),'failed':len(failed),'failures':failed,'maximum_duration_s':max((r.get('duration_s',0)for r in rs),default=0)}
report={'summary':summary,'cases':rows,'hardware_commands':0,'scope':'Actual SettledGoalRPP and selected heading/progress checkers. Goal acceptance immediately stops further commands, matching controller-server completion; queued plant transport drains instead of being erased. Controlled ROS simulation time. Synthetic pure-yaw deadzone/delay/first-order plant; not biped/field validation.','full_body_interval_s':.01,'seed':0}
Path(args.output).write_text(json.dumps(report,indent=2));print(json.dumps({k:{a:b for a,b in v.items()if a!='failures'}for k,v in summary.items()}),flush=True)
raise SystemExit(0 if all(r['passed']for r in rows if r['candidate']or r['kind']!='free')else 1)
