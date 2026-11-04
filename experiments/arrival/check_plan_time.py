import argparse,csv,json,math,subprocess,sys
from pathlib import Path
import numpy as np,yaml
p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--binary',required=True);p.add_argument('--output',required=True);p.add_argument('--expect-old-time',action='store_true');a=p.parse_args()
src=Path(a.source);sys.path[:0]=[str(src/'navigation'),str(src/'experiments/arrival')]
from configure import config
from check import intrusions
out=Path(a.output);out.mkdir(parents=True,exist_ok=True);rows=[]
empty=np.zeros((160,200),dtype=int);closed=empty.copy();closed[:,60:63]=254;detour=empty.copy();detour[25:56,60:63]=254
cases=[('straight',empty,[0.,0.,0.],[1.,0.,0.],True),('pivot',empty,[0.,0.,0.],[0.,0.,1.],True),('detour',detour,[0.,0.,0.],[3.,0.,0.],True),('closed',closed,[0.,0.,0.],[3.,0.,0.],False)]
for name,cells,start,goal,expected in cases:
 prefix=out/name;mp=prefix.with_suffix('.map.txt');mp.write_text('200 160 .05 -2 -2\n'+' '.join(map(str,cells.ravel())))
 cfg=config(gait_profile='p2-flat-model')['planner_server']['ros__parameters']['GridBased'];cfg['max_planning_time']=1.;params=prefix.with_suffix('.yaml');params.write_text(yaml.safe_dump({'arrival_planner':{'ros__parameters':{'GridBased':cfg}}}))
 proc=subprocess.run([a.binary,str(mp),str(prefix),'p2_nav_controller::ExactHeadingLattice',*map(str,start),*map(str,goal),'--ros-args','--params-file',str(params),'--log-level','warn'],capture_output=True,text=True,timeout=18);prefix.with_suffix('.log').write_text(proc.stdout+proc.stderr);assert proc.returncode==0,proc.stderr[-1000:]
 row=json.loads(prefix.with_suffix('.json').read_text());assert row['planned']==expected,(name,row)
 if expected:
  path=[[float(v[k])for k in ('x','y','yaw')]for v in csv.DictReader(prefix.with_suffix('.csv').open())];row['filled_body_intrusions']=intrusions(cells,.05,(-2.,-2.),path);
  if not a.expect_old_time:assert row['filled_body_intrusions']==0,(name,row)
  row['new_result_time_valid']=-.01<=row['path_age_at_result_s']<=.5 and row['path_stamp_s']-row['input_goal_stamp_s']>=4.9
  if not a.expect_old_time:assert row['new_result_time_valid'],(name,row)
 row['case']=name;rows.append(row)
if a.expect_old_time:assert all(not row['new_result_time_valid']for row in rows[:2]),'Old short paths did not reproduce stale goal time'
result=dict(time_regression_passed=True,all_geometry_checks_passed=all(row.get('filled_body_intrusions',0)==0 for row in rows),passed=not a.expect_old_time,expected_old_header_bug=a.expect_old_time,source=str(src),hardware_commands=0,seed=0,footprint_total_side_m=.55,source_goal_age_s=5,cases=rows);(out/'report.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
