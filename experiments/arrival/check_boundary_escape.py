"""Native full-body boundary escape; synthetic fixtures only, no hardware I/O."""
from pathlib import Path
import argparse,json,math,os,subprocess,sys,yaml,copy
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'navigation'))
from configure import config
p=argparse.ArgumentParser();p.add_argument('--binary-dir',required=True);p.add_argument('--output',required=True);a=p.parse_args()
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1' and 200<=int(os.environ['ROS_DOMAIN_ID'])<=232
b=Path(a.binary_dir);out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
base=config(gait_profile='p2-flat-model');params=base['controller_server']['ros__parameters'];planner=base['planner_server']['ros__parameters']['PositionOnly']
w=h=100;r=.05;ox=oy=-1.02;cells=[0]*(w*h);ix=int((-.24-ox)/r);iy=int((-.295-oy)/r);cells[iy*w+ix]=254
mf=out/'synthetic.map';mf.write_text(f'{w} {h} {r} {ox} {oy}\n'+' '.join(map(str,cells)))
geometry=out/'synthetic.geometry';geometry.write_text(f'{w} {h} {r} {ox} {oy} 0 0 0\n'+' '.join(map(str,cells)))
unit=subprocess.run([str(b/'boundary_escape'),str(geometry)],capture_output=True,text=True,timeout=10);(out/'geometry.log').write_text(unit.stdout+unit.stderr);assert unit.returncode==0,unit.stderr
rows=[]
for name,enabled,obstacle in [('old',False,False),('fixed',True,False),('fixed_obstacle',True,True),('fixed_pose_ripple',True,False)]:
 prefix=out/name
 pp=dict(params);pp['FollowPath']=dict(params['FollowPath'],boundary_forward_escape=enabled)
 pp['GridBased']=dict(planner,boundary_forward_escape=enabled)
 pf=prefix.with_suffix('.yaml');pf.write_text(yaml.safe_dump({'preview_controller':{'ros__parameters':pp},'arrival_planner':{'ros__parameters':{'GridBased':copy.deepcopy(pp['GridBased'])}}}))
 cmd=[str(b/'arrival_planner'),str(mf),str(prefix)+'-plan','p2_nav_controller::PositionOnlyLattice','0','0','0','.6','0','0','--ros-args','--params-file',str(pf),'--log-level','error']
 plan=subprocess.run(cmd,capture_output=True,text=True,timeout=30);(out/(name+'-planner.log')).write_text(plan.stdout+plan.stderr);assert plan.returncode==0,plan.stderr
 pr=json.loads(Path(str(prefix)+'-plan.json').read_text());assert pr['planned']==enabled,pr
 path=Path(str(prefix)+'-plan.csv')if enabled else out/'old-reference.csv'
 if not enabled:path.write_text('x,y,yaw\n0,0,0\n.6,0,0\n')
 env=dict(os.environ,P2_TEST_BOUNDARY_ESCAPE='1',P2_TEST_REPLAY_MAP=str(mf),P2_TEST_REPLAY_CSV=str(path),P2_TEST_FIXED_REFERENCE='1',P2_TEST_POSITION_GOAL='1')
 if obstacle:env['P2_TEST_BOUNDARY_OBSTACLE']='1'
 if name=='fixed_pose_ripple':env.update(P2_TEST_POSE_XY_NOISE='.003',P2_TEST_POSE_YAW_NOISE='.001')
 cmd=[str(b/'tracking_replay'),str(prefix),'straight','0.7','.3','--ros-args','--params-file',str(pf),'--log-level','error']
 run=subprocess.run(cmd,capture_output=True,text=True,env=env,timeout=50);prefix.with_suffix('.log').write_text(run.stdout+run.stderr);assert run.returncode==0,run.stderr
 result=json.loads(prefix.with_suffix('.json').read_text());actual=json.loads(Path(str(prefix)+'.actual-parameters.json').read_text());assert actual['boundary_forward_escape']==enabled and actual['use_collision_detection']
 passed=result['goal_reached']==enabled and result['body_intrusions']==0
 if obstacle:passed=passed and result['obstacle_verified_in_costmap'] and result['controller_collision_stops']>0 and result['obstacle_cleared_after_two_seconds']
 rows.append(dict(name=name,passed=passed,result=result,actual=actual,planner=pr,parameters=pp,command=cmd));print(json.dumps(dict(name=name,passed=passed,result=result)),flush=True)
 report=dict(complete=False,cases=rows);(out/'report.json').write_text(json.dumps(report,indent=2))
report=dict(complete=True,passed=all(v['passed']for v in rows),cases=rows,seed=0,geometry=json.loads(unit.stdout),scope='Actual native planner/controller/full55cm polygon; existing edge overlap distinguished from NEW occupied sweep; synthetic planar latency, not physical gait verification.',hardware_commands=0)
(out/'report.json').write_text(json.dumps(report,indent=2));assert report['passed']
