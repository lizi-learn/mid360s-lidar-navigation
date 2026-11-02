"""Bounded actual-plugin synthetic comparison, never a physical acceptance test."""
import argparse,hashlib,json,os,subprocess,sys
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'navigation'))
from configure import config

def main():
 p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);p.add_argument('--baseline-results');a=p.parse_args()
 assert os.environ.get('ROS_LOCALHOST_ONLY')=='1' and int(os.environ.get('ROS_DOMAIN_ID','0'))>=200
 out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);rows=[];prior=json.loads(Path(a.baseline_results).read_text())['cases'] if a.baseline_results else []
 for gain,delay,cohort in [(1.2,.3,'nominal'),(.8,.5,'heldout')]:
  for kind in ('straight','gentle_s','corner','obstacle'):
   for name,lo,hi in [('before',.4,.8),('long_preview',1.2,1.6)]:
    cached=next((r for r in prior if name=='before' and r['cohort']==cohort and r['kind']==kind and r['profile']=='before'),None)
    if cached is not None:
     rows.append({**cached,'reused_prior_receipt':str(Path(a.baseline_results).resolve())});continue
    prefix=out.parent/f'{cohort}-{kind}-{name}'
    params=config(gait_profile='p2-flat-model',footprint_vertices=[[.275,.275],[.275,-.275],[-.275,-.275],[-.275,.275]],footprint_padding=0.)
    controller=params['controller_server']['ros__parameters'];controller['FollowPath'].update(min_lookahead_dist=lo,max_lookahead_dist=hi,settled_pivot_response=False,curved_forward_input=0.)
    # Isolate preview distance here. The combined settled-pivot profile is
    # covered by arrival/check_tracking.py and check_pivot_native.py.
    if name=='before':controller['FollowPath'].update(collision_lookahead_floor=0.,minimum_pivot_input=0.)
    paramfile=prefix.with_suffix('.params.yaml');paramfile.write_text(yaml.safe_dump({'preview_controller':{'ros__parameters':controller}}))
    cmd=[a.binary,str(prefix),kind,str(gain),str(delay),'--ros-args','--params-file',str(paramfile),'--log-level','error']
    result=subprocess.run(cmd,capture_output=True,text=True,timeout=45)
    prefix.with_suffix('.log').write_text(result.stdout+result.stderr)
    assert 'Do not call canTransform or lookupTransform with a timeout' not in result.stderr, 'Fixture timed TF API has no actual listener thread'
    assert len(result.stderr)<2_000_000,'Unexpected unbounded fixture error output'
    if result.returncode:
     row={'cohort':cohort,'kind':kind,'profile':name,'returncode':result.returncode,'error':result.stderr[-2000:]};rows.append(row)
     out.write_text(json.dumps({'complete':False,'cases':rows},indent=2));raise RuntimeError(row)
    row=json.loads(prefix.with_suffix('.json').read_text());actual=json.loads(prefix.with_suffix('.actual-parameters.json').read_text())
    assert actual['settled_pivot_response'] is False and actual['curved_forward_input']==0.,actual
    assert actual['min_lookahead_dist']==lo and actual['max_lookahead_dist']==hi,actual
    assert actual['rotate_to_heading_angular_vel']==1. and actual['regulated_linear_scaling_min_speed']==.6 and actual['use_collision_detection'] and not actual['allow_reversing'],actual
    assert len(actual['actual_footprint'])==4 and all(abs(abs(value)-.275)<1e-9 for point in actual['actual_footprint'] for value in point),actual
    row.update(profile=name,cohort=cohort,parameters=controller,actual_parameters=actual,receipt=prefix.name+'.json');rows.append(row)
    print(json.dumps({k:row[k] for k in ('cohort','kind','profile','goal_reached','body_intrusions','cross_track_rms_m','yaw_reversals','controller_collision_stops')}),flush=True)
    out.write_text(json.dumps({'complete':False,'cases':rows},indent=2))
 summary=[];ok=True
 for cohort in ('nominal','heldout'):
  for kind in ('straight','gentle_s','corner','obstacle'):
   b=next(r for r in rows if r['cohort']==cohort and r['kind']==kind and r['profile']=='before');c=next(r for r in rows if r['cohort']==cohort and r['kind']==kind and r['profile']=='long_preview')
   passed=c['goal_reached'] and c['goal_error_m']<.15 and c['goal_yaw_error_rad']<.15 and c['body_intrusions']==0
   if kind=='obstacle':passed=passed and c['obstacle_verified_in_costmap'] and c['controller_collision_stops']>0 and c['obstacle_cleared_after_two_seconds']
   if kind=='straight':passed=passed and c['cross_track_rms_m']<b['cross_track_rms_m'] and c['yaw_reversals']<b['yaw_reversals']
   summary.append({'cohort':cohort,'kind':kind,'candidate_case_passed':passed,'rms_before_m':b['cross_track_rms_m'],'rms_candidate_m':c['cross_track_rms_m'],'reversals_before':b['yaw_reversals'],'reversals_candidate':c['yaw_reversals'],'baseline_goal_reached':b['goal_reached'],'baseline_body_intrusions':b['body_intrusions']});ok=ok and passed
 result={'complete':True,'candidate_checks_passed':ok,'seed':0,'model_identity':'Planar forward/yaw dynamics + actual native SpeedResponse + explicit delay/first-order lag; no full gait/robot dynamics, localization, perception or planner. Actual own Nav2 RPP controller, synthetic costmap, unchanged55cm body and goal tolerances.','model_gain_delay_sweeps':[[1.2,.3],[.8,.5]],'control_code_sha256':hashlib.sha256((ROOT/'navigation/p2_nav_controller/src/settled_goal_rpp.cpp').read_bytes()).hexdigest(),'fixture_sha256':hashlib.sha256(Path(__file__).with_name('fixture.cpp').read_bytes()).hexdigest(),'hardware_commands':0,'full_body_check_interval_s':.01,'summary':summary,'cases':rows}
 out.write_text(json.dumps(result,indent=2));print(json.dumps({'complete':True,'candidate_checks_passed':ok,'cases':len(rows)}))
 if not ok:raise SystemExit(1)
if __name__=='__main__':main()
