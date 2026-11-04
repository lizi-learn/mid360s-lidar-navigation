#!/usr/bin/env python3
"""Actual planner reuse: small pose changes, new obstacle, opening and new goal."""
import argparse,csv,json,math,os,subprocess
from pathlib import Path
import numpy as np,yaml
from check import intrusions,config

def main():
 p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
 cells=np.zeros((180,240),dtype=int);r=.05;origin=(-2.,-3.)
 yy,xx=np.mgrid[:180,:240];wx=origin[0]+(xx+.5)*r;wy=origin[1]+(yy+.5)*r
 cells[(abs(wx-3.5)<=.075)&(wy>=2.)&(wy<=3.)]=254
 mp=out.parent/'map.txt';mp.write_text('240 180 .05 -2 -3\n'+' '.join(map(str,cells.ravel())))
 params=config(gait_profile='p2-flat-model')['planner_server']['ros__parameters']['PositionOnly'];pp=out.parent/'planner.yaml';pp.write_text(yaml.safe_dump({'arrival_planner':{'ros__parameters':{'GridBased':params}}}))
 prefix=out.parent/'sequence';env=dict(os.environ,P2_ARRIVAL_REPLAN_SEQUENCE='1')
 proc=subprocess.run([a.binary,str(mp),str(prefix),'p2_nav_controller::PositionOnlyLattice','2','2.5','0','5.2','2.5','0','--ros-args','--params-file',str(pp),'--log-level','warn'],env=env,capture_output=True,text=True,timeout=35);(out.parent/'planner.log').write_text(proc.stdout+proc.stderr)
 assert proc.returncode==0,proc.stderr[-3000:]
 paths={k:[[float(z[c])for c in ('x','y','yaw')]for z in csv.DictReader(Path(str(prefix)+'-'+k+'.csv').open())]for k in ('initial','perturbed','blocked','opened','newgoal')}
 def side(path):
  ys=[q[1]for q in path if abs(q[0]-3.5)<.1];assert ys
  assert min(ys)>3.275 or max(ys)<1.725,ys
  return -1 if sum(ys)/len(ys)<2.5 else 1
 initial=side(paths['initial']);assert side(paths['perturbed'])==initial,'same-goal detour flipped under small pose update'
 blocked=cells.copy();blocked[(wx>=3.1)&(wx<=4.3)&((wy<2.)if initial<0 else(wy>3.))]=254
 assert side(paths['blocked'])==-initial,'new obstacle did not invalidate old detour'
 rows=[]
 for k,path in paths.items():
  grid=blocked if k=='blocked'else np.zeros_like(cells)if k in ('opened','newgoal')else cells
  hits=intrusions(grid,r,origin,path);assert hits==0,(k,hits)
  end=[2.2,5.5]if k=='newgoal'else[5.2,2.5];assert math.dist(path[-1][:2],end)<.15
  rows.append(dict(case=k,points=len(path),body_intrusions=hits))
 assert max(abs(q[1]-2.48)for q in paths['opened'])<.04,'new direct shortcut not used'
 # In this60cm corridor, the original55cm body fits while an extra4cm
 # margin does not. Small source-pose changes must not discard the valid tail.
 narrow=np.full((80,200),254,dtype=int);yy,xx=np.mgrid[:80,:200];wy=(yy+.5)*r
 narrow[(wy>1.2)&(wy<1.8)]=0
 narrow_map=out.parent/'narrow-map.txt';narrow_map.write_text('200 80 .05 0 0\n'+' '.join(map(str,narrow.ravel())))
 narrow_prefix=out.parent/'narrow';narrow_env=dict(os.environ,P2_ARRIVAL_REUSE_NARROW='1')
 process=subprocess.run([a.binary,str(narrow_map),str(narrow_prefix),'p2_nav_controller::PositionOnlyLattice','1','1.5','0','7.5','1.5','0','--ros-args','--params-file',str(pp),'--log-level','info'],env=narrow_env,capture_output=True,text=True,timeout=25)
 (out.parent/'narrow.log').write_text(process.stdout+process.stderr);assert process.returncode==0,process.stderr[-3000:]
 reused=[[float(z[c])for c in ('x','y','yaw')]for z in csv.DictReader(Path(str(narrow_prefix)+'-reused.csv').open())]
 assert intrusions(narrow,r,(0.,0.),reused)==0,'Original body touched obstacle/unknown cells'
 assert math.dist(reused[-1][:2],[7.5,1.5])<.01
 blocked=json.loads(Path(str(narrow_prefix)+'-blocked.json').read_text());assert blocked['rejected']and blocked['marked_cells']>0
 receipts=(process.stdout+process.stderr).count('Reusing complete route with original footprint');assert receipts>0,'Original-body fallback was not exercised'
 rows.append(dict(case='same_body_narrow_reuse_and_new_blocked_goal',points=len(reused),body_intrusions=0,reuse_receipts=receipts,actual_obstacle_cells=blocked['marked_cells'],blocked_rejected=True))
 # Alternate intermediate targets on one actual planner instance. A second
 # route leg must not erase the first leg's still-valid detour; a real new
 # obstacle must nevertheless invalidate it.
 yy,xx=np.mgrid[:cells.shape[0],:cells.shape[1]];wx=origin[0]+(xx+.5)*r;wy=origin[1]+(yy+.5)*r
 alt=out.parent/'alternating';alt_env=dict(os.environ,P2_ARRIVAL_ALTERNATING_SEQUENCE='1')
 proc=subprocess.run([a.binary,str(mp),str(alt),'p2_nav_controller::PositionOnlyLattice','2','2.5','0','5.2','2.5','0','--ros-args','--params-file',str(pp),'--log-level','warn'],env=alt_env,capture_output=True,text=True,timeout=35)
 (out.parent/'alternating.log').write_text(proc.stdout+proc.stderr);assert proc.returncode==0,proc.stderr[-3000:]
 alt_paths={k:[[float(z[c])for c in ('x','y','yaw')]for z in csv.DictReader(Path(str(alt)+'-alternating-'+k+'.csv').open())]for k in ('initial','other','reused','blocked')}
 assert side(alt_paths['initial'])==side(alt_paths['reused']),'valid detour changed side after another route leg'
 assert side(alt_paths['blocked'])==-side(alt_paths['initial']),'new actual obstacle failed to invalidate per-leg memory'
 for key,path in alt_paths.items():
  grid=cells.copy()
  if key in ('initial','blocked'):grid[(wx>=3.1)&(wx<=4.3)&((wy<2.)if key=='initial'else(wy>3.))]=254
  hits=intrusions(grid,r,origin,path);assert hits==0,(key,hits)
  rows.append(dict(case='alternating_'+key,points=len(path),body_intrusions=hits))
 result=dict(passed=True,cases=rows,small_pose_update_keeps_side=True,new_obstacle_changes_side=True,new_opening_uses_direct=True,new_goal_invalidates_route=True,hardware_commands=0,fixture_seed=0,scope='Actual own Nav2 planner, same-goal synthetic replans; no perception, control tracking or physical walking acceptance')
 out.write_text(json.dumps(result,indent=2));print(json.dumps(result))
if __name__=='__main__':main()
