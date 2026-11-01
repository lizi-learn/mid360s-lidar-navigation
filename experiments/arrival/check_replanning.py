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
 result=dict(passed=True,cases=rows,small_pose_update_keeps_side=True,new_obstacle_changes_side=True,new_opening_uses_direct=True,new_goal_invalidates_route=True,hardware_commands=0,fixture_seed=0,scope='Actual own Nav2 planner, same-goal synthetic replans; no perception, control tracking or physical walking acceptance')
 out.write_text(json.dumps(result,indent=2));print(json.dumps(result))
if __name__=='__main__':main()
