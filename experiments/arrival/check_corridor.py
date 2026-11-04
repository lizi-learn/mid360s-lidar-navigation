#!/usr/bin/env python3
"""Actual native corridor-area planning; exact visits and full-body constraints."""
import argparse,csv,json,math,os,subprocess
from pathlib import Path
import numpy as np,yaml
from check import intrusions,config

def main():
    p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
    r=.05;origin=(-2.,-3.);cells=np.zeros((180,240),dtype=int)
    yy,xx=np.mgrid[:180,:240];wx=origin[0]+(xx+.5)*r;wy=origin[1]+(yy+.5)*r
    ref=[3.5,1.];cells[(abs(wx-ref[0])<.04)&(abs(wy-ref[1])<.04)]=254
    mp=out.parent/'map.txt';mp.write_text('240 180 .05 -2 -3\n'+' '.join(map(str,cells.ravel())))
    base=config(gait_profile='p2-flat-model')['planner_server']['ros__parameters']
    cases=[]
    def run(name,plugin,goal=ref,final=(7.5,1.),directed=False,extra=None,grid=None):
        params=dict(base['GuideDirected' if directed else 'GuidePosition'])
        params.update(final_x=float(final[0]),final_y=float(final[1]),final_request_ns=1)
        pp=out.parent/(name+'.yaml');pp.write_text(yaml.safe_dump({'arrival_planner':{'ros__parameters':{'GridBased':params}}}))
        pathmap=mp
        if grid is not None:
            pathmap=out.parent/(name+'-map.txt');pathmap.write_text('240 180 .05 -2 -3\n'+' '.join(map(str,grid.ravel())))
        prefix=out.parent/name
        env=dict(os.environ,**(extra or {}))
        if plugin.endswith('CorridorRouteLattice'):env['P2_ARRIVAL_FINAL_REQUEST']='1'
        else:env.pop('P2_ARRIVAL_FINAL_REQUEST',None)
        proc=subprocess.run([a.binary,str(pathmap),str(prefix),plugin,'0','1','0',str(goal[0]),str(goal[1]),'.4','--ros-args','--params-file',str(pp),'--log-level','warn'],env=env,capture_output=True,text=True,timeout=35)
        (out.parent/(name+'.log')).write_text(proc.stdout+proc.stderr)
        assert proc.returncode==0,(name,proc.stderr[-3000:])
        result=json.loads(Path(str(prefix)+'.json').read_text())
        return result,prefix
    old,_=run('old-exact-reference','p2_nav_controller::PositionOnlyLattice')
    assert not old['planned'],'Fixture did not reproduce the occupied reference failure'
    current,prefix=run('corridor-reference','p2_nav_controller::CorridorRouteLattice',extra={'P2_ARRIVAL_CORRIDOR_SEQUENCE':'1'})
    assert current['planned'],current
    receipts=json.loads(Path(str(prefix)+'-area-receipts.json').read_text())
    for suffix in ['', '-area-initial','-area-other','-area-reused','-area-new-obstacle']:
        path=[[float(q[k])for k in ('x','y','yaw')]for q in csv.DictReader(Path(str(prefix)+suffix+'.csv').open())]
        grid=cells.copy()
        if suffix=='-area-new-obstacle':
            x,y=receipts['actual_new_obstacle'];grid[int((y-origin[1])/r),int((x-origin[0])/r)]=254
        hits=intrusions(grid,r,origin,path);assert hits==0,(suffix,hits)
        target=[1.,3.]if suffix=='-area-other'else ref
        assert math.dist(path[-1][:2],target)<=.65,(suffix,path[-1],target)
        cases.append(dict(case=suffix or 'occupied_reference_area',points=len(path),body_intrusions=hits))
    assert receipts['blocked_area_rejected']and receipts['occupied_final_rejected']
    for directed in [False,True]:
        q,prefix=run('exact-final-'+str(directed),'p2_nav_controller::CorridorRouteLattice',goal=[7.5,1.],directed=directed)
        assert q['planned'],q
        path=[[float(z[k])for k in ('x','y','yaw')]for z in csv.DictReader(Path(str(prefix)+'.csv').open())]
        assert intrusions(cells,r,origin,path)==0
        assert math.dist(path[-1][:2],[7.5,1.])<.1
        if directed:assert abs(math.remainder(path[-1][2]-.4,2*math.pi))<.02
        cases.append(dict(case='exact_directed_final'if directed else'exact_position_final',points=len(path),body_intrusions=0))
    split=np.zeros_like(cells);split[abs(wx-3.5)<.075]=254
    q,prefix=run('nearest-free-disconnected','p2_nav_controller::CorridorRouteLattice',goal=[3.6,1.],final=[1.,4.],grid=split)
    assert q['planned'],q
    path=[[float(z[k])for k in ('x','y','yaw')]for z in csv.DictReader(Path(str(prefix)+'.csv').open())]
    assert intrusions(split,r,origin,path)==0
    assert path[-1][0]<3.3 and math.dist(path[-1][:2],[3.6,1.])<=.65,'Selected the unreachable side of a wall'
    cases.append(dict(case='nearest_free_other_component_skipped',points=len(path),body_intrusions=0))
    unknown=cells.copy();unknown[(abs(wx-ref[0])<1.)&(abs(wy-ref[1])<1.)]=255
    q,_=run('unknown-reference-area','p2_nav_controller::CorridorRouteLattice',grid=unknown)
    assert not q['planned'],'Unknown area accepted'
    result=dict(passed=True,cases=cases,blocked_area_rejected=True,unknown_area_rejected=True,occupied_final_rejected=True,exact_final_heading_preserved=True,original_exact_reference_failed=True,reference_radius=.6,footprint_total_side=.55,new_obstacle_reselected=True,alternating_leg_selection_stable=True,fixture_seed=0,hardware_commands=0,scope='Actual Nav2 planner plugin on synthetic live costmaps; not physical tracking acceptance')
    out.write_text(json.dumps(result,indent=2));print(json.dumps(result))
if __name__=='__main__':main()
