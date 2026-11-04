#!/usr/bin/env python3
"""Actual lattice plugins with full-square replay checks; synthetic public inputs only by default."""
from pathlib import Path
import argparse,csv,json,math,subprocess,sys,hashlib
import numpy as np,yaml
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'navigation'))
from configure import config

def intrusions(cells,r,origin,path):
    h,w=cells.shape;ox,oy=origin;total=0
    if not len(path):return 1
    for a,b in zip([path[0],*path],path):
        angle=math.remainder(b[2]-a[2],2*math.pi);n=max(1,math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])/.01),math.ceil(abs(angle)/.01))
        for k in range(n+1):
            f=k/n;x=a[0]+f*(b[0]-a[0]);y=a[1]+f*(b[1]-a[1]);yaw=a[2]+f*angle;c,s=math.cos(yaw),math.sin(yaw)
            lx,ux=math.floor((x-.43-ox)/r),math.floor((x+.43-ox)/r);ly,uy=math.floor((y-.43-oy)/r),math.floor((y+.43-oy)/r)
            if lx<0 or ly<0 or ux>=w or uy>=h:total+=1;continue
            yy,xx=np.where(cells[ly:uy+1,lx:ux+1]>=254);dx=ox+(xx+lx+.5)*r-x;dy=oy+(yy+ly+.5)*r-y
            hit=(np.abs(dx)<=.275*(abs(c)+abs(s))+r/2)&(np.abs(dy)<=.275*(abs(c)+abs(s))+r/2)&(np.abs(c*dx+s*dy)<=.275+r/2*(abs(c)+abs(s)))&(np.abs(-s*dx+c*dy)<=.275+r/2*(abs(c)+abs(s)))
            if np.any(hit):total+=1
    return total

def main():
    p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);p.add_argument('--private-replays');a=p.parse_args();out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
    r=.05;origin=(-2.,-3.);cells=np.zeros((180,240),dtype=int);cells[100:,:]=254
    cases=[dict(name='wall_adjacent_position',cells=cells,r=r,origin=origin,start=[0.,0.,0.],goal=[6.025,1.675,math.pi/4],expected=True)]
    corridor=np.full((160,220),254);corridor[50:75,15:170]=0;corridor[50:145,145:170]=0
    cases.append(dict(name='reverse_facing_corner',cells=corridor,r=r,origin=(-2.,-3.),start=[0.,0.,math.pi],goal=[5.85,3.,math.pi/2],expected=True))
    blocked=cells.copy();blocked[:,90:94]=254
    cases.append(dict(name='closed_wall',cells=blocked,r=r,origin=origin,start=[0.,0.,0.],goal=[6.025,1.675,math.pi/4],expected=False))
    inside=cells.copy();inside[60,41]=254
    cases.append(dict(name='interior_start_obstacle',cells=inside,r=r,origin=origin,start=[0.,0.,0.],goal=[6.025,1.675,math.pi/4],expected=False))
    pivot=np.full((100,140),254);pivot[30:50,10:130]=0
    cases.append(dict(name='directed_in_place_start',cells=pivot,r=r,origin=(0.,0.),start=[1.,2.,math.pi],goal=[5.,2.,0.],expected=True,check_directed=True,require_initial_pivot=True))
    no_pivot=np.full((100,140),254);no_pivot[36:49,10:130]=0
    cases.append(dict(name='directed_pivot_does_not_fit',cells=no_pivot,r=r,origin=(0.,0.),start=[1.,2.125,math.pi],goal=[5.,2.125,0.],expected=False,check_directed=True))
    if a.private_replays:
        for item in json.loads(Path(a.private_replays).read_text()):
            d=json.loads(Path(item['snapshot']).read_text());m=d['maps']['global'];cases.append(dict(name=item['name'],cells=np.array(m['cells']).reshape(m['height'],m['width']),r=m['resolution'],origin=m['origin'],start=item.get('start',d['pose']),goal=item['goal'],expected=item.get('expected',True),snapshot_sha256=hashlib.sha256(Path(item['snapshot']).read_bytes()).hexdigest(),private=True))
    rows=[];ok=True
    for case in cases:
        name=case['name'];grid=case['cells'];h,w=grid.shape;mp=out.parent/(name+'.map.txt');mp.write_text(f"{w} {h} {case['r']} {case['origin'][0]} {case['origin'][1]}\n"+' '.join(map(str,grid.ravel())))
        for plugin in ('ExactHeadingLattice','PositionOnlyLattice'):
            prefix=out.parent/(name+'-'+plugin);params=config(gait_profile='p2-flat-model')['planner_server']['ros__parameters']['PositionOnly' if plugin=='PositionOnlyLattice' else 'GridBased'];params.update(plugin='p2_nav_controller::'+plugin)
            pp=prefix.with_suffix('.params.yaml');pp.write_text(yaml.safe_dump({'arrival_planner':{'ros__parameters':{'GridBased':params}}}))
            cmd=[a.binary,str(mp),str(prefix),'p2_nav_controller::'+plugin,*map(str,case['start']),*map(str,case['goal']),'--ros-args','--params-file',str(pp),'--log-level','warn']
            proc=subprocess.run(cmd,capture_output=True,text=True,timeout=18);prefix.with_suffix('.log').write_text(proc.stdout+proc.stderr)
            row=json.loads(prefix.with_suffix('.json').read_text()) if proc.returncode==0 else {'planned':False,'fixture_error':proc.stderr[-1500:],'returncode':proc.returncode}
            actual=json.loads(prefix.with_suffix('.actual.json').read_text()) if proc.returncode==0 else {}
            if actual:
                assert not actual['radius'] and len(actual['footprint'])==4 and all(abs(abs(v)-.275)<1e-9 for q in actual['footprint'] for v in q)
                assert actual['tolerance']==.05
            row.update(actual_parameters=actual,case=name,plugin=plugin,expected_position_path=case['expected'],private=case.get('private',False),snapshot_sha256=case.get('snapshot_sha256'))
            if row['planned']:
                path=[[float(z[k])for k in ('x','y','yaw')]for z in csv.DictReader(prefix.with_suffix('.csv').open())];row['body_intrusions']=intrusions(grid,case['r'],case['origin'],path);row['endpoint_error_m']=math.hypot(path[-1][0]-case['goal'][0],path[-1][1]-case['goal'][1]);row['path_length_m']=sum(math.dist(x[:2],y[:2])for x,y in zip(path,path[1:]));row['full55cm_square_checked_every_m_rad']=.01
                if plugin=='PositionOnlyLattice':
                    assert row['path_stamp_s']-row['input_goal_stamp_s']>=4.9,'output reused old goal request time'
                    assert -.01<=row['path_age_at_result_s']<=.5,'newly computed path source stamp invalid'
                if case.get('require_initial_pivot'):
                    prefix=[]
                    for pose in path:
                        if math.dist(pose[:2],case['start'][:2])>1e-6:break
                        prefix.append(pose)
                    row['initial_pivot_translation_m']=max((math.dist(q[:2],case['start'][:2])for q in prefix),default=None)
                    row['initial_pivot_angle_rad']=max((abs(math.remainder(q[2]-case['start'][2],2*math.pi))for q in prefix),default=0.)
                    assert row['initial_pivot_angle_rad']>math.pi/2,'reverse-facing route omitted its actual zero-translation pivot'
                if plugin=='ExactHeadingLattice':
                    row['arrival_yaw_error_rad']=abs(math.remainder(path[-1][2]-case['goal'][2],2*math.pi))
                    assert row['arrival_yaw_error_rad']<.02,'directed fallback lost the requested arrival heading'
            if plugin=='PositionOnlyLattice' or case.get('check_directed'):
                row['passed']=row['planned']==case['expected'] and (not row['planned']or row['body_intrusions']==0 and row['endpoint_error_m']<=.15) and proc.returncode==0;ok=ok and row['passed']
            rows.append(row);out.write_text(json.dumps({'complete':False,'cases':rows},indent=2));print(json.dumps(row),flush=True)
    out.write_text(json.dumps({'complete':True,'candidate_checks_passed':ok,'seed':0,'hardware_commands':0,'identity':'Actual own Humble lattice plugins; full55cm square with zero padding; original unknown and actual occupied cells retained; synthetic grids or explicitly private frozen costmap replay. Planning verification only, not perception or biped dynamics.','cases':rows},indent=2))
    if not ok:raise SystemExit(1)
if __name__=='__main__':main()
