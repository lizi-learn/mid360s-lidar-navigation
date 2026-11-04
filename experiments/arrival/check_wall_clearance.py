"""Native planner: prefer room in open space, retain real narrow passages."""
from pathlib import Path
import argparse,csv,json,os,subprocess,sys
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'navigation'))
from configure import config
from check import intrusions

p=argparse.ArgumentParser()
p.add_argument('--binary',required=True)
p.add_argument('--output',required=True)
p.add_argument('--baseline',action='store_true')
a=p.parse_args()
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1' and 200<=int(os.environ['ROS_DOMAIN_ID'])<=232
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
params=config(gait_profile='p2-flat-model')['planner_server']['ros__parameters']['PositionOnly']
pf=out/'params.yaml';pf.write_text(yaml.safe_dump({'arrival_planner':{'ros__parameters':{'GridBased':params}}}))
w,h,res=160,120,.05
reports=[]
for side in [-1,1]:
    for narrow in [False,True]:
        name=('lower' if side<0 else 'upper')+('-narrow' if narrow else '-open')
        cells=np.zeros((h,w),dtype=np.uint8)
        cells[0,:]=cells[-1,:]=cells[:,0]=cells[:,-1]=254
        marked=0
        for iy in range(h):
            for ix in range(w):
                x,y=(ix+.5)*res,(iy+.5)*res
                relative=side*(y-2.5)
                if 2.5<=x<=4. and (relative>=.4 or (narrow and relative<=-.4)):
                    cells[iy,ix]=254;marked+=1
        scene=out/(name+'.map')
        scene.write_text(f'{w} {h} {res} 0 0\n'+' '.join(map(str,cells.ravel())))
        prefix=out/name
        run=subprocess.run([a.binary,str(scene),str(prefix),'p2_nav_controller::PositionOnlyLattice',
            '1','2.5','0','6','2.5','0','--ros-args','--params-file',str(pf),'--log-level','error'],
            capture_output=True,text=True,timeout=25)
        prefix.with_suffix('.log').write_text(run.stdout+run.stderr)
        result=json.loads(prefix.with_suffix('.json').read_text())
        assert run.returncode==0 and result['planned'],result
        path=[[float(r[k]) for k in ['x','y','yaw']] for r in csv.DictReader(prefix.with_suffix('.csv').open())]
        bad=intrusions(cells,res,(0.,0.),path)
        # A measured map distance, not a replacement footprint or admission gate.
        centre_gap=min(.4-side*(y-2.5) for x,y,yaw in path if 2.75<=x<=3.75)
        reports.append(dict(case=name,marked_obstacle_cells=marked,body_intrusions=bad,
            centre_gap_along_obstacle=centre_gap,seconds=result['seconds'],narrow=narrow))
        assert marked and bad==0
        if not narrow and not a.baseline:
            assert centre_gap>=.54,(name,centre_gap)
passed=all(r['body_intrusions']==0 for r in reports)
report=dict(passed=passed,cases=reports,seed=0,hardware_commands=0,footprint_total_side=.55,
    scope='Actual native planner on mirrored synthetic maps. Narrow gaps remain usable; no physical walking claim.')
(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
