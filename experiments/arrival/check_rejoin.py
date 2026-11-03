#!/usr/bin/env python3
"""Full-size body regression for the short sideways rejoin and observed block.

Generated corridor only; no recorded environment, motor client or sensor stamp.
"""
from pathlib import Path
import argparse, json, math, subprocess
p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=True);rows=[]
w=h=180;r=.05;ox=oy=-2.
route=out/'route.csv';route.write_text('x,y,yaw\n'+''.join(f'0,{i*.02},{math.pi/2}\n'for i in range(301)))
def run(label,start,variant,block=None):
    cells=[254 if ox+(i%w+.5)*r<-.7 else 0 for i in range(w*h)]
    marked=0
    if block is not None:
        for i in range(len(cells)):
            x=ox+(i%w+.5)*r;y=oy+(i//w+.5)*r
            if abs(x)<.15 and abs(y-3.)<.15:cells[i]=block;marked+=1
    mapfile=out/(label+'.map');mapfile.write_text(f'{w} {h} {r} {ox} {oy}\n'+' '.join(map(str,cells)))
    proc=subprocess.run([a.binary,str(mapfile),str(route),*map(str,start),variant,str(out/(label+'.csv'))],capture_output=True,text=True,timeout=15)
    assert proc.returncode==0,proc.stderr
    row=json.loads(proc.stdout);row.update(case=label,variant=variant,marked_obstacle_cells=marked)
    rows.append(row);return row
for i,offset in enumerate([.32,.4,.46]):
    start=[offset,.2,1.1]
    old=run(f'offset{i}-old',start,'legacy');new=run(f'offset{i}-new',start,'candidate')
    assert old['planned']and new['planned']and old['full_sweep_clear']and new['full_sweep_clear']
    assert abs(new['first_pivot_rad'])<abs(old['first_pivot_rad'])-.3
    assert new['two_metre_absolute_turn']<old['two_metre_absolute_turn']-.5
for cost,name in[(254,'occupied'),(255,'unknown')]:
    blocked=run(name,[.4,.2,1.1],'candidate',cost)
    assert blocked['marked_obstacle_cells']>0 and not blocked['planned']
report=dict(passed=True,seed=0,full_body_side_m=.55,padding_m=0.,hardware_commands=0,cases=rows,scope='Actual route-rejoin helper and filled-polygon sweep, generated corridor. No physical motion or dynamic balance claim.')
(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
