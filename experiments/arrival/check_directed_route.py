"""Native transit vs final-heading semantics, including repeated coordinates."""
import argparse,csv,json,math,os,subprocess,sys
from pathlib import Path
import numpy as np,yaml
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'navigation'),str(Path(__file__).parent)]
from configure import config
from check import intrusions
p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
cells=np.zeros((100,140),dtype=np.uint8);mp=out/'synthetic.map.txt';mp.write_text('140 100 .05 -2 -2\n'+' '.join(map(str,cells.ravel())))
cfg=config(gait_profile='p2-flat-model')['planner_server']['ros__parameters']['RouteDirected'];cfg.update(final_x=1.,final_y=0.)
params=out/'params.yaml';params.write_text(yaml.safe_dump({'arrival_planner':{'ros__parameters':{'GridBased':cfg}}}))
rows=[]
for label,final in [('transit_same_coordinates_as_final',False),('actual_final_heading',True)]:
 env=dict(os.environ);env.pop('P2_ARRIVAL_FINAL_REQUEST',None)
 if final:env['P2_ARRIVAL_FINAL_REQUEST']='1'
 prefix=out/label;cmd=[a.binary,str(mp),str(prefix),'p2_nav_controller::RouteDirectedLattice','0','0','0','1','0',str(math.pi/2),'--ros-args','--params-file',str(params),'--log-level','warn']
 proc=subprocess.run(cmd,capture_output=True,text=True,timeout=20,env=env);prefix.with_suffix('.log').write_text(proc.stdout+proc.stderr);assert proc.returncode==0,proc.stderr
 row=json.loads(prefix.with_suffix('.json').read_text());assert row['planned'],row
 path=[[float(r[k])for k in ['x','y','yaw']]for r in csv.DictReader(prefix.with_suffix('.csv').open())]
 row.update(case=label,body_intrusions=intrusions(cells,.05,(-2.,-2.),path),final_yaw=path[-1][2],request_is_final=final)
 assert row['body_intrusions']==0
 expected=math.pi/2 if final else 0.;assert abs(math.remainder(path[-1][2]-expected,2*math.pi))<.02,row
 rows.append(row)
report=dict(passed=True,cases=rows,hardware_commands=0,scope='Actual RouteDirectedLattice and native parameters on synthetic55cm planar grid. Request header unchanged; same XY alone does not request a final turn.')
(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
