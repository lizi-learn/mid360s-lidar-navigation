"""Original full footprint under small physical sway and sustained drift."""
from pathlib import Path
import argparse,json,os,subprocess,sys
import yaml

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'navigation'))
from configure import config
p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True)
a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1' and 200<=int(os.environ['ROS_DOMAIN_ID'])<=232
params=config(gait_profile='p2-flat-model')['controller_server']['ros__parameters']
pf=out/'params.yaml';pf.write_text(yaml.safe_dump({'preview_controller':{'ros__parameters':params}}))
reference=out/'reference.csv';reference.write_text('x,y,yaw\n'+'\n'.join(f'{1+i*.02},2.5,0' for i in range(351)))
rows=[];width,height,res=200,120,.05
for side in [-1,1]:
    cells=[254 if x in (0,width-1) or y in (0,height-1) or
           (3<=(x+.5)*res<=5 and side*((y+.5)*res-2.5)>=.5) else 0
           for y in range(height) for x in range(width)]
    scene=out/(str(side)+'.map');scene.write_text(f'{width} {height} {res} 0 0\n'+' '.join(map(str,cells)))
    for drift in [0.,side*.12]:
        prefix=out/(str(side)+'-'+str(drift))
        env=dict(os.environ,P2_TEST_REPLAY_MAP=str(scene),P2_TEST_REPLAY_CSV=str(reference),
            P2_TEST_POSITION_GOAL='1',P2_TEST_FIXED_REFERENCE='1',P2_TEST_LATERAL_RATIO=str(drift),
            P2_TEST_LATERAL_RIPPLE='.25',P2_TEST_OBSTACLE_PROBE=f'4 {2.5+side*.6}')
        run=subprocess.run([a.binary,str(prefix),'ripple','.8','.5','--ros-args','--params-file',str(pf),
            '--log-level','error'],capture_output=True,text=True,env=env,timeout=35)
        Path(str(prefix)+'.log').write_text(run.stdout+run.stderr)
        assert run.returncode==0,run.stderr[-2000:]
        row=json.loads(Path(str(prefix)+'.json').read_text());row['side']=side
        actual=json.loads(Path(str(prefix)+'.actual-parameters.json').read_text())
        assert all(abs(abs(v)-.275)<1e-8 for q in actual['actual_footprint'] for v in q),actual
        assert row['loaded_controller_library'],row
        row['passed']=row['goal_reached'] and row['body_intrusions']==0 and row['static_obstacle_probe_verified'] and row['commanded_y']==0
        rows.append(row);print(json.dumps(row),flush=True)
report=dict(passed=all(r['passed'] for r in rows),seed=0,hardware_commands=0,cases=rows,
    scope='Actual controller and source-identified native library; synthetic2Hz physical lateral ripple plus bias. Not calibrated biped dynamics.')
(out/'report.json').write_text(json.dumps(report,indent=2));assert report['passed']
