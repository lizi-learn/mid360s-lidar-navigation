"""Actual RPP/goal checker with synthetic half-input deadzone and obstacle."""
from pathlib import Path
import argparse
import copy
import json
import os
import subprocess
import sys
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'navigation'))
from configure import config

p = argparse.ArgumentParser()
p.add_argument('--binary', required=True)
p.add_argument('--output', required=True)
a = p.parse_args()
out = Path(a.output)
out.mkdir(parents=True, exist_ok=True)
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1' and 200 <= int(os.environ['ROS_DOMAIN_ID']) <= 232
short = out / 'short.csv'
short.write_text('x,y,yaw\n0,0,0\n0.49,0,0\n')
rows = []
for name, enabled, scale, obstacle in [('old_half', False, .5, False),
                                      ('fixed_half', True, .5, False),
                                      ('fixed_full', True, 1., False),
                                      ('half_obstacle', True, .5, True)]:
    params = copy.deepcopy(config(gait_profile='p2-flat-model')['controller_server']['ros__parameters'])
    params['FollowPath']['terminal_input_response'] = enabled
    params['goal_checker'].update(xy_goal_tolerance=.4, yaw_goal_tolerance=.3490658503988659)
    prefix = out / name
    pf = prefix.with_suffix('.params.yaml')
    pf.write_text(yaml.safe_dump({'preview_controller': {'ros__parameters': params}}))
    env = dict(os.environ, P2_TEST_INPUT_SCALE=str(scale), P2_TEST_X_DEADZONE='.25')
    if not obstacle:
        env.update(P2_TEST_REPLAY_CSV=str(short), P2_TEST_FIXED_REFERENCE='1')
    command = [a.binary, str(prefix), 'obstacle' if obstacle else 'straight', '1.', '.3',
               '--ros-args', '--params-file', str(pf), '--log-level', 'error']
    run = subprocess.run(command, env=env, capture_output=True, text=True, timeout=50)
    prefix.with_suffix('.log').write_text(run.stdout + run.stderr)
    assert run.returncode == 0, run.stderr[-2000:]
    result = json.loads(prefix.with_suffix('.json').read_text())
    actual = json.loads(prefix.with_suffix('.actual-parameters.json').read_text())
    assert actual['terminal_input_response'] == enabled and actual['use_collision_detection']
    assert all(abs(abs(v) - .275) < 1e-8 for pt in actual['actual_footprint'] for v in pt)
    passed = result['goal_reached'] == enabled and result['body_intrusions'] == 0
    if enabled:
        passed = passed and result['goal_error_m'] <= .4
    if obstacle:
        passed = passed and result['obstacle_verified_in_costmap'] and result['controller_collision_stops'] > 0 and result['obstacle_cleared_after_two_seconds']
    rows.append(dict(name=name, passed=passed, result=result, actual=actual, scale=scale,
                     forward_deadzone=.25, command=command, parameters=params))
    (out / 'report.json').write_text(json.dumps({'complete': False, 'cases': rows}, indent=2))
    print(json.dumps(dict(name=name, passed=passed, reached=result['goal_reached'], duration=result['duration_s'],
                         error=result['goal_error_m'], intrusions=result['body_intrusions'])), flush=True)
report = dict(complete=True, passed=all(r['passed'] for r in rows), cases=rows, seed=0,
              hardware_commands=0, scope='Actual native plugin/goal checker/full55cm footprint with synthetic input deadzone and delayed planar response; not physical gait verification.')
(out / 'report.json').write_text(json.dumps(report, indent=2))
assert report['passed']
