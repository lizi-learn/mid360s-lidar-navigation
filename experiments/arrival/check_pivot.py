#!/usr/bin/env python3
"""Replay observed low-input stall with actual pure response code, not a robot certification."""
from pathlib import Path
import argparse,csv,json,subprocess,math
p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);trace=out.with_suffix('.csv');subprocess.run([a.binary,str(trace)],check=True)
rows=[{k:float(v)for k,v in r.items()}for r in csv.DictReader(trace.open())];small=[r for r in rows if r['minimum']>0 and abs(r['goal'])<.5];old=[r for r in rows if r['minimum']==0 and abs(r['goal'])<.5];large=[r for r in rows if r['minimum']>0 and abs(r['goal'])>=.5]
assert len(small)==24 and all(r['reached'] and r['settled_error']<.15 for r in small)
unchanged=True
for r in large:
 b=next(b for b in rows if b['minimum']==0 and all(b[k]==r[k]for k in ('gain','delay','deadzone','goal')))
 # Keep the wider-delay limitation visible. This patch must not worsen it.
 unchanged=unchanged and r['settled_error']<=b['settled_error']+1e-8
assert unchanged
result={'legacy_method':'first-entry latch; stops issuing corrections forever after first entry into yaw tolerance, unlike actual controller; see check_pivot_native.py for settled actual-plugin/progress/collision/deactivation comparisons','seed':0,'scope':'Actual PivotBreakaway, terminal_velocity and native SpeedResponse headers; synthetic pure-yaw deadzone/first-order/delay plant. No physical robot, sensor freshness, balance or collision acceptance.','hardware_commands':0,'original_goal_tolerance_rad':.15,'small_angle_cases':len(small),'small_angle_all_reached_and_settled':True,'small_baseline_unreached':sum(not r['reached'] for r in old),'larger_angle_cases':len(large),'larger_angle_no_regression':unchanged,'larger_angle_still_failed_original_tolerance':sum(not r['reached']or r['settled_error']>=.15 for r in large),'complete_full_angle_acceptance':False,'cases':rows};out.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items()if k!='cases'}))
