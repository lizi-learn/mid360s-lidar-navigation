"""Generate an explicitly experimental planning profile; starts no processes."""
from pathlib import Path
import argparse
import json
import math
import sys
import yaml

EXPERIMENT=Path(__file__).resolve().parent
sys.path.insert(0,str(EXPERIMENT.parent.parent/'navigation'))
from configure import config as baseline_config

def parameters(frozen_replay=False,headings=32):
    result=baseline_config(gait_profile='p2-flat-model')
    if headings!=32:raise ValueError('Gait-space experiment requires 32 forward headings')
    grid=result['global_costmap']['global_costmap']['ros__parameters']
    resolution=grid['resolution']
    if resolution!=.05:raise ValueError('The experimental primitives require the reviewed 5cm resolution')
    padding=grid['footprint_padding']
    footprint=json.loads(grid['footprint'])
    radius=max(math.hypot(x+math.copysign(padding,x),y+math.copysign(padding,y)) for x,y in footprint)
    grid['inflation_layer']['inflation_radius']=round(math.ceil((radius+resolution)/resolution)*resolution,6)
    if frozen_replay:grid['obstacle_layer']['enabled']=False
    planner=result['planner_server']['ros__parameters']['GridBased']
    planner.update(plugin='p2_nav_controller::GaitFeasibleLattice',lattice_filepath=str(EXPERIMENT/f'biped-straight-pivot{headings}.json'),smooth_path=False,analytic_expansion_max_length=.001,lookup_table_size=.05,gait_max_midroute_rotation=math.pi/2,gait_min_forward_leg=.5,gait_terminal_region=.3)
    # Deliberately leave controller and goal checker selection to the baseline.
    # Experimental gait tracking variants have not passed whole navigation.
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True,type=Path);parser.add_argument('--frozen-replay',action='store_true');parser.add_argument('--headings',type=int,choices=[32],default=32);args=parser.parse_args()
    args.output.write_text(yaml.safe_dump(parameters(args.frozen_replay,args.headings),sort_keys=False))
