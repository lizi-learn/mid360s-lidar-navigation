"""Explicit in-place rotation intent; no global planner or actuator access."""
import math
from pathlib import Path

def rotation_goal(pose,radians):
    if type(radians)not in (int,float)or not math.isfinite(radians)or not 0<abs(radians)<=100000:
        raise ValueError('原地转角须为有限非零弧度值')
    if pose is None or len(pose)!=3 or not all(math.isfinite(v)for v in pose):
        raise ValueError('原地转向缺少新鲜定位姿态')
    return dict(x=pose[0],y=pose[1],yaw=math.remainder(pose[2]+radians,math.tau))

def rotation_tree(folder,radians):
    # Revalidate before rendering a numeric action port, never interpolate UI XML.
    rotation_goal([0.,0.,0.],radians)
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    path=folder/('right'if radians<0.else'left')
    path=path.with_name(path.name+'-'+float(radians).hex().replace('-','m').replace('+','p')+'.xml')
    path.write_text('<?xml version="1.0"?>\n<root main_tree_to_execute="MainTree">\n'
        '<BehaviorTree ID="MainTree"><Spin spin_dist="'+format(radians,'.17g')+
        '" time_allowance="'+format(max(45.,45.*abs(radians)/math.pi),'.17g')+'"/></BehaviorTree>\n</root>\n')
    return str(path)
