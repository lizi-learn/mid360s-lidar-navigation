"""Lightweight display/reroute receipt, preserving the full original path.

Nav2 still consumes the original Path. This representation only keeps the
coordinates and original ROS source time needed by the workbench, so its
control heartbeat need not allocate thousands of nested ROS pose objects.
"""
import json,math

def encode_view(points,source_ns):
    value={'version':1,'source_ros_ns':source_ns,'points':points}
    raw=json.dumps(value,allow_nan=False,separators=(',',':'))
    decode_view(raw)
    return raw

def decode_view(raw):
    value=json.loads(raw)
    if not isinstance(value,dict) or value.get('version')!=1:raise ValueError('path_view_version')
    stamp=value['source_ros_ns'];points=value['points']
    if type(stamp) is not int or stamp<=0 or not isinstance(points,list):raise ValueError('path_view_source')
    if any(not isinstance(p,list) or len(p)!=2 or any(type(x) not in (int,float) or not math.isfinite(x) for x in p) for p in points):raise ValueError('path_view_coordinates')
    return points,stamp
