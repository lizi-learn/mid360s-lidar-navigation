"""Metric relative targets from a fresh map -> base_link pose, not input*time."""
import math

def forward_goal(pose, meters):
    if type(meters)not in(int,float)or not math.isfinite(meters)or not 0<meters<=100000:
        raise ValueError('前进距离须为正的有限米数')
    if pose is None or len(pose)!=3 or not all(math.isfinite(v)for v in pose):
        raise ValueError('相对前进缺少新鲜机身定位姿态')
    x,y,yaw=pose
    return dict(x=x+meters*math.cos(yaw),y=y+meters*math.sin(yaw),yaw=yaw,heading_mode='position')

def visit_sequence(document, point_ids):
    """Build one temporary mission snapshot; never write the saved catalogue."""
    if not isinstance(point_ids,list)or not 1<=len(point_ids)<=100 or any(not isinstance(p,str)for p in point_ids):
        raise ValueError('参观点位需包含1到100个已保存点位ID')
    known={p['id']for p in document['points']}
    if any(p not in known for p in point_ids):raise ValueError('参观序列包含不存在的点位')
    route_id='temporary_sequence'
    document['routes']=[r for r in document['routes']if r['id']!=route_id]+[dict(
        id=route_id,name='临时产品参观',mode='showcase',loops=1,description='',
        steps=[dict(point_id=p,speak=True)for p in point_ids])]
    return route_id
