"""Mid-mission semantic choices; physical feasibility remains with Nav2."""
import copy
import math
from reception_routes import route,worthwhile_shortcut

class LiveRouter:
    def __init__(self):self.reset()
    def reset(self):self.fingerprint=None;self.last_check=-math.inf;self.last_reason=''
    def choose(self,document,states,pose,command,previous=None,remaining=None,now=0.,waiting=False):
        fingerprint=tuple(sorted(states.items()))
        if fingerprint==self.fingerprint and (not waiting or now-self.last_check<document['policy']['replan_interval']):return None
        self.fingerprint=fingerprint;self.last_check=now
        points={p['id']:p for p in document['points']};original=command['points'];pending=original
        if type(remaining)is int and 0<remaining<=len(original):
            passed=len(original)-remaining
            pending=original[passed:]
            if passed:previous=original[passed-1]['id']
        if not pending or not pose:return None
        start=previous if previous in points else min(points,key=lambda i:math.hypot(points[i]['x']-pose[0],points[i]['y']-pose[1]))
        goal=pending[-1]['id']
        if start==goal:return None
        old_nodes=[start]+[p['id']for p in pending]
        old_nodes=[p for i,p in enumerate(old_nodes)if i==0 or old_nodes[i-1]!=p]
        blocked=False
        for a,b in zip(old_nodes,old_nodes[1:]):
            edges=[e for e in document['edges']if e['a']==a and e['b']==b or e['bidirectional']and e['b']==a and e['a']==b]
            if edges and all(e['door_id']and states.get(e['door_id'])!='open'for e in edges):blocked=True
        candidate=route(document,start,goal,states)
        if candidate is None:
            return {'wait':True,'reason':'通道受阻，暂时没有可用绕行连接'}if blocked or waiting else None
        ids=candidate['points'][1:]
        replacement=[]
        for i in ids:
            point=copy.deepcopy(points[i])
            if i!=goal:point.update(kind='transit',yaw=None,script='',product_id='',dwell=0.,guest_wait=False)
            replacement.append(point)
        def distance_path(path):
            coordinates=[pose[:2]]+[[p['x'],p['y']]for p in path]
            return sum(math.hypot(b[0]-a[0],b[1]-a[1])for a,b in zip(coordinates,coordinates[1:]))
        old_cost=distance_path(pending);new_cost=distance_path(replacement)
        changed=ids!=[p['id']for p in pending]
        if waiting or blocked or changed and worthwhile_shortcut(old_cost,new_cost,document['policy']['shortcut_gain']):
            return {'wait':False,'points':replacement,'reason':'通道已恢复，继续行程'if waiting else '当前通道受阻，改走可用连接'if blocked else '发现更短的开放通道',
                    'old_cost':old_cost,'new_cost':new_cost,'edges':candidate['edges']}
        return None
