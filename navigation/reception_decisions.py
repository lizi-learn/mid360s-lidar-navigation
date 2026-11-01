"""Finite semantic route observations. Never edits an occupancy map."""
import math
import threading
from reception_routes import DoorEvidence, route, worthwhile_shortcut

class RouteDecisions:
    def __init__(self):
        self.lock=threading.RLock();self.doors={};self.sequence=0
    def observe(self,data,now,clear_hold=1.5):
        if not isinstance(data,dict) or set(data)-{'door_id','state','source_monotonic','source_sequence'}:
            raise ValueError('门状态证据字段无效')
        door=data.get('door_id')
        if not isinstance(door,str)or len(door)>64:raise ValueError('门标识无效')
        with self.lock:
            evidence=self.doors.setdefault(door,DoorEvidence())
            accepted=evidence.observe(data.get('state'),data.get('source_monotonic',float('nan')),data.get('source_sequence'),now,clear_hold)
            if accepted:self.sequence+=1
            return accepted
    def states(self,document,now):
        with self.lock:return {d['id']:self.doors[d['id']].current(now)if d['id']in self.doors else 'unknown'for d in document['doors']}
    def plan(self,document,start,goal,now):return route(document,start,goal,self.states(document,now))
    def expand(self,document,route_id,now):
        """Adaptive routes keep visit order, replace optional transit waypoints.

        Unknown door edges are excluded only from this semantic graph. The
        existing Nav2 free-space planner remains authoritative for real motion.
        """
        import copy
        result=copy.deepcopy(document);r=next(r for r in result['routes']if r['id']==route_id)
        if r['mode']!='adaptive':return result,[]
        points={p['id']:p for p in document['points']}
        visits=[s for s in r['steps']if points[s['point_id']]['kind']!='transit']
        if not visits:visits=r['steps']
        steps=[];reports=[]
        for index,step in enumerate(visits):
            if index:
                previous=visits[index-1]['point_id'];path=self.plan(document,previous,step['point_id'],now)
                if path:
                    steps.extend({'point_id':i,'speak':False}for i in path['points'][1:-1]);reports.append(path)
            steps.append(step)
        r['steps']=steps
        return result,reports
