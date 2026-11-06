"""Adapt the existing Console API; do not publish cmd_vel or drive joints."""
import copy
import math
import unicodedata
import uuid
from .http import JsonHTTP
from .errors import NavigationError
from .validation import finite, text

BUSY = {"dispatching", "navigating", "rerouting", "waiting_route", "presenting", "guest_wait", "dwell","paused"}
SESSION_STATE = {"planning":"accepted", "ready":"accepted", "starting":"accepted",
                 "executing":"running", "replanning":"blocked", "arrived":"succeeded",
                 "blocked":"failed", "cancelled":"cancelled", "stopped":"cancelled",
                 "manual":"cancelled", "paused":"paused", "idle":"unknown"}
MISSION_STATE = {"dispatching":"accepted", "navigating":"running", "rerouting":"blocked",
                 "waiting_route":"blocked", "presenting":"presenting", "guest_wait":"waiting",
                 "dwell":"dwelling", "paused":"paused", "completed":"succeeded",
                 "failed":"failed", "cancelled":"cancelled", "manual":"cancelled",
                 "interrupted":"interrupted"}

class P2Backend:
    name = "p2"
    def __init__(self, base_url="http://127.0.0.1:8930", *, timeout=20.):
        self.http = JsonHTTP(base_url, timeout=timeout)

    def raw_status(self): return self.http.request("GET", "/api/status?summary=1")
    @staticmethod
    def current_task(status):
        session=status.get("session") or {};mission=(status.get("reception")or{}).get("mission")or{}
        if mission.get("phase")in BUSY:
            return {"type":"mission","phase":mission["phase"],"run_id":mission.get("run_id"),
                    "route_name":mission.get("route_name"),"point":mission.get("point"),
                    "station_index":mission.get("index"),"station_total":mission.get("total")}
        if session.get("cancel_pending"):
            return {"type":"cancellation","phase":"cancelling","generation":session.get("generation")}
        phase=session.get("phase")
        # E-middle sets the session to paused even before any goal exists, and
        # E-left deliberately does not resume it. This is idle switch state,
        # not an occupied task. A completed/cancelled goal can also be retained.
        paused_goal=(phase=="paused" and session.get("goal")is not None
                     and not session.get("terminal_receipt")
                     and (not session.get("last_stop")or session["last_stop"].get("phase")=="paused"))
        if phase in("starting","executing","replanning")or paused_goal:
            return {"type":"navigation","phase":phase,"generation":session.get("generation")}
        return None
    def _post(self, path, body):
        token = self.raw_status().get("request_token")
        if not token: raise NavigationError("BACKEND_PROTOCOL", "Console request token missing", status=502)
        return self.http.request("POST", path, body, {"X-Nav-Token": token})
    def _document(self): return self.http.request("GET", "/api/reception")["document"]

    def _extension(self):
        try: return self.http.request("GET", "/api/sdk/version")
        except NavigationError as exc:
            if exc.status == 404: return {"conditional_task_control": False}
            raise

    def capabilities(self):
        extension=self._extension();enabled=bool(extension.get("conditional_task_control"))
        return {"backend":self.name, "api_version":"1.0", "hardware":True,
                "features":{"move_pose":enabled,"move_marker":enabled,"saved_routes":enabled,
                            "marker_crud":True,"visit_introduction":enabled,"maps_list":True,
                            "global_cancel":True,"conditional_task_control":enabled,
                            **{k:enabled and bool(extension.get(k))for k in('relative_forward','relative_rotation','visit_sequence')},
                            'motion_sequence':enabled and bool(extension.get('conditional_relative_chaining')),
                            "velocity_si":False,"elevator":False,"charging":False,"motor_estop":False},
                "verification":"P2 navigation derived from V2; SDK adapter contract-tested separately",
                "units":{"position":"m","theta":"rad","D_input":"manual_equivalent_not_measured_m/s"}}

    def status(self):
        s=self.raw_status();pose=s.get("pose");gate=s.get("gate") or {};session=s.get("session") or {}
        current=self.current_task(s)
        state=SESSION_STATE.get(session.get("phase"),"unknown")
        if current and current["type"]=="mission":state=MISSION_STATE[current["phase"]]
        elif not current and session.get("phase")=="paused":state="idle"
        return {"backend":self.name,"robot_id":s.get("robot_name"),"map_id":(s.get("reception")or{}).get("map_id"),
                "pose":None if not pose else {"x":pose[0],"y":pose[1],"theta":pose[2],"frame":"map"},
                "data_fresh":gate.get("data_fresh"),"localized":gate.get("localized"),
                "operator_mode":session.get("operator_mode"),"navigation_permitted":session.get("navigation_permitted"),
                "navigation_state":state,"task_active":current is not None,"current_task":current,
                "notice":session.get("notice"),"battery_percent":None,"charging":None,
                "hardware_estop":None,"software_estop":None,
                "unknown_fields":["battery_percent","charging","hardware_estop","software_estop"]}

    def maps(self):
        s=self.raw_status();current=(s.get("reception")or{}).get("map_id")
        return [{"id":x,"active":x==current}for x in s.get("maps",[])]

    @staticmethod
    def marker(p, map_id):
        return {"id":p["id"],"name":p["name"],"x":p["x"],"y":p["y"],"theta":p.get("yaw"),
                "map_id":map_id,"script":p.get("script",""),"dwell":p.get("dwell",0),
                "guest_wait":p.get("guest_wait",False),"kind":p.get("kind","visit")}
    def markers(self):
        d=self._document();return [self.marker(p,d["map_id"])for p in d["points"]]
    def routes(self): return self._document()["routes"]

    def _resolve(self, target, document):
        if "marker" in target:
            points=[p for p in document["points"]if p["name"]==target["marker"] or p["id"]==target["marker"]]
            if len(points)!=1: raise NavigationError("MARKER_NOT_FOUND" if not points else "AMBIGUOUS_MARKER", "Marker must resolve uniquely",status=404 if not points else 409)
            p=points[0];return {"x":p["x"],"y":p["y"],"theta":p.get("yaw")}
        return target

    def submit(self, request, *, after=None, cancelled=lambda:False):
        extension=self._extension()
        if not extension.get("conditional_task_control"):
            raise NavigationError("UNSUPPORTED_CAPABILITY","Install the additive Console SDK extension to enable concurrent-safe dispatch")
        s=self.raw_status();session=s["session"];mission=s["reception"]["mission"]
        if not session.get("navigation_permitted"):
            raise NavigationError('OPERATOR_PAUSED'if session.get('operator_mode')=='paused'else"MANUAL_PRIORITY",session.get("notice") or "E左档才允许导航",status=409)
        current=self.current_task(s)
        if current:
            raise NavigationError("TASK_BUSY","Existing navigation must finish or be explicitly cancelled",details={"current_task":current})
        if after is not None:
            phase=after.get('resume_completed_phase','arrived')
            if session['phase']=='paused'and phase!='paused':raise NavigationError('OPERATOR_PAUSED','Previous step paused; explicit continuation required')
            if(after.get('type')!='navigation'or after.get('generation')!=session['generation']or after.get('map_id')!=s['reception']['map_id']or session['phase']!=phase
               or phase!='arrived'and not(session.get('terminal_receipt')or{}).get('succeeded')):
                raise NavigationError('TASK_REPLACED','Previous sequence step no longer has an unchanged success receipt')
        expected={"generation":session["generation"],"run_id":mission["run_id"],"epoch":mission["epoch"],"map_id":s["reception"]["map_id"]}
        feature={'forward':'relative_forward','rotate':'relative_rotation','tour':'visit_sequence'}.get(request['kind'])
        if feature and not extension.get(feature):raise NavigationError('UNSUPPORTED_CAPABILITY','Console extension does not yet support '+feature)
        if request['kind']in('forward','rotate'):
            key='meters'if request['kind']=='forward'else'radians'
            path='/api/relative-goal'if request['kind']=='forward'else'/api/rotate'
            if cancelled():raise NavigationError('SEQUENCE_CANCELLED','Sequence intent cancelled before dispatch')
            payload={key:request[key],'_sdk_expected':expected}
            if after is not None:payload['_sdk_expected_phase']=after.get('resume_completed_phase','arrived')
            prepared=self._post(path,payload)
            if cancelled():
                self._post('/api/stop',{'_sdk_expected':dict(expected,generation=prepared['generation'])})
                raise NavigationError('SEQUENCE_CANCELLED','Sequence cancelled before confirmation')
            ref=dict(type='navigation',generation=prepared['generation'],map_id=s['reception']['map_id'],intent=request['kind'])
            try:self._post('/api/confirm',{'generation':prepared['generation']})
            except NavigationError as exc:exc.details={**exc.details,'native_ref':ref};raise
            return ref
        if request["kind"] in ("route","visit","tour"):
            d=self._document();body={"command":"start","revision":d["revision"]}
            if request["kind"]=="route":body["route_id"]=request["route_id"]
            elif request['kind']=='tour':
                ids=[]
                for marker in request['markers']:
                    matches=[p for p in d['points']if marker in(p['name'],p['id'])]
                    if len(matches)!=1:raise NavigationError('MARKER_NOT_FOUND','Each tour marker must resolve uniquely',status=404)
                    ids.append(matches[0]['id'])
                body['point_ids']=ids
            else:
                points=[p for p in d["points"]if request["marker"]in(p["name"],p["id"])]
                if len(points)!=1:raise NavigationError("MARKER_NOT_FOUND","Marker must resolve uniquely",status=404)
                body["point_id"]=points[0]["id"]
            payload={"reception_command":body}
            payload['_sdk_expected']=expected
            m=self._post("/api/confirm",payload)
            return {"type":"mission","run_id":m["run_id"],"epoch":m["epoch"],"map_id":d["map_id"]}
        d=self._document();p=self._resolve(request["target"],d)
        goal={"x":p["x"],"y":p["y"],"yaw":p["theta"]if p.get("theta")is not None else 0.,
              "heading_mode":"pose"if p.get("theta")is not None else"position","_arrival_profile":"presentation",
              "_sdk_expected_generation":session["generation"],
              "_sdk_expected":{"generation":session["generation"],"run_id":mission["run_id"],"epoch":mission["epoch"],"map_id":s["reception"]["map_id"]}}
        prepared=self._post("/api/goal",goal)
        self._post("/api/confirm",{"generation":prepared["generation"]})
        return {"type":"navigation","generation":prepared["generation"],"map_id":d["map_id"]}

    def observe(self, ref):
        s=self.raw_status();ss=s["session"];m=s["reception"]["mission"]
        if s["reception"]["map_id"]!=ref.get("map_id"):
            return {"state":"interrupted","reason":"map_changed"}
        if ref["type"]=="mission":
            if m["run_id"]!=ref["run_id"]:return {"state":"interrupted","reason":"task_replaced"}
            return {"state":MISSION_STATE.get(m["phase"],"unknown"),"native_phase":m["phase"],
                    "error":m.get("error"),"station_index":m.get("index"),"station_total":m.get("total"),
                    "arrival_reached":m["phase"]in("presenting","guest_wait","dwell","completed"),
                    "navigation_state":"succeeded"if m["phase"]=="completed"else SESSION_STATE.get(ss["phase"],"unknown")}
        if ss["generation"]!=ref["generation"]:return {"state":"interrupted","reason":"task_replaced_or_manual_takeover"}
        return {"state":SESSION_STATE.get(ss["phase"],"unknown"),"native_phase":ss["phase"],
                "reason":ss.get("notice"),"arrival_reached":ss["phase"]=="arrived",
                'native_terminal_succeeded':bool((ss.get('terminal_receipt')or{}).get('succeeded')and(ss.get('terminal_receipt')or{}).get('generation')==ref['generation']),
                "navigation_state":SESSION_STATE.get(ss["phase"],"unknown")}

    def control(self, ref, operation):
        s=self.raw_status();ss=s["session"];m=s["reception"]["mission"]
        current=self.observe(ref)
        if current["state"]in("succeeded","failed","cancelled","interrupted","unknown"):
            raise NavigationError("TASK_TERMINAL","Task is terminal or its authority was replaced")
        if not self._extension().get("conditional_task_control"):
            raise NavigationError("UNSUPPORTED_CAPABILITY","Task-scoped control needs the additive Console SDK extension; global stop remains available")
        expected={"generation":ss["generation"],"run_id":m["run_id"],"epoch":m["epoch"],"map_id":s["reception"]["map_id"]}
        if operation=="cancel":
            self._post("/api/stop",{"_sdk_expected":expected});return ref
        if ref["type"]!="mission":raise NavigationError("UNSUPPORTED_CAPABILITY","Pause/continue is available for saved visits and routes; coordinate moves support cancel")
        body={"reception_command":{"command":operation,"epoch":m["epoch"],"map_id":s["reception"]["map_id"]},"_sdk_expected":expected}
        result=self._post("/api/confirm",body)
        return {**ref,"epoch":result["epoch"]}

    def stop(self): self._post("/api/stop",{});return {"stop_requested":True,"scope":"navigation_only"}

    def resume_relative(self, ref, request, *, cancelled=lambda:False):
        s=self.raw_status();ss=s['session'];mission=s['reception']['mission'];pose=s.get('pose');goal=ss.get('goal')
        if(ref.get('type')!='navigation'or ref.get('generation')!=ss['generation']or ref.get('map_id')!=s['reception']['map_id']
           or ss['phase']not in('paused','cancelled')or not ss.get('navigation_permitted')or not goal):
            raise NavigationError('MANUAL_PRIORITY','Sequence pause reference changed or E does not permit explicit resume')
        expected=dict(generation=ss['generation'],run_id=mission['run_id'],epoch=mission['epoch'],map_id=s['reception']['map_id'])
        if request['kind']=='forward':
            path='/api/goal';payload=dict(goal,heading_mode='position',_sdk_expected_generation=ss['generation'])
        else:
            if abs(request['radians'])>math.pi or not pose:
                raise NavigationError('UNSUPPORTED_CAPABILITY','Paused multi-turn rotation has no native accumulated-angle feedback; no full-turn replay')
            remaining=math.remainder(goal['yaw']-pose[2],math.tau)
            if remaining==0:raise NavigationError('POSE_UNAVAILABLE','Exact-zero residual has no new action receipt; query or cancel the sequence')
            path='/api/rotate';payload=dict(radians=remaining)
        payload.update(_sdk_expected=expected,_sdk_expected_phase=ss['phase'])
        if cancelled():raise NavigationError('SEQUENCE_CANCELLED','Sequence cancelled before explicit resume')
        prepared=self._post(path,payload)
        if cancelled():
            self._post('/api/stop',{'_sdk_expected':dict(expected,generation=prepared['generation'])})
            raise NavigationError('SEQUENCE_CANCELLED','Sequence cancelled before resume confirmation')
        self._post('/api/confirm',{'generation':prepared['generation']})
        return dict(ref,generation=prepared['generation'])

    def save_marker(self, data):
        if not isinstance(data,dict)or set(data)-{"name","x","y","theta","script","dwell","guest_wait","kind"}:
            raise NavigationError("INVALID_ARGUMENT","Unsupported marker fields",status=400)
        name=text(data.get("name"),"name");d=self._document();copy_d=copy.deepcopy(d)
        key=lambda n:unicodedata.normalize("NFKC",n).strip().casefold()
        p=next((x for x in copy_d["points"]if key(x["name"])==key(name)),None)
        if p is None:
            pose=self.status()["pose"]
            if ("x"not in data or"y"not in data)and pose is None:
                raise NavigationError("POSE_UNAVAILABLE","Current measured pose unavailable")
            p={"id":"sdk_"+uuid.uuid4().hex,"name":name,"x":data.get("x",(pose or{}).get("x")),
               "y":data.get("y",(pose or{}).get("y")),"yaw":data.get("theta",(pose or{}).get("theta")),
               "product_id":"","script":"","dwell":0.,"guest_wait":False,"kind":"visit"};copy_d["points"].append(p)
        for k,v in data.items():p["yaw"if k=="theta"else k]=v
        self._post("/api/save",{"reception":{"operation":"save","document":copy_d,"expected_revision":d["revision"]}})
        return self.marker(p,d["map_id"])

    def delete_marker(self, name):
        d=self._document();points=[p for p in d["points"]if name in(p["name"],p["id"])]
        if len(points)!=1:raise NavigationError("MARKER_NOT_FOUND","Marker must resolve uniquely",status=404)
        pid=points[0]["id"]
        if any(s["point_id"]==pid for r in d["routes"]for s in r["steps"])or any(pid in(e["a"],e["b"])for e in d["edges"]):
            raise NavigationError("MARKER_IN_USE","Remove the marker from saved routes/edges before deleting it")
        changed=copy.deepcopy(d);changed["points"]=[p for p in d["points"]if p["id"]!=pid]
        self._post("/api/save",{"reception":{"operation":"save","document":changed,"expected_revision":d["revision"]}})
        return {"deleted":True,"name":points[0]["name"]}
