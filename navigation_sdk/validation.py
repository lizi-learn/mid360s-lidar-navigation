import math
from .errors import NavigationError

def finite(value, name):
    if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 100000:
        raise NavigationError("INVALID_ARGUMENT", name + " must be a finite number", status=400)
    return float(value)

def text(value, name, maximum=80):
    if not isinstance(value, str) or not value.strip() or len(value) > maximum or any(ord(c)<32 for c in value):
        raise NavigationError("INVALID_ARGUMENT", "Invalid " + name, status=400)
    return value.strip()

def task_request(data):
    if not isinstance(data, dict): raise NavigationError("INVALID_ARGUMENT", "Expected an object", status=400)
    if data.get('kind')in('forward','rotate')and set(data)=={'kind','meters'if data['kind']=='forward'else'radians'}:
        key='meters'if data['kind']=='forward'else'radians';value=finite(data[key],key)
        if value==0 or key=='meters'and value<0:raise NavigationError('INVALID_ARGUMENT','Distance must be positive; rotation must be nonzero',status=400)
        return dict(kind=data['kind'],**{key:value})
    if data.get('kind')=='tour'and set(data)=={'kind','markers'}:
        if not isinstance(data['markers'],list)or not 1<=len(data['markers'])<=100:
            raise NavigationError('INVALID_ARGUMENT','Tour needs 1 to 100 saved markers',status=400)
        return dict(kind='tour',markers=[text(p,'marker')for p in data['markers']])
    if data.get("kind") == "visit" and set(data) == {"kind", "marker"}:
        return {"kind":"visit", "marker":text(data["marker"], "marker")}
    if data.get("kind") == "route" and set(data) == {"kind", "route_id"}:
        return {"kind": "route", "route_id": text(data["route_id"], "route_id", 64)}
    if data.get("kind") != "move" or set(data) != {"kind", "target"} or not isinstance(data["target"], dict):
        raise NavigationError("INVALID_ARGUMENT", "Expected a move target or route_id", status=400)
    target = data["target"]
    if set(target) == {"marker"}: target = {"marker": text(target["marker"], "marker")}
    elif set(target) in ({"x", "y"}, {"x", "y", "theta"}):
        target = {"x": finite(target["x"], "x"), "y": finite(target["y"], "y"),
                  "theta": None if target.get("theta") is None else finite(target["theta"], "theta")}
        if target["theta"] is not None and abs(target["theta"]) > math.pi:
            raise NavigationError("INVALID_ARGUMENT", "theta is radians in [-pi,pi]", status=400)
    else: raise NavigationError("INVALID_ARGUMENT", "Specify one marker OR one metric pose", status=400)
    return {"kind": "move", "target": target}
