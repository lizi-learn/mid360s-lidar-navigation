"""Synthetic planar door fixtures. Never imported by a real motor interface."""
import math
import numpy as np

def checked_doors(rows):
    if not isinstance(rows,list)or len(rows)>20:raise ValueError('Synthetic doors must be a list of at most20')
    result=[]
    for d in rows:
        if not isinstance(d,dict)or set(d)!={'id','x','y','yaw','width','thickness','closed'}or type(d['closed'])is not bool:raise ValueError('Invalid synthetic door')
        if not isinstance(d['id'],str)or len(d['id'])>64:raise ValueError('Invalid synthetic door identity')
        if any(type(d[k])not in(int,float)or not math.isfinite(d[k])for k in('x','y','yaw','width','thickness')):raise ValueError('Invalid synthetic door coordinates')
        if not .1<=d['width']<=10 or not .05<=d['thickness']<=1:raise ValueError('Invalid synthetic door dimensions')
        result.append(dict(d))
    return result

def occupied(doors,x,y):
    result=np.zeros(np.broadcast(x,y).shape,dtype=bool)
    for d in doors:
        if not d['closed']:continue
        dx=x-d['x'];dy=y-d['y'];c,s=math.cos(d['yaw']),math.sin(d['yaw'])
        result|=(np.abs(c*dx+s*dy)<=d['width']/2)&(np.abs(-s*dx+c*dy)<=d['thickness']/2)
    return result
