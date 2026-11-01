"""Finite projected-LiDAR ray evidence and a reversible authored-door overlay.

No ray return is never free-space evidence. Opening evidence is 2D scan
visibility, not a claim that an entire 3D body-height volume was observed.
"""
import copy
import math
import numpy as np

class DoorRays:
    def __init__(self,bin_size=.10,window=1.):
        self.bin_size=bin_size;self.window=window;self.histories={};self.last_stamp=-math.inf
    def observe(self,document,origin,endpoints,stamp,now):
        if not math.isfinite(stamp)or not -.05<=now-stamp<=.5 or stamp<=self.last_stamp:return []
        self.last_stamp=stamp
        points=np.asarray(endpoints,dtype=float)
        if points.ndim!=2 or points.shape[1]!=2 or len(points)>4096 or not np.isfinite(points).all():return []
        source=np.asarray(origin,dtype=float)
        if source.shape!=(2,)or not np.isfinite(source).all():return []
        results=[]
        for door in document['doors']:
            centre=np.array([door['x'],door['y']]);relative=points-centre
            if np.linalg.norm(source-centre)>12:continue
            tangent=np.array([math.cos(door['yaw']),math.sin(door['yaw'])]);normal=np.array([-tangent[1],tangent[0]])
            width=door['width'];count=max(1,math.ceil(width/self.bin_size));key=(document['map_id'],door['id'],door['x'],door['y'],width,door['yaw'])
            bins=self.histories.setdefault(key,np.full(count,-math.inf));s0=float((source-centre)@normal)
            n=relative@normal;u=relative@tangent
            hits=(np.abs(n)<=.125)&(np.abs(u)<=width/2)
            blockers=int(np.count_nonzero(hits))
            if blockers>=2:
                bins[:]=-math.inf;state='closed'
            else:
                crosses=(s0*n<0)&(np.abs(n)>=.25)
                denom=n-s0;fraction=np.divide(-s0,denom,out=np.full_like(n,np.nan),where=np.abs(denom)>1e-9)
                crossing=source+(points-source)*fraction[:,None]
                lateral=(crossing-centre)@tangent
                valid=crosses&(np.abs(lateral)<=width/2)&np.isfinite(lateral)
                indices=np.floor((lateral[valid]+width/2)/width*count).astype(int);indices=np.clip(indices,0,count-1)
                bins[indices]=stamp
                state='open'if bool(np.all(now-bins<=self.window))else 'unknown'
            covered=int(np.count_nonzero(now-bins<=self.window))
            results.append({'door_id':door['id'],'state':state,'source_monotonic':stamp,
                            'coverage_bins':covered,'total_bins':count,'blocking_returns':blockers,
                            'scope':'finite projected scan rays, not full 3D clearance'})
        # Bound history when annotations/maps change.
        live={(document['map_id'],d['id'],d['x'],d['y'],d['width'],d['yaw'])for d in document['doors']}
        self.histories={k:v for k,v in self.histories.items()if k in live}
        return results

class DoorOverlay:
    """Only authored door stripes can differ; the original map is never edited."""
    def __init__(self):self.cached_key=None;self.masks={}
    def apply(self,grid,document,states):
        # grid contains width,height,resolution,origin,cells (Nav2 convention).
        key=(grid['width'],grid['height'],grid['resolution'],tuple(grid['origin']),
             tuple((d['id'],d['x'],d['y'],d['width'],d['yaw'])for d in document['doors']))
        if key!=self.cached_key:
            self.cached_key=key;self.masks={};res=grid['resolution'];ox,oy=grid['origin']
            for d in document['doors']:
                radius=d['width']/2+.15;indices=[];c,s=math.cos(d['yaw']),math.sin(d['yaw'])
                for row in range(max(0,math.floor((d['y']-radius-oy)/res)),min(grid['height'],math.ceil((d['y']+radius-oy)/res))):
                    for col in range(max(0,math.floor((d['x']-radius-ox)/res)),min(grid['width'],math.ceil((d['x']+radius-ox)/res))):
                        x=ox+(col+.5)*res-d['x'];y=oy+(row+.5)*res-d['y']
                        # Cell centres in the .15m authored leaf stripe. The full
                        # configured robot footprint remains with Nav2 checks.
                        if abs(c*x+s*y)<=d['width']/2 and abs(-s*x+c*y)<=.075:
                            indices.append(row*grid['width']+col)
                self.masks[d['id']]=indices
        output=list(grid['cells']);base=grid['cells'];changes=[]
        for d in document['doors']:
            state=states.get(d['id'],'unknown')
            for i in self.masks[d['id']]:
                if state=='closed':output[i]=100
                elif state=='open'and base[i]>=65:output[i]=0
        for i,(a,b)in enumerate(zip(base,output)):
            if a!=b:changes.append(i)
        return output,{'changed_cells':len(changes),'door_states':dict(states),'original_map_modified':False,
                       'unknown_cells_promoted':sum(base[i]<0 and output[i]==0 for i in changes)}
