"""3D rolling-scan geometry for reproducible motion compensation tests.

Pose input must come from an estimator or an explicitly labelled test oracle.
This module is not a LiDAR-inertial state estimator or a Livox optical model.
"""
import numpy as np
from scipy.spatial.transform import Rotation,Slerp

class PoseSeries:
    def __init__(self, stamps, position, quaternion_xyzw, max_gap=.02):
        self.stamps=np.asarray(stamps,dtype=float);self.position=np.asarray(position,dtype=float)
        q=np.asarray(quaternion_xyzw,dtype=float)
        n=len(self.stamps)
        if n<2 or self.stamps.ndim!=1 or self.position.shape!=(n,3) or q.shape!=(n,4):raise ValueError('Invalid pose series dimensions')
        if not np.isfinite(np.r_[self.stamps,self.position.ravel(),q.ravel()]).all():raise ValueError('Nonfinite pose series')
        if (np.diff(self.stamps)<=0).any() or (abs(np.linalg.norm(q,axis=1)-1)>.01).any():raise ValueError('Invalid pose timing or quaternion')
        self.max_gap=max_gap;self.rotation=Rotation.from_quat(q);self.slerp=Slerp(self.stamps,self.rotation)

    def at(self, stamps):
        times=np.asarray(stamps,dtype=float)
        if times.ndim!=1 or not np.isfinite(times).all() or (times<self.stamps[0]).any() or (times>self.stamps[-1]).any():raise ValueError('Point time outside pose history')
        right=np.clip(np.searchsorted(self.stamps,times),1,len(self.stamps)-1)
        if (self.stamps[right]-self.stamps[right-1]>self.max_gap).any():raise ValueError('Pose gap too large for compensation')
        p=np.column_stack([np.interp(times,self.stamps,self.position[:,k]) for k in range(3)])
        return p,self.slerp(times)

    def world_points(self, points, point_stamps):
        xyz=np.asarray(points,dtype=float)
        if xyz.shape!=(len(point_stamps),3) or not np.isfinite(xyz).all():raise ValueError('Invalid scan points')
        p,r=self.at(point_stamps)
        return r.apply(xyz)+p


def raycast_boxes(origins, directions, boxes, floor=0., ceiling=3.):
    o=np.asarray(origins,dtype=float);d=np.asarray(directions,dtype=float);boxes=np.asarray(boxes,dtype=float)
    if o.ndim!=2 or o.shape[1]!=3 or d.shape!=o.shape or boxes.ndim!=2 or boxes.shape[1]!=6:raise ValueError('Invalid ray scene')
    if not np.isfinite(np.r_[o.ravel(),d.ravel(),boxes.ravel()]).all() or not np.allclose(np.linalg.norm(d,axis=1),1,atol=1e-5):raise ValueError('Rays must be finite unit vectors')
    if (boxes[:,3:]<=boxes[:,:3]).any():raise ValueError('Invalid box extent')
    distance=np.full(len(o),np.inf)
    down=d[:,2]<-1e-8;up=d[:,2]>1e-8
    distance[down]=(floor-o[down,2])/d[down,2]
    distance[up]=(ceiling-o[up,2])/d[up,2]
    distance[distance<=0]=np.inf
    for b in boxes:
        parallel=abs(d)<1e-12
        outside=(parallel&((o<b[:3])|(o>b[3:]))).any(axis=1)
        safe=np.where(parallel,1.,d)
        aa=(b[:3]-o)/safe;bb=(b[3:]-o)/safe
        lo=np.where(parallel,-np.inf,np.minimum(aa,bb)).max(axis=1)
        hi=np.where(parallel,np.inf,np.maximum(aa,bb)).min(axis=1)
        hit=np.where(lo>1e-6,lo,hi)
        valid=(~outside)&(hi>=np.maximum(lo,0.))&(hit>1e-6)
        distance[valid]=np.minimum(distance[valid],hit[valid])
    return distance
