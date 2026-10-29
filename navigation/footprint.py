"""Explicit convex ground envelope; geometry metadata does not certify hardware."""
import json
from pathlib import Path
import numpy as np

DEFAULT = [[.35,.40],[.35,-.40],[-.35,-.40],[-.35,.40]]

def checked(vertices):
    p=np.asarray(vertices,dtype=float)
    if p.ndim!=2 or p.shape[1]!=2 or not 3<=len(p)<=64 or not np.isfinite(p).all():raise ValueError('Invalid footprint vertices')
    edges=np.roll(p,-1,axis=0)-p
    if (np.linalg.norm(edges,axis=1)<1e-6).any():raise ValueError('Repeated footprint vertices')
    crosses=edges[:,0]*np.roll(edges,-1,axis=0)[:,1]-edges[:,1]*np.roll(edges,-1,axis=0)[:,0]
    if not ((crosses>1e-8).all() or (crosses<-1e-8).all()):raise ValueError('Footprint must be strictly convex and ordered')
    side=edges[:,0]*(-p[:,1])-edges[:,1]*(-p[:,0])
    if not ((side>=-1e-8).all() or (side<=1e-8).all()):raise ValueError('Body origin must be inside footprint')
    # Reject self-intersecting star orderings, including those with same-sign turns.
    for i in range(len(p)):
        delta=p-p[i]
        signed=edges[i,0]*delta[:,1]-edges[i,1]*delta[:,0]
        if not ((signed>=-1e-8).all() or (signed<=1e-8).all()):raise ValueError('Footprint edges do not form a convex hull')
    return p.tolist()

def load(path):
    data=json.loads(Path(path).read_text())
    if data.get('schema_version')!=1:raise ValueError('Unsupported footprint profile')
    return checked(data['vertices'])
