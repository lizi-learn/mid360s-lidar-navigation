"""Authored keepout lines and speed areas, separate from saved sensor maps."""
import copy
import math
import numpy as np
from reception_catalog import ident, number, text


def validate_annotations(rows):
    if not isinstance(rows, list) or len(rows) > 200:
        raise ValueError('地图标注最多200项')
    result=[];ids=set()
    for raw in rows:
        if not isinstance(raw,dict) or set(raw)-{'id','name','kind','vertices','width','factor'}:
            raise ValueError('地图标注字段无效')
        row=copy.deepcopy(raw);key=ident(row.get('id'))
        if key in ids:raise ValueError('地图标注标识重复')
        ids.add(key);row['name']=text(row.get('name',''),80)
        kind=row.get('kind');points=row.get('vertices')
        if kind not in ('keepout','speed') or not isinstance(points,list) or not 2<=len(points)<=64:
            raise ValueError('请选择禁行线或减速多边形')
        row['vertices']=[[number(p[0],-100000,100000),number(p[1],-100000,100000)]
                         for p in points if isinstance(p,list) and len(p)==2]
        if len(row['vertices'])!=len(points):raise ValueError('标注坐标无效')
        if any(math.dist(a,b)<.001 for a,b in zip(points,points[1:])):raise ValueError('相邻标注点重复')
        if kind=='keepout':
            row['width']=number(row.get('width',.10),.02,2.)
            if 'factor'in row:raise ValueError('禁行线不含速度倍率')
        else:
            if len(points)<3:raise ValueError('减速区至少需要三个顶点')
            area=abs(sum(a[0]*b[1]-b[0]*a[1]for a,b in zip(points,points[1:]+points[:1])))/2
            if area<.01:raise ValueError('减速区面积过小或顶点顺序无效')
            row['factor']=number(row.get('factor',.5),.1,1.)
            if 'width'in row:raise ValueError('减速区不含线宽')
        result.append(row)
    return result


def inside(x,y,vertices):
    """Vectorized even/odd polygon test; callers also include boundary distance."""
    hit=np.zeros(np.broadcast(x,y).shape,dtype=bool)
    for a,b in zip(vertices,vertices[1:]+vertices[:1]):
        if a[1]!=b[1]:hit^=((a[1]>y)!=(b[1]>y))&(x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0])
    return hit


def distance(x,y,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1];length=dx*dx+dy*dy
    f=np.clip(((x-a[0])*dx+(y-a[1])*dy)/max(length,1e-20),0,1)
    return np.hypot(x-a[0]-f*dx,y-a[1]-f*dy)


class AnnotationLayer:
    def __init__(self):self.key=None;self.indices=[]
    def apply(self,grid,rows):
        # Bound work to the annotations' map-clipped bounding boxes and cache
        # indices, so periodic state/door publications do not rerasterize maps.
        key=(grid['width'],grid['height'],grid['resolution'],tuple(grid['origin']),
             tuple((r['id'],r['kind'],tuple(map(tuple,r['vertices'])),r.get('width'))for r in rows))
        if key!=self.key:
            self.key=key;self.indices=[];res=grid['resolution'];ox,oy=grid['origin'];w,h=grid['width'],grid['height']
            for row in rows:
                if row['kind']!='keepout':continue
                pts=row['vertices'];margin=row['width']/2+res*math.sqrt(2)/2
                left=max(0,math.floor((min(p[0]for p in pts)-margin-ox)/res));right=min(w,math.ceil((max(p[0]for p in pts)+margin-ox)/res))
                bottom=max(0,math.floor((min(p[1]for p in pts)-margin-oy)/res));top=min(h,math.ceil((max(p[1]for p in pts)+margin-oy)/res))
                yy,xx=np.mgrid[bottom:top,left:right];x=ox+(xx+.5)*res;y=oy+(yy+.5)*res
                mask=np.zeros(x.shape,dtype=bool)
                for a,b in zip(pts,pts[1:]):mask|=distance(x,y,a,b)<=margin
                self.indices.extend((yy[mask]*w+xx[mask]).tolist())
        output=list(grid['cells'])
        for i in self.indices:output[i]=100
        return output,{'keepout_cells':len(set(self.indices)),'original_map_modified':False}

    @staticmethod
    def speed(rows,pose,footprint_radius=.389):
        if pose is None:return 1.,[]
        x,y=pose[:2];factor=1.;active=[]
        for row in rows:
            if row['kind']!='speed':continue
            pts=row['vertices']
            # Circumscribed footprint: reduce before any body corner enters.
            intersects=bool(inside(x,y,pts))or any(float(distance(x,y,a,b))<=footprint_radius for a,b in zip(pts,pts[1:]+pts[:1]))
            if intersects:factor=min(factor,row['factor']);active.append(row['id'])
        return factor,active


def nearby_candidates(grid,target,footprint,radius=1.,limit=12):
    """Grid-valid alternatives only. Each selected pose still needs Nav2 planning.

    No occupancy clearing, unknown promotion, body shrinking or silent goal move.
    """
    radius=number(radius,.1,3.);yaw=number(target.get('yaw',0),-math.pi,math.pi)
    x,y=(number(target[k],-100000,100000)for k in ('x','y'));res=grid['resolution']
    result=[]
    for ring in np.arange(max(.15,res*2),radius+res/2,max(.15,res*2)):
        for angle in np.linspace(0,2*math.pi,max(8,math.ceil(2*math.pi*ring/.15)),endpoint=False):
            px=x+ring*math.cos(angle);py=y+ring*math.sin(angle)
            if footprint_free(grid,px,py,yaw,footprint):
                if all(math.hypot(px-p['x'],py-p['y'])>=.20 for p in result):
                    result.append(dict(x=float(px),y=float(py),yaw=yaw,distance=float(ring),planner_verified=False))
                    if len(result)>=limit:return result
    return result


def footprint_free(grid,x,y,yaw,footprint):
    """Conservative filled-polygon/cell overlap, including all interior cells."""
    c,s=math.cos(yaw),math.sin(yaw);pts=[[x+c*a-s*b,y+s*a+c*b]for a,b in footprint]
    res=grid['resolution'];ox,oy=grid['origin'];w,h=grid['width'],grid['height']
    left=math.floor((min(p[0]for p in pts)-ox)/res);right=math.floor((max(p[0]for p in pts)-ox)/res)
    bottom=math.floor((min(p[1]for p in pts)-oy)/res);top=math.floor((max(p[1]for p in pts)-oy)/res)
    if left<0 or bottom<0 or right>=w or top>=h:return False
    yy,xx=np.mgrid[bottom:top+1,left:right+1];wx=ox+(xx+.5)*res;wy=oy+(yy+.5)*res
    mask=inside(wx,wy,pts)
    for a,b in zip(pts,pts[1:]+pts[:1]):mask|=distance(wx,wy,a,b)<=res*math.sqrt(2)/2
    cells=np.asarray(grid['cells']).reshape(h,w)[yy,xx]
    return bool(np.all((cells[mask]>=0)&(cells[mask]<20)))
