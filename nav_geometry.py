"""Gravity alignment and conservative, observed-floor path previews."""
import heapq
import math
from collections import deque
import numpy as np
from scipy.ndimage import binary_dilation, binary_closing

def level_rotation(acceleration):
    a = np.asarray(acceleration, dtype=float)
    if not np.all(np.isfinite(a)) or np.linalg.norm(a) < .5:
        raise ValueError('Invalid gravity observation')
    flip = np.diag([1., -1., -1.])
    u = flip @ (a / np.linalg.norm(a))
    z = np.array([0., 0., 1.])
    v = np.cross(u, z)
    c = float(u @ z)
    if c < -.99:
        raise ValueError('Expected inverted mounting; verify installation')
    k = np.array([[0., -v[2], v[1]], [v[2], 0., -v[0]], [-v[1], v[0], 0.]])
    return (np.eye(3) + k + k @ k / (1 + c)) @ flip

def estimate_floor(points):
    p = np.asarray(points)
    radius = np.linalg.norm(p[:, :2], axis=1)
    p = p[(radius > .7) & (radius < 7) & (p[:, 2] < -.4) & (p[:, 2] > -2.5)]
    if len(p) < 80:
        raise ValueError('Insufficient visible floor')
    bins = np.arange(-2.5, -.3, .025)
    counts, edges = np.histogram(p[:, 2], bins)
    mode = edges[np.argmax(counts)] + .0125
    selected = p[np.abs(p[:, 2] - mode) < .05]
    if len(selected) < 60 or np.ptp(selected[:, 0]) < 1 or np.ptp(selected[:, 1]) < 1:
        raise ValueError('Floor observation too narrow')
    return float(np.median(selected[:, 2]))

class FloorMap:
    def __init__(self, size=300, resolution=.1, radius=.4):
        self.size, self.resolution, self.radius = size, resolution, radius
        self.origin = -size * resolution / 2
        self.origin_xy = np.array([self.origin,self.origin])
        self.floor = np.zeros((size, size), dtype=bool)
        self.obstacles = np.zeros_like(self.floor)
        self.frames = deque(maxlen=20)
        self.floor_hits = np.zeros((size, size), dtype=np.uint16)
        self.obstacle_hits = np.zeros_like(self.floor_hits)

    def cell(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        col, row = math.floor((x-self.origin_xy[0])/self.resolution), math.floor((y-self.origin_xy[1])/self.resolution)
        return (row,col) if 0 <= row < self.size and 0 <= col < self.size else None

    def world(self, cell):
        row,col=cell
        return [float(self.origin_xy[0]+(col+.5)*self.resolution),float(self.origin_xy[1]+(row+.5)*self.resolution)]

    def recenter(self,x,y):
        center=self.origin_xy+self.size*self.resolution/2
        if np.max(np.abs(np.array([x,y])-center))<3:return
        new=np.floor((np.array([x,y])-self.size*self.resolution/2)/self.resolution)*self.resolution
        dc,dr=np.rint((self.origin_xy-new)/self.resolution).astype(int)
        def shift(a):
            b=np.roll(a,(int(dr),int(dc)),axis=(0,1))
            if abs(dr)>=self.size or abs(dc)>=self.size:b[:]=0;return b
            if dr>0:b[:dr,:]=0
            elif dr<0:b[dr:,:]=0
            if dc>0:b[:,:dc]=0
            elif dc<0:b[:,dc:]=0
            return b
        self.frames=deque(((shift(a),shift(b)) for a,b in self.frames),maxlen=self.frames.maxlen)
        self.floor_hits=shift(self.floor_hits);self.obstacle_hits=shift(self.obstacle_hits)
        self.floor=shift(self.floor);self.obstacles=shift(self.obstacles);self.origin_xy=new

    def update(self, points):
        p = np.asarray(points)
        p = p[np.all(np.isfinite(p), axis=1)]
        cells = np.floor((p[:, :2] - self.origin_xy)/self.resolution).astype(int)
        inside = (cells >= 0).all(axis=1) & (cells < self.size).all(axis=1)
        cells, p = cells[inside], p[inside]
        floor = np.abs(p[:,2]) < .09
        obstacle = (p[:,2] > .15) & (p[:,2] < 2.1)
        floor_frame=np.zeros_like(self.floor);obstacle_frame=np.zeros_like(self.floor)
        floor_frame[cells[floor,1],cells[floor,0]] = True
        obstacle_frame[cells[obstacle,1],cells[obstacle,0]] = True
        if len(self.frames)==self.frames.maxlen:
            old_floor,old_obstacles=self.frames.popleft()
            self.floor_hits-=old_floor;self.obstacle_hits-=old_obstacles
        self.frames.append((floor_frame,obstacle_frame))
        self.floor_hits+=floor_frame;self.obstacle_hits+=obstacle_frame
        # A short window prevents single noisy returns and old localization
        # drift from permanently filling the local map with obstacles.
        self.floor[:]=self.floor_hits>=2;self.obstacles[:]=self.obstacle_hits>=2

    def layers(self):
        # Close only sub-cell sampling holes; unknown space stays blocked.
        observed = binary_closing(self.floor, iterations=1)
        n = math.ceil(self.radius/self.resolution)
        y,x = np.ogrid[-n:n+1,-n:n+1]
        disk = x*x+y*y <= (self.radius/self.resolution)**2
        inflated = binary_dilation(self.obstacles, structure=disk)
        free = observed & ~inflated
        occupancy = np.full(self.floor.shape, -1, dtype=np.int8)
        occupancy[observed] = 0
        occupancy[self.obstacles] = 100
        return free, occupancy

    def plan(self, start, goal):
        free, _ = self.layers()
        s, g = self.cell(*start), self.cell(*goal)
        if s is None or g is None:
            raise ValueError('目标超出当前地图范围')
        if not free[g]:
            raise ValueError('目标位于障碍、未知地面或距离障碍过近')
        if not free[s]:
            # A small observation gap under the fixed robot is allowed only for
            # selecting a nearby, observed start, never for opening a corridor.
            candidates=np.argwhere(free)
            if not len(candidates):raise ValueError('尚未观察到可通行地面')
            ds=np.linalg.norm(candidates-np.array(s),axis=1)
            if ds.min()*self.resolution > .65:raise ValueError('雷达下方尚无连通的已知地面')
            s=tuple(candidates[ds.argmin()])
        queue=[(0.,s)];cost={s:0.};parent={}
        moves=[(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
        while queue:
            _,current=heapq.heappop(queue)
            if current==g:break
            for dr,dc in moves:
                nxt=(current[0]+dr,current[1]+dc)
                if not (0<=nxt[0]<self.size and 0<=nxt[1]<self.size and free[nxt]):continue
                if dr and dc and not (free[current[0]+dr,current[1]] and free[current[0],current[1]+dc]):continue
                candidate=cost[current]+math.hypot(dr,dc)
                if candidate<cost.get(nxt,float('inf')):
                    cost[nxt]=candidate;parent[nxt]=current
                    heapq.heappush(queue,(candidate+math.hypot(nxt[0]-g[0],nxt[1]-g[1]),nxt))
        if g not in cost:raise ValueError('已知地面内没有连通路径')
        cells=[g]
        while cells[-1]!=s:cells.append(parent[cells[-1]])
        return [self.world((r,c)) for r,c in reversed(cells)]
