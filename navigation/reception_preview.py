"""Bounded, read-only route/task preview. No ROS, network or motor interfaces.

This is a geometric/task storyboard, not a gait or live Nav2 validation. A
circumscribed square is used for this preview's search only; production robot
geometry, planning, timing, and control settings are never changed.
"""
import heapq
import math
import time
import threading
import numpy as np
from scipy.ndimage import distance_transform_edt
from reception_catalog import validate
from reception_annotations import AnnotationLayer

_preview_lock=threading.Lock()


def preview_isolated(grid,document,route_id):
    """Keep the CPU-bound preview out of the live ROS interpreter's GIL."""
    import json,os,subprocess,sys,tempfile
    from pathlib import Path
    if not _preview_lock.acquire(blocking=False):raise ValueError('已有预演正在计算，请稍后再试')
    try:
        with tempfile.TemporaryDirectory(prefix='p2-route-preview-')as tmp:
            source=Path(tmp)/'input.json';result=Path(tmp)/'result.json'
            source.write_text(json.dumps(dict(grid=grid,document=document,route_id=route_id)))
            env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
            subprocess.run([sys.executable,str(Path(__file__).resolve()),str(source),str(result)],env=env,
                           check=True,timeout=15,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
            response=json.loads(result.read_text())
            if 'error'in response:raise ValueError(response['error'])
            return response
    except subprocess.TimeoutExpired as e:raise ValueError('本次预演计算超时，请缩短路线')from e
    finally:_preview_lock.release()


def preview(grid,document,route_id):
    document=validate(document)
    if grid.get('map_id',document['map_id'])!=document['map_id']:raise ValueError('预演地图与点位不一致')
    route=next((r for r in document['routes']if r['id']==route_id),None)
    if not route:raise ValueError('请选择要预演的路线')
    if not route['steps']:raise ValueError('这条路线还没有点位，请先加入点位再预演')
    points={p['id']:p for p in document['points']}
    if len(route['steps'])*route['loops']>150:raise ValueError('一次预演最多150站，可减少循环次数')
    cells,annotation_report=AnnotationLayer().apply(grid,document['annotations'])
    cells=np.asarray(cells).reshape(grid['height'],grid['width']);res=grid['resolution'];ox,oy=grid['origin']
    # Include the complete occupied cell rectangle in the distance bound.
    free=(cells>=0)&(cells<20);free[[0,-1],:]=False;free[:,[0,-1]]=False
    clearance=distance_transform_edt(free)*res
    valid=clearance>(math.sqrt(2)*.275+res*math.sqrt(2)/2)
    deadline=time.monotonic()+10.;expanded=0
    def cell(p):return (math.floor((p[1]-oy)/res),math.floor((p[0]-ox)/res))
    def world(q):return [ox+(q[1]+.5)*res,oy+(q[0]+.5)*res]
    def available(q):return 0<=q[0]<grid['height']and 0<=q[1]<grid['width']and valid[q]
    def visible(a,b):
        n=max(1,math.ceil(math.dist(a,b)/res*2))
        return all(available(cell([a[j]+(b[j]-a[j])*i/n for j in (0,1)]))for i in range(n+1))
    def path(a,b):
        nonlocal expanded
        start,goal=cell(a),cell(b)
        if not available(start)or not available(goal):raise ValueError('点位附近预演净空不足；可在地图微调点位。此预演不修改真实导航')
        queue=[(0.,0.,start)];best={start:0.};parent={};found=False
        while queue:
            _,cost,q=heapq.heappop(queue)
            if cost!=best[q]:continue
            if q==goal:found=True;break
            expanded+=1
            if expanded%512==0 and(time.monotonic()>deadline or expanded>180000):raise ValueError('预演计算已到本轮上限，请缩短路线')
            for dy,dx in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                n=(q[0]+dy,q[1]+dx)
                if not available(n)or(dx and dy and(not available((q[0]+dy,q[1]))or not available((q[0],q[1]+dx)))):continue
                new=cost+math.hypot(dx,dy)
                if new>=best.get(n,math.inf):continue
                best[n]=new;parent[n]=q;heapq.heappush(queue,(new+math.dist(n,goal),new,n))
        if not found:raise ValueError('这段预演没有找到路径，请检查禁行线与点位；未改变真实导航状态')
        seq=[goal]
        while seq[-1]!=start:seq.append(parent[seq[-1]])
        seq.reverse();raw=[a]+[world(q)for q in seq[1:-1]]+[b];result=[raw[0]];anchor=0
        while anchor<len(raw)-1:
            k=min(len(raw)-1,anchor+80)
            while k>anchor+1 and not visible(raw[anchor],raw[k]):k-=1
            result.append(raw[k]);anchor=k
        return result
    samples=[];events=[];elapsed=0.;length=0.
    first=points[route['steps'][0]['point_id']]
    home=next((p for p in document['points']if p['kind']=='home'),first)
    pose=[home['x'],home['y'],home['yaw']or 0.]
    def sample(phase,index,point,script='',factor=1.):
        samples.append(dict(t=round(elapsed,3),pose=[round(v,4)for v in pose],phase=phase,index=index,
                            point=point['name'],script=script,factor=factor))
    def rotate(yaw,index,p):
        nonlocal elapsed
        delta=math.remainder(yaw-pose[2],2*math.pi);start=pose[2];n=max(1,math.ceil(abs(delta)/.6*5))
        for i in range(1,n+1):
            elapsed+=abs(delta)/.6/n;pose[2]=math.remainder(start+delta*i/n,2*math.pi);sample('turning',index,p)
    steps=route['steps']*route['loops']
    for index,step in enumerate(steps):
        p=points[step['point_id']];target=[p['x'],p['y']]
        try:line=path(pose[:2],target)
        except ValueError as e:raise ValueError(f'第{index+1}站「{p["name"]}」：{e}')from e
        for a,b in zip(line,line[1:]):
            distance=math.dist(a,b)
            if distance<1e-6:continue
            rotate(math.atan2(b[1]-a[1],b[0]-a[0]),index,p)
            n=max(1,math.ceil(distance/.07));length+=distance
            for i in range(1,n+1):
                pose[:2]=[a[j]+(b[j]-a[j])*i/n for j in (0,1)]
                factor,_=AnnotationLayer.speed(document['annotations'],pose)
                elapsed+=distance/n/(.4*factor);sample('moving',index,p,factor=factor)
        if p['yaw']is not None:rotate(p['yaw'],index,p)
        script=(p['script']or next((q['script']for q in document['products']if q['id']==p['product_id']),''))if step['speak']else''
        events.append(dict(t=round(elapsed,3),kind='arrived',index=index,point=p['name']))
        if script:
            duration=max(2.,len(script)/5.)
            sample('presenting',index,p,script);elapsed+=duration;sample('presenting',index,p,script)
        if p['guest_wait']:
            sample('guest_wait',index,p);elapsed+=3.;sample('guest_wait',index,p)
            events.append(dict(t=round(elapsed,3),kind='simulated_continue',index=index,point=p['name']))
        if p['dwell']:
            sample('dwell',index,p);elapsed+=p['dwell'];sample('dwell',index,p)
        sample('arrived',index,p)
    sample('completed',len(steps)-1,points[steps[-1]['point_id']])
    return dict(schema=1,map_id=document['map_id'],revision=document['revision'],route_id=route_id,
                route_name=route['name'],scope='saved_map_geometric_task_preview_not_gait_or_live_nav2',
                hardware_commands=0,saved_map_sha256=grid.get('image_sha256'),body_m=.55,
                model_speed_mps=.4,model_turn_radps=.6,guest_continue_after_s=3.,
                duration_s=round(elapsed,3),length_m=round(length,2),samples=samples,events=events,
                annotations=annotation_report,expanded=expanded)


if __name__=='__main__':
    import json,os,sys
    from pathlib import Path
    try:os.nice(10)
    except OSError:pass
    payload=json.loads(Path(sys.argv[1]).read_text())
    try:response=preview(**payload)
    except ValueError as e:response={'error':str(e)}
    Path(sys.argv[2]).write_text(json.dumps(response,ensure_ascii=False))
