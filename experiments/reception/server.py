#!/usr/bin/env python3
"""Synthetic workflow sandbox. No ROS, robot connection, or actuator imports."""
import argparse
import copy
import heapq
import json
import math
from pathlib import Path
import secrets
import signal
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'navigation'))
from reception_catalog import Catalog
from reception_mission import MissionService
from reception_voice import VoiceCache
from reception_cloud import CloudStore
from reception_transfer import ContentTransfer
from session import NavigationSession

class Sandbox:
    def __init__(self, root):
        self.lock=threading.RLock();self.runtime=Path(root);self.map_name='synthetic_showroom'
        self.session=NavigationSession(authority_timeout=.25);self.request_token=secrets.token_urlsafe(24)
        self.reception_cloud=CloudStore(self.runtime/'cloud');self.reception_catalog=Catalog(self.runtime/'catalog');self.reception_voice=VoiceCache(self.runtime/'reception-voice-cache')
        self.reception_transfer=ContentTransfer(self.reception_catalog,self.reception_cloud)
        if not self.reception_catalog.read(self.map_name)['revision']:
            self.reception_catalog.save(json.loads((ROOT/'examples/reception_synthetic.json').read_text()),0)
        self.pose=[0.,0.,0.];self.path=[];self.active=False;self.operator='navigation_allowed';self.sequence=0
        self.speech=None;self.velocity=[0.,0.,0.];self.closed=threading.Event();self.commands=[]
        self.map=self.make_map();self.reception=MissionService(self,self.reception_catalog)
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def make_map(self):
        w,h,res=180,80,.1;origin=[-2.,-4.];cells=[]
        for y in range(h):
            for x in range(w):
                a=origin[0]+x*res;b=origin[1]+y*res
                wall=x in (0,w-1)or y in (0,h-1)or(5<a<7 and 1.5<b<2.2)or(10<a<11 and -2.8<b<-1.7)
                cells.append(100 if wall else 0)
        return {'width':w,'height':h,'resolution':res,'origin':origin,'cells':cells}
    def free(self,x,y):
        # Full cell rectangle of a .55 m square; no nine-sample corner shortcut.
        m=self.map;r=m['resolution']
        left=math.floor((x-.275-m['origin'][0])/r);right=math.floor((x+.275-m['origin'][0])/r)
        bottom=math.floor((y-.275-m['origin'][1])/r);top=math.floor((y+.275-m['origin'][1])/r)
        if left<0 or bottom<0 or right>=m['width']or top>=m['height']:return False
        return all(m['cells'][j*m['width']+i]==0 for j in range(bottom,top+1) for i in range(left,right+1))
    def plan(self,x,y):
        m=self.map;r=.2
        start=(round(self.pose[0]/r),round(self.pose[1]/r));goal=(round(x/r),round(y/r))
        heap=[(0,start)];best={start:0.};parent={};found=False
        while heap and len(best)<20000:
            _,p=heapq.heappop(heap)
            if p==goal:found=True;break
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                q=(p[0]+dx,p[1]+dy);cost=best[p]+math.hypot(dx,dy)
                if cost>=best.get(q,math.inf)or not self.free(q[0]*r,q[1]*r):continue
                if dx and dy and(not self.free(p[0]*r+dx*r,p[1]*r)or not self.free(p[0]*r,p[1]*r+dy*r)):continue
                best[q]=cost;parent[q]=p;heapq.heappush(heap,(cost+math.hypot(q[0]-goal[0],q[1]-goal[1]),q))
        if not found:raise ValueError('合成地图规划失败')
        path=[];p=goal
        while p!=start:path.append([p[0]*r,p[1]*r]);p=parent[p]
        path.reverse();return path or [[x,y]]
    def set_goal(self,data):
        with self.lock:
            g=self.session.prepare(data,time.monotonic());self.path=[];self.target=data
            start=self.pose[:]
            try:
                for point in data.get('_route_goals',[data]):
                    part=self.plan(point['x'],point['y']);self.path.extend(part)
                    self.pose=[point['x'],point['y'],0.]
            finally:self.pose=start
            self.session.planned(g,len(self.path),time.monotonic());return {'generation':g}
    def confirm_goal(self,data):
        with self.lock:
            if '_mission_epoch'in data and not self.reception.engine.valid(data['_mission_epoch']):raise ValueError('任务已取消')
            g=data['generation'];self.session.start(g,time.monotonic(),True,True)
            self.session.started(g,time.monotonic());self.active=True;self.commands.append({'model_only':True,'generation':g})
            return {'accepted':True,'hardware_execution_enabled':False}
    def cancel_reception_leg(self,epoch):
        if not self.reception.engine.valid(epoch):return False
        with self.lock:
            self.session.cancel();self.active=False;self.velocity=[0,0,0]
            self.session.acknowledge_cancel(self.session.cancel_generation)
        return True
    def stop(self):
        with self.lock:

            if self.reception.engine.phase not in ('paused','manual'):self.reception.engine.cancel()
            self.session.cancel();self.session.acknowledge_cancel(self.session.cancel_generation);self.active=False;self.velocity=[0.,0.,0.]
    def reception_speech_result(self):return self.speech
    def save(self,data):
        if set(data)!={'reception'}:raise ValueError('沙箱只接收导览内容')
        r=data['reception'];op=r.get('operation')
        if op=='versions':return self.reception_catalog.versions(r['map_id'])
        if op=='restore':return self.reception_catalog.restore(r['map_id'],r['source_revision'],r['expected_revision'])
        if op=='history':return self.reception.history.read(r.get('map_id')or self.map_name,r.get('before'),r.get('limit',100))
        if op=='stage':return self.reception_transfer.receive(r)
        if op=='read':
            d=self.reception_catalog.read(r.get('map_id')or self.map_name);return {'document':d,'voice':self.reception_voice.status(d)}
        if op=='cloud_read':return self.reception_cloud.read(r['map_id'])
        if op=='cloud_save':return self.reception_cloud.save(r['cloud'])
        if op=='save':return self.reception_catalog.save(r['document'],r['expected_revision'])
        if op=='prepare_voice':return self.reception_voice.prepare(self.reception_catalog.read(self.map_name))
        raise ValueError('未知配置请求')
    def status(self):
        return {'simulation':True,'scope':'Synthetic planar reception workflow only; not Nav2 or physical biped validation',
                'request_token':self.request_token,'robot_name':'P2 · 合成展厅','mode':'navigation','pose':self.pose[:],
                'map_revision':1,'maps':[self.map_name],'path':self.path,'session':self.session.status(time.monotonic()),
                'reception':{'map_id':self.map_name,'mission':self.reception.engine.status()},'hardware_control_requests':0,
                'model_start_requests':len(self.commands),'audio_feedback':{'enabled':True,'volume':55}}
    def run(self):
        last=time.monotonic()
        while not self.closed.wait(.02):
            now=time.monotonic();dt=min(now-last,.05);last=now;self.sequence+=1
            with self.lock:
                self.session.operator(self.operator,True,True,False,1,self.sequence,now,now)
                self.session.poll(now)
                if not self.session.status(now)['navigation_permitted']:
                    self.active=False;self.velocity=[0.,0.,0.]
                    # This synthetic adapter has no outstanding actuator/action.
                    self.session.acknowledge_cancel(self.session.cancel_generation)
                if self.active and self.path:
                    goal=self.path[0];dx,dy=goal[0]-self.pose[0],goal[1]-self.pose[1];distance=math.hypot(dx,dy);step=min(distance,.9*dt)
                    a=math.atan2(dy,dx);self.pose[2]=a
                    if self.free(self.pose[0]+step*math.cos(a),self.pose[1]+step*math.sin(a)):
                        self.pose[0]+=step*math.cos(a);self.pose[1]+=step*math.sin(a);self.velocity=[.9,0,0]
                    else:self.session.finished(self.session.generation,False);self.active=False;self.velocity=[0,0,0]
                    if distance<.06:self.path.pop(0)
                elif self.active:
                    if self.target['heading_mode']=='pose':self.pose[2]=self.target['yaw']
                    self.active=False;self.velocity=[0,0,0];self.session.finished(self.session.generation,True)
    def close(self):self.stop();self.closed.set();self.reception.close();self.thread.join(timeout=1)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--port',type=int,default=18941);ap.add_argument('--data-dir');args=ap.parse_args()
    tmp=tempfile.TemporaryDirectory()if not args.data_dir else None
    node=Sandbox(args.data_dir or tmp.name)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,data,status=200,kind='application/json'):
            raw=data if isinstance(data,bytes)else json.dumps(data,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
        def do_GET(self):
            path=urlsplit(self.path).path
            if path in ('/','/reception'):self.send((ROOT/'navigation/reception.html').read_bytes(),kind='text/html; charset=utf-8')
            elif path=='/api/status':self.send(node.status())
            elif path=='/api/map':self.send({'revision':1,'map':node.map})
            else:self.send({'error':'Not found'},404)
        def do_POST(self):
            try:
                if not secrets.compare_digest(self.headers.get('X-Nav-Token',''),node.request_token):return self.send({'error':'Invalid sandbox token'},403)
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=600000:raise ValueError('请求大小无效')
                d=json.loads(self.rfile.read(size))
                if self.path=='/api/save':result=node.save(d)
                elif self.path=='/api/confirm':result=node.reception.command(d['reception_command'])
                elif self.path=='/api/stop':node.stop();result={'stopped':True}
                elif self.path=='/api/sandbox/operator':node.operator=d['mode'];result={'simulation_only':True}
                else:return self.send({'error':'Not found'},404)
                self.send(result)
            except (ValueError,KeyError,TypeError)as e:self.send({'error':str(e)},409)
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler);server.timeout=.2
    quit=threading.Event()
    def stop(*_):quit.set()
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    print(json.dumps({'port':args.port,'pid':__import__('os').getpid(),'hardware_interfaces':0}),flush=True)
    try:
        while not quit.is_set():server.handle_request()
    finally:server.server_close();node.close();tmp.cleanup()if tmp else None
if __name__=='__main__':main()
