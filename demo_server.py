#!/usr/bin/env python3
"""Synthetic geometry only: no ROS, hardware, motors or real recordings."""
import argparse,json,math,pathlib,threading
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import numpy as np
from nav_geometry import FloorMap
ROOT=pathlib.Path(__file__).resolve().parent

def make_scene():
    floor_map=FloorMap(size=100,resolution=.1,radius=.3)
    x,y=np.meshgrid(np.arange(-4,4.01,.1),np.arange(-3,3.01,.1))
    ground=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)])
    walls=[]
    for x0,y0,x1,y1 in [(-2.,-.5,-1.,.5),(1.,1.,2.,2.),(2.,-2.,3.,-1.)]:
        for z in np.arange(.2,1.9,.1):
            xx=np.arange(x0,x1+.05,.1);yy=np.arange(y0,y1+.05,.1)
            walls.extend([[a,y0,z] for a in xx]+[[a,y1,z] for a in xx]+[[x0,b,z] for b in yy]+[[x1,b,z] for b in yy])
    points=np.vstack([ground,np.array(walls)])
    floor_map.update(points);floor_map.update(points)
    free,_=floor_map.layers()
    status={'demo':True,'demo_notice':'合成几何交互演示；不连接机器人、不发送电机指令，也不录制现场数据。','robot':'Synthetic platform','sensor':'MID-360S','mount':'倒装示意','raw_age':0.05,'imu_age':0.005,'map_age':0.05,'odom_age':0.05,'counts':{'imu':0,'map_frames':0},'initial_tilt_deg':5.,'sensor_height_m':1.5,'sensor_xy':[0.,0.],'sensor_yaw':0.,'error':'','localization_fault':'','points':points[::2].round(3).tolist(),'free_cells':np.argwhere(free).tolist(),'obstacle_cells':np.argwhere(floor_map.obstacles).tolist(),'grid':{'resolution':floor_map.resolution,'origin':floor_map.origin,'origin_xy':floor_map.origin_xy.tolist(),'size':floor_map.size},'path':[],'goal':None,'path_state':'请选择合成地面目标','execution_enabled':False,'neck':{'state':'合成演示 · 固定安装示意'},'survey':{'active':False,'bag_ready':False,'session':None,'state':'合成演示不进行实物录包','duration_s':0,'travelled_m':0,'relative_to_start':None,'imu':{'body':{'age_s':None,'recent_samples':0,'gyro_plot':[]},'lidar':{'age_s':None,'recent_samples':0,'gyro_plot':[]}},'return_check':None,'issues':[],'trajectory':[],'global_map':{},'last_saved':None}}
    return floor_map,status

class DemoServer(ThreadingHTTPServer):
    daemon_threads=True
    def __init__(self,address):
        self.floor_map,self.state=make_scene();self.state_lock=threading.RLock()
        super().__init__(address,DemoHandler)

class DemoHandler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def reply(self,value,status=200):
        raw=json.dumps(value,ensure_ascii=False,allow_nan=False).encode()
        self.send_response(status);self.send_header('Content-Type','application/json; charset=utf-8');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
    def do_GET(self):
        if self.path=='/api/status':
            with self.server.state_lock:self.reply(self.server.state)
        elif self.path in ['/','/index.html']:
            raw=(ROOT/'web/index.html').read_bytes();self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
        else:self.reply({'error':'Not found'},404)
    def do_POST(self):
        if self.path!='/api/goal':self.reply({'error':'合成演示不进行真实录包或硬件操作'},400);return
        try:
            length=int(self.headers.get('Content-Length','0'))
            if length<0 or length>1024:raise ValueError('Invalid request size')
            data=json.loads(self.rfile.read(length));goal=[float(data['x']),float(data['y'])]
            if not all(math.isfinite(v) for v in goal):raise ValueError('Invalid goal')
            with self.server.state_lock:
                route=self.server.floor_map.plan([0.,0.],goal);self.server.state.update(path=route,goal=goal,path_state='合成网格路径有效')
            self.reply({'path':route,'goal':goal,'preview_start':[0.,0.],'start_gap_m':0.,'execution_enabled':False,'demo':True})
        except (ValueError,KeyError,TypeError) as e:self.reply({'error':str(e)},400)

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--port',type=int,default=8921);args=parser.parse_args()
    server=DemoServer(('127.0.0.1',args.port));print(f'Synthetic demo: http://127.0.0.1:{server.server_port}/ (no hardware)',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()
if __name__=='__main__':main()
