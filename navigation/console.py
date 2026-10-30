#!/usr/bin/env python3
"""Local workflow console: mapping, map save/load, pose, goals, speed and stop."""
import argparse, gzip, json, math, os, re, signal, subprocess, threading, time, secrets
from pathlib import Path
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,Twist
from nav_msgs.msg import OccupancyGrid,Path as RosPath
from nav2_msgs.action import NavigateToPose,ComputePathToPose
from nav2_msgs.msg import SpeedLimit
from slam_toolbox.srv import SerializePoseGraph
from nav2_msgs.srv import SaveMap
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener

ROOT=Path(__file__).resolve().parent
SAFE_NAME=re.compile(r'[A-Za-z0-9_-]{1,48}\Z')

class Console(Node):
    def __init__(self,runtime,simulation,minimum_speed,footprint=None,odometry_model='differential',gait_profile='generic'):
        super().__init__('navigation_workflow_console')
        self.robot_name=None;self.runtime=runtime;self.simulation=simulation;self.mode='idle';self.controller='rpp';self.process=None
        self.minimum_speed=minimum_speed;self.footprint=footprint;self.odometry_model=odometry_model;self.gait_profile=gait_profile
        self.maximum_speed=.15 if gait_profile=='p2-flat-model' else .30
        self.route_process=None;self.route_directory=None
        self.lock=threading.RLock();self.grid=None;self.path=[];self.gate={};self.truth={};self.goal=None
        self.last_action='等待选择建图或载入地图';self.goal_handle=None;self.feedback={}
        self.goal_generation=0
        self.request_token=secrets.token_urlsafe(24)
        self.speed_value=min(.20,self.maximum_speed)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,qos)
        self.create_subscription(RosPath,'/plan',lambda m:setattr(self,'path',[[p.pose.position.x,p.pose.position.y] for p in m.poses]),10)
        self.create_subscription(String,'/p2_nav/gate_status',lambda m:setattr(self,'gate',json.loads(m.data)),10)
        self.create_subscription(String,'/p2_nav/sim_truth',lambda m:setattr(self,'truth',json.loads(m.data)),10)
        self.operator=self.create_publisher(String,'/p2_nav/operator',10)
        self.limiter=self.create_publisher(SpeedLimit,'/speed_limit',10)
        self.initial=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.nav=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.planner=ActionClient(self,ComputePathToPose,'compute_path_to_pose')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.create_timer(.5,self.publish_speed)
    def on_map(self,msg):self.grid=msg
    def wait_future(self,future,timeout=15):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:time.sleep(.02)
        if not future.done():raise TimeoutError('服务响应超时')
        return future.result()
    def op(self,**values):
        m=String();m.data=json.dumps(values);self.operator.publish(m)
    def stop(self):
        self.goal_generation+=1
        self.op(armed=False,paused=True)
        if self.route_process is not None and self.route_process.poll() is None:
            os.killpg(self.route_process.pid,signal.SIGINT)
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async();self.goal_handle=None
        self.last_action='已停止，目标已取消'
    def stop_process(self):
        self.stop()
        if self.route_process is not None and self.route_process.poll() is None:
            try:self.route_process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(self.route_process.pid,signal.SIGTERM);self.route_process.wait(timeout=5)
        self.route_process=None
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGINT)
            try:self.process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid,signal.SIGTERM)
                try:self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(self.process.pid,signal.SIGKILL);self.process.wait()
        self.process=None;self.mode='idle'
    def start(self,mode,name,controller):
        if mode not in ('mapping','navigation') or controller not in ('rpp','mppi'):raise ValueError('模式或控制器无效')
        if not SAFE_NAME.fullmatch(name):raise ValueError('地图名仅允许字母、数字、下划线和连字符')
        map_file=self.runtime/'maps'/name/'mapped.yaml'
        if mode=='navigation' and not map_file.is_file():raise ValueError('请先保存该地图')
        with self.lock:
            self.stop_process()
            self.grid=None;self.gate={};self.buffer.clear()
            from configure import config
            import yaml
            params=self.runtime/'nav2.yaml';params.write_text(yaml.safe_dump(config(controller,minimum_speed=self.minimum_speed,footprint_vertices=self.footprint,odometry_model=self.odometry_model,gait_profile=self.gait_profile),sort_keys=False))
            args=['bash',str(ROOT/'run.sh'),'mode:='+mode,'simulation:='+str(self.simulation).lower(),'params:='+str(params)]
            if mode=='navigation':args+=['map:='+str(map_file)]
            self.process=subprocess.Popen(args,cwd=ROOT,stdout=open(self.runtime/'workflow.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
            self.mode=mode;self.controller=controller;self.path=[];self.goal=None
            self.last_action='正在启动建图' if mode=='mapping' else '正在载入地图和定位'
        return {'mode':mode,'pid':self.process.pid,'simulation':self.simulation}
    def simulated_mapping_route(self,name):
        if not self.simulation or self.mode!='mapping':raise ValueError('仅在仿真建图模式可运行自动路线')
        if not SAFE_NAME.fullmatch(name):raise ValueError('地图名无效')
        if self.route_process is not None and self.route_process.poll() is None:raise ValueError('仿真路线仍在进行')
        target=self.runtime/'maps'/name
        if (target/'mapped.yaml').exists():raise ValueError('该地图已存在，请换一个新地图名')
        target.mkdir(parents=True,exist_ok=True);self.route_directory=target
        self.route_process=subprocess.Popen(['/usr/bin/python3','-S',str(ROOT/'benchmark.py'),'mapping','--directory',str(target)],
            cwd=ROOT,stdout=open(self.runtime/'mapping-route.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
        self.last_action='仿真正在按前向路线建图，结束后自动保存'
        return {'route_started':True,'simulation':True}
    def save_map(self,name):
        if self.mode!='mapping':raise ValueError('当前不是建图模式')
        if not SAFE_NAME.fullmatch(name):raise ValueError('地图名无效')
        target=self.runtime/'maps'/name;target.mkdir(parents=True,exist_ok=True)
        stem=str(target/'mapped')
        for service,typ in [('/map_saver/save_map',SaveMap),('/slam_toolbox/serialize_map',SerializePoseGraph)]:
            client=self.create_client(typ,service)
            if not client.wait_for_service(timeout_sec=5):raise ValueError('建图服务尚未就绪')
            req=typ.Request()
            if typ is SaveMap:req.map_topic='/map';req.map_url=stem;req.image_format='pgm';req.map_mode='trinary';req.free_thresh=.25;req.occupied_thresh=.65
            else:req.filename=stem
            response=self.wait_future(client.call_async(req),25)
            if not (response.result if typ is SaveMap else response.result==0):raise ValueError('地图保存失败')
            self.destroy_client(client)
        self.last_action='地图与位姿图已保存：'+name
        return {'saved':name,'files':[p.name for p in target.iterdir()]}
    def checked_pose(self,data):
        x,y,yaw=(float(data[k]) for k in ('x','y','yaw'))
        if not all(math.isfinite(v) for v in (x,y,yaw)):raise ValueError('坐标必须是有限数值')
        grid=self.grid
        if grid is None:raise ValueError('地图尚未收到')
        ix=int(math.floor((x-grid.info.origin.position.x)/grid.info.resolution))
        iy=int(math.floor((y-grid.info.origin.position.y)/grid.info.resolution))
        if not 0<=ix<grid.info.width or not 0<=iy<grid.info.height:raise ValueError('目标在地图外')
        cell=grid.data[iy*grid.info.width+ix]
        if not 0<=cell<20:raise ValueError('该点为未知区域或障碍，请选择已观察地面')
        pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=self.get_clock().now().to_msg()
        pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
        return pose
    def set_pose(self,data):
        self.stop();pose=self.checked_pose(data);msg=PoseWithCovarianceStamped();msg.header=pose.header;msg.pose.pose=pose.pose
        msg.pose.covariance[0]=.04;msg.pose.covariance[7]=.04;msg.pose.covariance[35]=.04
        self.initial.publish(msg);self.last_action='已提交初始位置，等待定位稳定'
        return {'initial_pose_set':True}
    def set_goal(self,data):
        if self.mode!='navigation':raise ValueError('请先载入地图进入导航模式')
        pose=self.checked_pose(data)
        if not self.gate.get('localized'):raise ValueError('定位尚未就绪，请先设置初始位置')
        from lifecycle_msgs.srv import GetState
        for name in (['planner_server','controller_server','bt_navigator','velocity_smoother','collision_monitor'] if self.simulation else ['planner_server']):
            client=self.create_client(GetState,'/'+name+'/get_state')
            try:
                if not client.wait_for_service(timeout_sec=.2) or self.wait_future(client.call_async(GetState.Request()),2).current_state.id!=3:raise ValueError('导航服务正在初始化，请等待就绪')
            finally:self.destroy_client(client)
        if not self.simulation:
            if not self.planner.wait_for_server(timeout_sec=3):raise ValueError('规划服务尚未就绪')
            request=ComputePathToPose.Goal();request.goal=pose;request.planner_id='GridBased'
            handle=self.wait_future(self.planner.send_goal_async(request),5)
            if not handle.accepted:raise ValueError('规划请求被拒绝')
            result=self.wait_future(handle.get_result_async(),10)
            if result.status!=4:raise ValueError('未找到可用路径')
            self.path=[[p.pose.position.x,p.pose.position.y] for p in result.result.path.poses]
            self.goal=[data['x'],data['y']];self.last_action='路径预演已生成，不驱动电机'
            return {'planned':True,'path_points':len(self.path),'hardware_execution_enabled':False}
        if not self.nav.wait_for_server(timeout_sec=3):raise ValueError('导航服务尚未就绪')
        self.stop();generation=self.goal_generation
        self.op(armed=True,paused=False,speed=self.speed_value);self.publish_speed()
        goal=NavigateToPose.Goal();goal.pose=pose
        handle=self.wait_future(self.nav.send_goal_async(goal,feedback_callback=self.on_feedback),5)
        if not handle.accepted:self.op(armed=False);raise ValueError('导航目标被拒绝')
        if generation!=self.goal_generation:
            handle.cancel_goal_async();raise ValueError('该目标已被停止或后续请求取代')
        self.goal_handle=handle;self.goal=[data['x'],data['y']];self.last_action='正在导航' if self.simulation else '正在计算实机导航预演（不驱动电机）'
        handle.get_result_async().add_done_callback(lambda f:self.finished(f,generation))
        return {'accepted':True,'hardware_execution_enabled':False}
    def on_feedback(self,feedback):
        f=feedback.feedback;self.feedback={'distance_remaining':f.distance_remaining,'recoveries':f.number_of_recoveries}
    def finished(self,future,generation):
        if generation!=self.goal_generation:return
        self.op(armed=False);status=future.result().status
        self.last_action={4:'已到达目标',5:'目标已取消',6:'导航失败，已停止'}.get(status,'导航已结束')
        self.goal_handle=None
    def speed(self,value):
        v=float(value)
        if not math.isfinite(v) or not self.minimum_speed<=v<=self.maximum_speed:raise ValueError(f'此步态配置的速度范围为 {self.minimum_speed:.2f}–{self.maximum_speed:.2f} m/s')
        self.speed_value=v;self.publish_speed()
        return {'speed_m_s':v}
    def publish_speed(self):
        self.op(speed=self.speed_value)
        limit=SpeedLimit();limit.percentage=False;limit.speed_limit=self.speed_value;self.limiter.publish(limit)
    def status(self):
        if self.route_process is not None and self.route_process.poll() is not None:
            report=self.route_directory/'mapping-report.json'
            good=report.exists() and 'error' not in json.loads(report.read_text())
            self.last_action='仿真建图路线已结束，地图已保存' if good else '仿真路线已停止，请检查地图状态'
            self.route_process=None
        pose=None
        try:
            t=self.buffer.lookup_transform('map','base_link',rclpy.time.Time());q=t.transform.rotation
            pose=[t.transform.translation.x,t.transform.translation.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z)]
        except Exception:pass
        grid=self.grid
        return {'robot_name':self.robot_name,'gait_profile':self.gait_profile,'request_token':self.request_token,'mode':self.mode,'simulation':self.simulation,'controller':self.controller,'message':self.last_action,
            'minimum_speed':self.minimum_speed,'maximum_speed':self.maximum_speed,'simulation_kind':'native_dynamics' if os.getenv('P2_NAV_NATIVE_EXEC') else 'planar',
            'process_alive':self.process is not None and self.process.poll() is None,'gate':self.gate,'pose':pose,
            'map':None if grid is None else {'width':grid.info.width,'height':grid.info.height,'resolution':grid.info.resolution,
                'origin':[grid.info.origin.position.x,grid.info.origin.position.y],'cells':list(grid.data)},
            'path':self.path,'goal':self.goal,'feedback':self.feedback,
            'maps':sorted(p.parent.name for p in (self.runtime/'maps').glob('*/mapped.yaml')),
            'hardware_execution_enabled':False,
            'scope':'平面导航仿真' if self.simulation else '实机导航预演；机身外参、整机零位和步态接入待验收'}

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--simulation',action='store_true');parser.add_argument('--port',type=int,default=8930)
    parser.add_argument('--minimum-speed',type=float,default=.03)
    parser.add_argument('--footprint-profile',default='')
    parser.add_argument('--odometry-model',choices=['differential','omni'],default='differential')
    parser.add_argument('--gait-handoff',action='store_true');parser.add_argument('--robot-profile',default='')
    parser.add_argument('--gait-profile',choices=['generic','p2-flat-model'],default='generic')
    parser.add_argument('--runtime',default='');args=parser.parse_args()
    if args.gait_handoff or args.gait_profile=='p2-flat-model':os.environ['P2_NAV_GAIT_HANDOFF']='true'
    selected=args.robot_profile or os.getenv('P2_NAV_ROBOT_PROFILE','')
    if not selected and not args.simulation and (ROOT.parent/'config/robot.local.json').is_file():selected=str(ROOT.parent/'config/robot.local.json')
    if selected and args.simulation:raise ValueError('Physical robot profiles are not used by simulation; choose an independent runtime')
    if selected:
        import sys
        sys.path.insert(0,str(ROOT.parent))
        from robot_profile import load,check_machine,bind_runtime,workflow_paths
        profile=load(selected);check_machine(profile)
        if os.getenv('P2_NAV_BODY_CALIBRATION'):
            from robot_profile import check_calibration
            check_calibration(profile,json.loads(Path(os.environ['P2_NAV_BODY_CALIBRATION']).read_text()))
        sensor_runtime,workflow_runtime=workflow_paths(profile,ROOT.parent/'runtime/robots',os.getenv('P2_NAV_RUNTIME',''),args.runtime)
        bind_runtime(profile,sensor_runtime)
        runtime=bind_runtime(profile,workflow_runtime)
        os.environ['P2_NAV_ROBOT_PROFILE']=str(Path(selected).resolve())
        os.environ.setdefault('P2_LIDAR_ALIGNMENT',str(sensor_runtime/'map_alignment.json'))
    else:
        runtime=Path(args.runtime or ROOT.parent/('runtime/simulation/workflow' if args.simulation else 'runtime/workflow')).resolve()
        if args.simulation:
            import sys
            sys.path.insert(0,str(ROOT.parent))
            from robot_profile import check_simulation_runtime
            check_simulation_runtime(runtime)
    runtime.mkdir(parents=True,exist_ok=True);(runtime/'maps').mkdir(exist_ok=True)
    if not .03<=args.minimum_speed<=(.15 if args.gait_profile=='p2-flat-model' else .25):raise ValueError('Invalid minimum speed profile')
    from footprint import load as load_footprint
    vertices=load_footprint(args.footprint_profile) if args.footprint_profile else None
    if selected and args.footprint_profile:
        data=json.loads(Path(args.footprint_profile).read_text())
        if data.get('hardware_verified') is True:
            from robot_profile import check_calibration
            check_calibration(profile,data)
    rclpy.init();node=Console(runtime,args.simulation,args.minimum_speed,vertices,args.odometry_model,args.gait_profile);node.robot_name=profile['name'] if selected else None;executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*a):pass
        def send(self,body,status=200,content_type='application/json'):
            raw=body if isinstance(body,bytes) else json.dumps(body,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(status);self.send_header('Content-Type',content_type);self.send_header('Cache-Control','no-store')
            if 'gzip' in self.headers.get('Accept-Encoding','') and len(raw)>1024:
                raw=gzip.compress(raw);self.send_header('Content-Encoding','gzip')
            self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
        def do_GET(self):
            if self.path=='/':self.send((ROOT/'console.html').read_bytes(),content_type='text/html; charset=utf-8')
            elif self.path=='/api/status':self.send(node.status())
            else:self.send({'error':'Not found'},404)
        def do_POST(self):
            try:
                if not secrets.compare_digest(self.headers.get('X-Nav-Token',''),node.request_token):
                    self.send({'error':'请刷新工作台后重试'},403);return
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=8192:raise ValueError('请求大小无效')
                d=json.loads(self.rfile.read(length))
                routes={'/api/start':lambda:node.start(d['mode'],d.get('name','office'),d.get('controller','rpp')),
                    '/api/sim_mapping_route':lambda:node.simulated_mapping_route(d['name']),
                    '/api/save':lambda:node.save_map(d['name']),'/api/goal':lambda:node.set_goal(d),
                    '/api/initialpose':lambda:node.set_pose(d),'/api/speed':lambda:node.speed(d['speed']),
                    '/api/stop':lambda:(node.stop() or {'stopped':True})}
                if self.path not in routes:self.send({'error':'Not found'},404);return
                self.send(routes[self.path]())
            except (ValueError,KeyError,TypeError,TimeoutError) as e:self.send({'error':str(e)},409)
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    stopping=threading.Event();server.timeout=.5
    def stop_signal(signum,frame):stopping.set()
    signal.signal(signal.SIGTERM,stop_signal);signal.signal(signal.SIGINT,stop_signal)
    try:
        while not stopping.is_set():server.handle_request()
    finally:
        node.stop_process();server.server_close();executor.shutdown();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
