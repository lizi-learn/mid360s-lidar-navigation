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
from nav2_msgs.msg import SpeedLimit,CollisionMonitorState
from slam_toolbox.srv import SerializePoseGraph
from nav2_msgs.srv import SaveMap
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from map_persistence import configure_save_request,validate_map_metadata
from session import NavigationSession
from urllib.parse import urlsplit,parse_qs

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
        self.goal_generation=0;self.session=NavigationSession();self.map_revision=0
        self.plan_handle=None;self.pending_nav_accept=False;self.pending_plan_accept=False;self.cancel_sent_handles=[];self.action_results={};self.accepted_actions={}
        self.collision_stop=False;self.path_revision=0;self.obstacle_path_revision=0
        self.virtual_operator=None;self.virtual_sequence=0;self.virtual_source=secrets.randbits(63)+1
        self.request_token=secrets.token_urlsafe(24)
        self.speed_value=min(.20,self.maximum_speed)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,qos)
        self.create_subscription(RosPath,'/plan',self.on_path,10)
        self.create_subscription(CollisionMonitorState,'/p2_nav/collision_state',self.on_collision,10)
        self.create_subscription(String,'/p2_nav/operator_authority',self.on_authority,10)
        self.create_subscription(String,'/p2_nav/gate_status',lambda m:setattr(self,'gate',json.loads(m.data)),10)
        self.create_subscription(String,'/p2_nav/sim_truth',lambda m:setattr(self,'truth',json.loads(m.data)),10)
        self.operator=self.create_publisher(String,'/p2_nav/operator',10)
        self.limiter=self.create_publisher(SpeedLimit,'/speed_limit',10)
        self.initial=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.nav=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.planner=ActionClient(self,ComputePathToPose,'compute_path_to_pose')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.create_timer(.5,self.publish_speed)
        self.create_timer(.02,self.poll_session)
    def on_map(self,msg):self.grid=msg;self.map_revision+=1
    def on_authority(self,msg):
        with self.lock:
            try:
                d=json.loads(msg.data)
                self.session.operator(d['switch_mode'],d['switch_mapping_verified'],d['receiver_connected'],
                    d['manual_activity'],d['source_instance'],d['source_sequence'],d['source_monotonic_ns']/1e9,time.monotonic())
            except (KeyError,TypeError,ValueError):
                self.session.operator('unknown',False,False,False,0,0,0.,time.monotonic())
    def simulation_operator(self,data):
        if not self.simulation:raise ValueError('真实机器人不接受网页伪造保护开关状态')
        mode=data['mode']
        if mode not in ('navigation_allowed','manual','paused'):raise ValueError('模拟档位无效')
        self.virtual_operator={'mode':mode,'connected':data.get('connected',True) is True,'activity':data.get('manual_activity',False) is True}
        return {'simulation_only':True,'mode':mode}
    def on_path(self,msg):
        with self.lock:
            self.path_revision+=1
            if self.session.phase in ('executing','replanning'):
                self.path=[[p.pose.position.x,p.pose.position.y] for p in msg.poses]
                age=(self.get_clock().now().nanoseconds/1e9)-(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
                fresh=-.05<=age<=.5 and self.path_revision>self.obstacle_path_revision and len(self.path)>1
                self.session.reroute(time.monotonic(),fresh,not self.collision_stop and self.health_ready())
    def on_collision(self,msg):
        with self.lock:
            self.collision_stop=msg.action_type==CollisionMonitorState.STOP
            if self.collision_stop and self.session.phase=='executing':
                self.obstacle_path_revision=self.path_revision;self.session.obstacle(time.monotonic())
    def health_ready(self):
        g=self.gate
        return bool(g.get('localized') and g.get('ground_ok') and
            -.05<=g.get('scan_age',999)<=.5 and -.05<=g.get('odom_age',999)<=.5)
    def poll_session(self):
        with self.lock:
            now=time.monotonic()
            if self.simulation and self.virtual_operator:
                v=self.virtual_operator;self.virtual_sequence+=1
                self.session.operator(v['mode'],True,v['connected'],v['activity'],self.virtual_source,self.virtual_sequence,now,now)
            status=self.session.poll(now)
            if status['cancel_pending']:
                self.stop_actions()
            if self.session.phase=='replanning':self.session.reroute(now,False,not self.collision_stop and self.health_ready())
            if self.session.phase in ('starting','executing','replanning'):
                if not self.health_ready():self.session.cancel();self.session._notice('地面、定位或传感器未就绪，导航已停止')
                self.op(armed=not self.session.status(now)['force_navigation_zero'],paused=self.session.status(now)['force_navigation_zero'])
    def stop_actions(self):
        self.op(armed=False,paused=True)
        # Cancellation is acknowledged by terminal action results, never merely
        # because a cancel RPC was sent. Unknown acceptance keeps the latch.
        for handle in (self.goal_handle,self.plan_handle):
            if handle is not None and handle not in self.cancel_sent_handles:
                self.cancel_sent_handles.append(handle);handle.cancel_goal_async()
        if self.goal_handle is None and self.plan_handle is None and not self.pending_nav_accept and not self.pending_plan_accept:
            self.session.acknowledge_cancel(self.session.cancel_generation)

    def wait_future(self,future,timeout=15):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:time.sleep(.02)
        if not future.done():raise TimeoutError('服务响应超时')
        return future.result()
    def op(self,**values):
        m=String();m.data=json.dumps(values);self.operator.publish(m)
    def stop(self):
        with self.lock:self.session.cancel();self.stop_actions()
        self.goal_generation+=1
        self.op(armed=False,paused=True)
        if self.route_process is not None and self.route_process.poll() is None:
            os.killpg(self.route_process.pid,signal.SIGINT)
        if self.goal_handle is not None:self.goal_handle.cancel_goal_async()
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
        self.goal_handle=self.plan_handle=None;self.pending_nav_accept=self.pending_plan_accept=False
        self.cancel_sent_handles=[];self.accepted_actions={};self.action_results={}
        self.acknowledge_if_quiet(self.session.cancel_generation)
    def start(self,mode,name,controller):
        if mode not in ('mapping','navigation') or controller not in ('rpp','mppi'):raise ValueError('模式或控制器无效')
        if not SAFE_NAME.fullmatch(name):raise ValueError('地图名仅允许字母、数字、下划线和连字符')
        map_file=self.runtime/'maps'/name/'mapped.yaml'
        if mode=='navigation' and not map_file.is_file():raise ValueError('请先保存该地图')
        if mode=='navigation':validate_map_metadata(map_file)
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
            if typ is SaveMap:configure_save_request(req,stem)
            else:req.filename=stem
            response=self.wait_future(client.call_async(req),25)
            if not (response.result if typ is SaveMap else response.result==0):raise ValueError('地图保存失败')
            self.destroy_client(client)
        validate_map_metadata(target/'mapped.yaml')
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
    def acknowledge_if_quiet(self,generation):
        if self.goal_handle is None and self.plan_handle is None and not self.pending_nav_accept and not self.pending_plan_accept:
            self.session.acknowledge_cancel(generation)
    def track_acceptance(self,kind,generation,future):
        with self.lock:
            key=(kind,generation)
            if key in self.accepted_actions:return self.accepted_actions[key]
            if kind=='plan':self.pending_plan_accept=False
            else:self.pending_nav_accept=False
            try:
                handle=future.result()
                if handle is None or not hasattr(handle,'accepted'):raise ValueError('No action acceptance receipt')
            except Exception:
                # A transport exception cannot prove the server rejected the
                # request. Keep its unknown acceptance pending until the owned
                # action server is actually stopped, not just another poll.
                if kind=='plan':self.pending_plan_accept=True
                else:self.pending_nav_accept=True
                self.session.cancel();self.op(armed=False,paused=True)
                self.session._notice('执行回执异常，导航已停止；请重新载入工作流程')
                return None
            self.accepted_actions[key]=handle
            if not handle.accepted:self.acknowledge_if_quiet(generation);return handle
            if kind=='plan':self.plan_handle=handle
            else:self.goal_handle=handle
            result=handle.get_result_async();self.action_results[key]=result
            result.add_done_callback(lambda f:self.action_terminal(kind,generation,handle,f))
            expected='planning' if kind=='plan' else 'starting'
            if self.session.generation!=generation or self.session.phase!=expected:
                handle.cancel_goal_async();self.cancel_sent_handles.append(handle)
            elif kind=='nav':
                if self.session.started(generation,time.monotonic()):self.op(armed=True,paused=False,speed=self.speed_value)
                else:handle.cancel_goal_async();self.cancel_sent_handles.append(handle)
            return handle
    def action_terminal(self,kind,generation,handle,future):
        with self.lock:
            try:status=future.result().status
            except Exception:
                self.session.cancel();self.stop_actions()
                self.session._notice('终止回执异常，导航保持停止；请重新载入工作流程')
                return
            if kind=='plan' and self.plan_handle is handle:self.plan_handle=None
            if kind=='nav' and self.goal_handle is handle:self.goal_handle=None
            self.cancel_sent_handles=[h for h in self.cancel_sent_handles if h is not handle]
            if kind=='nav':
                if self.session.finished(generation,status==4):
                    self.op(armed=False,paused=True);self.last_action=self.session.message
            self.acknowledge_if_quiet(generation)
    def set_goal(self,data):
        if self.mode!='navigation':raise ValueError('请先载入地图进入导航模式')
        pose=self.checked_pose(data)
        if not self.gate.get('localized'):raise ValueError('定位尚未就绪，请先设置初始位置')
        with self.lock:
            generation=self.session.prepare(data,time.monotonic());self.path=[];self.goal=None
            self.pending_plan_accept=True
            self.op(armed=False,paused=True)
        if not self.planner.wait_for_server(timeout_sec=3):
            with self.lock:
                self.pending_plan_accept=False;self.session.planned(generation,0,time.monotonic());self.acknowledge_if_quiet(generation)
            raise ValueError('规划服务尚未就绪')
        request=ComputePathToPose.Goal();request.goal=pose;request.planner_id='GridBased'
        with self.lock:
            if self.session.generation!=generation or self.session.phase!='planning':
                self.pending_plan_accept=False;self.acknowledge_if_quiet(generation);raise ValueError('规划已取消')
            future=self.planner.send_goal_async(request)
        try:self.wait_future(future,5)
        except TimeoutError:
            self.session.cancel();future.add_done_callback(lambda f:self.track_acceptance('plan',generation,f));raise
        handle=self.track_acceptance('plan',generation,future)
        if handle is None or not handle.accepted:self.session.planned(generation,0,time.monotonic());raise ValueError('规划请求被拒绝')
        try:result=self.wait_future(self.action_results[('plan',generation)],10)
        except TimeoutError:self.session.cancel();self.stop_actions();raise
        path=[[p.pose.position.x,p.pose.position.y] for p in result.result.path.poses] if result.status==4 else []
        if not self.session.planned(generation,len(path),time.monotonic()):raise ValueError('路径未完成或已取消，请重新选择目标')
        self.path=path;self.goal=[data['x'],data['y']];self.last_action='路径已预览；确认开始前不会行走'
        return {'planned':True,'generation':generation,'path_points':len(path),'hardware_execution_enabled':False}
    def confirm_goal(self,data):
        generation=int(data['generation'])
        with self.lock:
            self.session.start(generation,time.monotonic(),self.simulation,self.health_ready())
            self.goal_generation=generation;pose=self.checked_pose(self.session.goal)
            self.pending_nav_accept=True
        if not self.nav.wait_for_server(timeout_sec=3):
            self.pending_nav_accept=False;self.session.cancel();self.acknowledge_if_quiet(generation);raise ValueError('导航服务未就绪')
        goal=NavigateToPose.Goal();goal.pose=pose
        with self.lock:
            if self.session.generation!=generation or self.session.phase!='starting':
                self.pending_nav_accept=False;self.acknowledge_if_quiet(generation);raise ValueError('开始已取消')
            future=self.nav.send_goal_async(goal,feedback_callback=self.on_feedback)
        try:self.wait_future(future,5)
        except TimeoutError:
            self.session.cancel();self.op(armed=False,paused=True)
            future.add_done_callback(lambda f:self.track_acceptance('nav',generation,f));raise
        handle=self.track_acceptance('nav',generation,future)
        if handle is None or not handle.accepted:
            self.session.cancel();self.acknowledge_if_quiet(generation);raise ValueError('导航请求被拒绝')
        if self.session.phase!='executing':raise ValueError('开始过程中已取消或手动接管')
        return {'accepted':True,'generation':generation,'hardware_execution_enabled':False}
    def on_feedback(self,feedback):
        f=feedback.feedback;self.feedback={'distance_remaining':f.distance_remaining,'recoveries':f.number_of_recoveries}
    def speed(self,value):
        v=float(value)
        if not math.isfinite(v) or not self.minimum_speed<=v<=self.maximum_speed:raise ValueError(f'此步态配置的速度范围为 {self.minimum_speed:.2f}–{self.maximum_speed:.2f} m/s')
        self.speed_value=v;self.publish_speed()
        return {'speed_m_s':v}
    def publish_speed(self):
        self.op(speed=self.speed_value)
        limit=SpeedLimit();limit.percentage=False;limit.speed_limit=self.speed_value;self.limiter.publish(limit)
    def status(self,include_cells=True):
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
            'session':self.session.status(time.monotonic()),'map_revision':self.map_revision,
            'execution_ready':self.simulation,'map':None if grid is None else {'width':grid.info.width,'height':grid.info.height,'resolution':grid.info.resolution,
                'origin':[grid.info.origin.position.x,grid.info.origin.position.y],**({'cells':list(grid.data)} if include_cells else {})},
            'path':self.path,'goal':self.goal,'feedback':self.feedback,
            **({'truth':self.truth} if self.simulation else {}),
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
            elif urlsplit(self.path).path=='/api/status':self.send(node.status(include_cells=parse_qs(urlsplit(self.path).query).get('summary')!=['1']))
            elif self.path=='/api/map':self.send({'revision':node.map_revision,'map':node.status()['map']})
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
                    '/api/confirm':lambda:node.confirm_goal(d),'/api/sim_operator':lambda:node.simulation_operator(d),
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
