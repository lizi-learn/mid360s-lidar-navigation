#!/usr/bin/env python3
"""MID-360S live LiDAR map and path preview. No locomotion command publisher."""
import os, json, math, threading, time, pathlib, argparse, gzip, signal
from collections import deque
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.ndimage import label
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from nav_geometry import level_rotation, estimate_floor, FloorMap
from survey import Survey
from localization_health import source_age,pose_discontinuous

ROOT=pathlib.Path(__file__).resolve().parent
RUNTIME=pathlib.Path(os.getenv('P2_NAV_RUNTIME',str(ROOT/'runtime'))).resolve()
ROBOT_IDENTITY=None
LIDAR_IN_IMU=np.array([-.011,-.02329,.04412])

def cloud_xyz(msg):
    fields={f.name:f for f in msg.fields}
    endian='>' if msg.is_bigendian else '<'
    dtype=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,
        'offsets':[fields[k].offset for k in ['x','y','z']],'itemsize':msg.point_step})
    a=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step)).ravel()
    return np.stack([a[k] for k in ['x','y','z']],axis=1).astype(float)

class NavigationView(Node):
    def __init__(self):
        super().__init__('mid360s_navigation_view')
        self.lock=threading.RLock();self.rotation=None;self.height=None
        self.acc=[];self.recent_acc=deque(maxlen=100);self.raw_time=0.;self.imu_time=0.;self.cloud_time=0.;self.odom_time=0.
        self.imu_count=0;self.cloud_count=0
        self.imu_stamp=None;self.cloud_stamp=None;self.odom_stamp=None;self.previous_odom=None;self.localization_error=""
        self.points=np.empty((0,3))
        self.sensor_xy=[0.,0.];self.sensor_z=None;self.sensor_yaw=0.;self.sensor_pitch=0.;self.path=[];self.goal=None;self.path_state='未选择目标'
        self.floor_map=FloorMap();self.floor_error='等待雷达惯性建图';self.survey=Survey(ROOT,RUNTIME)
        self.last_map_update=0.;self.initial_tilt=None
        self.static_tf=StaticTransformBroadcaster(self);self.dynamic_tf=TransformBroadcaster(self)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub=self.create_publisher(OccupancyGrid,'/map',latched)
        self.path_pub=self.create_publisher(Path,'/p2_nav/planned_path',latched)
        self.cloud_pub=self.create_publisher(PointCloud2,'/p2_nav/cloud_level',qos_profile_sensor_data)
        # Cloud/map callbacks can occupy more than the default five IMU samples.
        # Keep a bounded backlog; source stamps and stale-data checks stay unchanged.
        imu_qos=QoSProfile(depth=64,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu,'/livox/imu',self.on_imu,imu_qos)
        self.create_subscription(Imu,'/p2_nav/body_imu',lambda msg:self.survey.imu('body',msg),imu_qos)
        self.create_subscription(PointCloud2,'/cloud_registered',self.on_cloud,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/Odometry',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(PoseStamped,'/goal_pose',self.on_rviz_goal,10)
        self.create_timer(.5,self.publish_map)
        self.create_timer(.1,self.publish_reference)

    def on_imu(self,msg):
        self.survey.imu('lidar',msg)
        with self.lock:
            self.imu_time=time.monotonic();self.imu_count+=1;self.imu_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            a=msg.linear_acceleration
            self.recent_acc.append([a.x,a.y,a.z])
            if self.rotation is None:
                self.acc.append([a.x,a.y,a.z])
                if len(self.acc)>=100:
                    try:
                        acc=np.mean(self.acc,axis=0)
                        self.rotation=level_rotation(acc)
                        self.initial_tilt=math.degrees(math.acos(np.clip(-acc[2]/np.linalg.norm(acc),-1,1)))
                    except ValueError as e:self.floor_error=str(e);self.acc=[]

    def on_cloud(self,msg):
        with self.lock:
            if self.rotation is None:return
            p=cloud_xyz(msg);p=p[np.isfinite(p).all(axis=1)]
            p=p@self.rotation.T
            if self.height is None:
                try:
                    self.height=-estimate_floor(p)
                    t=TransformStamped();t.header.stamp=self.get_clock().now().to_msg()
                    t.header.frame_id='map';t.child_frame_id='camera_init';t.transform.translation.z=self.height
                    q=Rotation.from_matrix(self.rotation).as_quat()
                    t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,q)
                    self.static_tf.sendTransform(t)
                    RUNTIME.mkdir(parents=True,exist_ok=True)
                    (RUNTIME/'map_alignment.json').write_text(json.dumps({'rotation':self.rotation.tolist(),'initial_height_m':self.height,'initial_tilt_deg':self.initial_tilt,'robot_identity':ROBOT_IDENTITY},indent=2))
                except ValueError as e:self.floor_error=str(e);return
            p[:,2]+=self.height
            radius=np.linalg.norm(p[:,:2]-np.array(self.sensor_xy),axis=1)
            p=p[(radius>.45)&(radius<14)&(p[:,2]>-.3)&(p[:,2]<3)]
            self.points=p[::max(1,len(p)//7000)]
            self.cloud_time=time.monotonic();self.cloud_count+=1;self.floor_error='';self.cloud_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            self.raw_time=self.cloud_time
            if not self.localization_error and self.cloud_time-self.last_map_update>.35:
                self.floor_map.recenter(*self.sensor_xy);self.floor_map.update(p);self.last_map_update=self.cloud_time
                self.survey.cloud(p)
                if self.path:
                    free,_=self.floor_map.layers()
                    if any(self.floor_map.cell(*v) is None or not free[self.floor_map.cell(*v)] for v in self.path):
                        self.path=[];self.path_state='地图发生变化，请重新选择目标'
                        empty=Path();empty.header.frame_id='map';empty.header.stamp=self.get_clock().now().to_msg();self.path_pub.publish(empty)

    def on_odom(self,msg):
        with self.lock:
            if self.rotation is None:return
            pos=msg.pose.pose.position
            q=msg.pose.pose.orientation
            imu_rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            # FAST-LIO odometry describes the IMU origin; the preview arrow
            # and height describe the LiDAR optical origin.
            p=self.rotation@(np.array([pos.x,pos.y,pos.z])+imu_rotation@LIDAR_IN_IMU)
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if self.previous_odom is not None and pose_discontinuous(self.previous_odom[1],p[:2],stamp-self.previous_odom[0]):
                self.localization_error='定位出现位置跳变或时间回退，已暂停路径预览；需要重新核验定位'
                with self.survey.lock:
                    if self.survey.active:self.survey.issues.add(self.localization_error)
                self.path=[];self.path_state=self.localization_error
                empty=Path();empty.header.frame_id='map';empty.header.stamp=self.get_clock().now().to_msg();self.path_pub.publish(empty)
                (RUNTIME/'localization-fault.json').write_text(json.dumps({'time':time.time(),'source_stamp':stamp,'before_xy':list(self.previous_odom[1]),'after_xy':p[:2].tolist(),'dt':stamp-self.previous_odom[0],'execution_enabled':False},ensure_ascii=False,indent=2))
            self.previous_odom=(stamp,p[:2].tolist());self.odom_stamp=stamp
            self.sensor_xy=p[:2].tolist();self.sensor_z=float(p[2])
            orientation=self.rotation@imu_rotation@np.diag([1.,-1.,-1.])
            self.sensor_yaw=math.atan2(orientation[1,0],orientation[0,0]);self.odom_time=time.monotonic()
            self.sensor_pitch=math.atan2(-orientation[2,0],math.hypot(orientation[0,0],orientation[1,0]))
            if self.height is not None:self.survey.pose(self.sensor_xy,self.sensor_z+self.height,self.sensor_yaw,self.sensor_pitch,msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)

    def message_age(self,kind):
        stamp=getattr(self,kind+'_stamp');received=getattr(self,kind+'_time')
        if stamp is None or not received:return None
        return source_age(self.get_clock().now().nanoseconds*1e-9,stamp,time.monotonic()-received)

    def publish_reference(self):
        with self.lock:
            if self.height is None or self.localization_error or self.message_age("odom") is None or self.message_age("odom")>1:return
            t=TransformStamped();t.header.stamp=self.get_clock().now().to_msg();t.header.frame_id='map';t.child_frame_id='navigation_preview'
            t.transform.translation.x=float(self.sensor_xy[0]);t.transform.translation.y=float(self.sensor_xy[1])
            t.transform.rotation.z=math.sin(self.sensor_yaw/2);t.transform.rotation.w=math.cos(self.sensor_yaw/2)
            self.dynamic_tf.sendTransform(t)

    def publish_map(self):
        with self.lock:
            if self.height is None or self.localization_error or self.message_age("cloud") is None or self.message_age("cloud")>1:return
            _,occupancy=self.floor_map.layers()
            msg=OccupancyGrid();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='map'
            msg.info.resolution=self.floor_map.resolution;msg.info.width=self.floor_map.size;msg.info.height=self.floor_map.size
            msg.info.origin.position.x=float(self.floor_map.origin_xy[0]);msg.info.origin.position.y=float(self.floor_map.origin_xy[1]);msg.info.origin.orientation.w=1.
            msg.data=occupancy.ravel().tolist();self.map_pub.publish(msg)

    def plan(self,x,y):
        with self.lock:
            now=time.monotonic()
            if self.localization_error:raise ValueError(self.localization_error)
            if self.message_age('cloud') is None or self.message_age('cloud')>1 or self.message_age('odom') is None or self.message_age('odom')>.7:
                raise ValueError('雷达或定位数据已过期，暂停规划')
            free,_=self.floor_map.layers()
            start=self.sensor_xy
            start_cell=self.floor_map.cell(*start)
            if start_cell is None:raise ValueError('当前定位超出地图范围')
            goal_cell=self.floor_map.cell(x,y)
            if goal_cell is None or not free[goal_cell]:raise ValueError('目标位于障碍、未知地面或距离障碍过近')
            offset=0.
            components,_=label(free)
            goal_component=components[goal_cell]
            if not free[start_cell] or components[start_cell]!=goal_component:
                candidates=np.argwhere(components==goal_component)
                if not len(candidates):raise ValueError('尚未观察到可通行地面')
                delta=candidates-np.array(start_cell)
                nearest=candidates[np.linalg.norm(delta,axis=1).argmin()]
                offset=float(np.linalg.norm(nearest-np.array(start_cell))*self.floor_map.resolution)
                if offset>2:raise ValueError('附近没有可用于预览的已知地面')
                start=self.floor_map.world(nearest)
            path=self.floor_map.plan(start,[x,y])
            self.path=path;self.goal=[x,y];self.path_state='路径有效'
            msg=Path();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='map'
            for i,p in enumerate(path):
                pose=PoseStamped();pose.header=msg.header;pose.pose.position.x=p[0];pose.pose.position.y=p[1];pose.pose.orientation.w=1.
                msg.poses.append(pose)
            self.path_pub.publish(msg)
            return {'path':path,'goal':self.goal,'preview_start':start,'start_gap_m':offset,'execution_enabled':False,'message':'路径仅覆盖已知地面；雷达下方盲区不自动补成可通行区域'}

    def on_rviz_goal(self,msg):
        if msg.header.frame_id!='map':return
        try:self.plan(msg.pose.position.x,msg.pose.position.y)
        except ValueError as e:self.get_logger().warn(str(e))

    def status(self):
        with self.lock:
            now=time.monotonic();free,_=self.floor_map.layers()
            points=self.points
            gravity=np.mean(self.recent_acc,axis=0) if self.recent_acc else None
            return {'robot':'MID-360S platform','sensor':'MID-360S','mount':'倒装','raw_age':round(self.message_age('cloud'),3) if self.message_age('cloud') is not None else None,
                'imu_age':round(self.message_age('imu'),3) if self.message_age('imu') is not None else None,'map_age':round(self.message_age('cloud'),3) if self.message_age('cloud') is not None else None,
                'odom_age':round(self.message_age('odom'),3) if self.message_age('odom') is not None else None,'odom_receive_age':round(now-self.odom_time,3) if self.odom_time else None,'counts':{'imu':self.imu_count,'map_frames':self.cloud_count},
                'map_kind':'随雷达移动的近期局部图；推架记录另存累计图',
                'initial_tilt_deg':self.initial_tilt,'sensor_height_m':self.height+self.sensor_z if self.height is not None and self.sensor_z is not None else None,'alignment_origin_height_m':self.height,'error':self.localization_error or self.floor_error,'localization_fault':self.localization_error,
                'sensor_xy':self.sensor_xy,'sensor_yaw':self.sensor_yaw,'points':np.round(points,3).tolist(),
                'gravity_raw':gravity.tolist() if gravity is not None else None,
                'gravity_pitch_deg':math.degrees(math.atan2(-gravity[0],-gravity[2])) if gravity is not None else None,
                'lio_pitch_deg':math.degrees(self.sensor_pitch),
                'free_cells':np.argwhere(free)[::1].tolist(),'obstacle_cells':np.argwhere(self.floor_map.obstacles).tolist(),
                'grid':{'resolution':self.floor_map.resolution,'origin':self.floor_map.origin,'origin_xy':self.floor_map.origin_xy.tolist(),'size':self.floor_map.size},
                'path':self.path,'goal':self.goal,'path_state':self.path_state,'execution_enabled':False,'robot_name':ROBOT_IDENTITY['name'] if ROBOT_IDENTITY else None,
                'localization_reference':'雷达地面投影；机身与颈部运动外参待核验',
                'survey':self.survey.status(),
                'neck':json.loads((RUNTIME/'neck_status.json').read_text()) if (RUNTIME/'neck_status.json').is_file() else {'state':'固定安装姿态 · 此程序不控制电机'}}

def main():
    global RUNTIME,ROBOT_IDENTITY
    selected=os.getenv('P2_NAV_ROBOT_PROFILE',str(ROOT/'config/robot.local.json'))
    if pathlib.Path(selected).is_file():
        from robot_profile import load,check_machine,bind_runtime,runtime_path,identity
        profile=load(selected);check_machine(profile)
        RUNTIME=bind_runtime(profile,os.getenv('P2_NAV_RUNTIME') or runtime_path(profile,ROOT/'runtime/robots'));ROBOT_IDENTITY=identity(profile)
    elif os.getenv('P2_NAV_ROBOT_PROFILE'):raise ValueError('Selected robot profile missing')
    RUNTIME.mkdir(parents=True,exist_ok=True)
    parser=argparse.ArgumentParser();parser.add_argument('--bind',default='127.0.0.1');parser.add_argument('--port',type=int,default=8920);args=parser.parse_args()
    rclpy.init();node=NavigationView();stop=threading.Event()
    def spin():
        try:rclpy.spin(node)
        except Exception as e:node.get_logger().error(str(e))
        finally:stop.set()
    worker=threading.Thread(target=spin,daemon=True);worker.start()
    def limits():
        while not stop.wait(2):node.survey.check_limits()
    monitor=threading.Thread(target=limits,daemon=True);monitor.start()
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:stop.set())
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send_json(self,value,code=200):
            raw=json.dumps(value,ensure_ascii=False,allow_nan=False).encode();compressed='gzip' in self.headers.get('Accept-Encoding','') and len(raw)>2048
            if compressed:raw=gzip.compress(raw,compresslevel=1)
            self.send_response(code);self.send_header('Content-Type','application/json; charset=utf-8');self.send_header('Cache-Control','no-store')
            if compressed:self.send_header('Content-Encoding','gzip')
            self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
        def do_GET(self):
            if self.path=='/api/status':self.send_json(node.status());return
            if self.path not in ['/','/index.html']:self.send_json({'error':'Not found'},404);return
            raw=(ROOT/'web/index.html').read_bytes();self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
        def do_POST(self):
            try:
                if self.path not in ['/api/goal','/api/survey/start','/api/survey/return','/api/survey/stop']:self.send_json({'error':'Not found'},404);return
                n=int(self.headers.get('Content-Length','0'))
                if n>1024:raise ValueError('请求过大')
                data=json.loads(self.rfile.read(n) or b'{}')
                if self.path=='/api/survey/start':
                    with node.lock:
                        if node.localization_error:raise ValueError(node.localization_error)
                        if node.message_age('odom') is None or node.message_age('odom')>.7:raise ValueError('定位数据已过期，暂停开始记录')
                        health={'map_age':node.message_age('cloud') if node.message_age('cloud') is not None else 1e6,'imu_age':node.message_age('imu') if node.message_age('imu') is not None else 1e6}
                    self.send_json(node.survey.start(health));return
                if self.path=='/api/survey/return':
                    if data.get('physical_return_confirmed') is not True:raise ValueError('需要操作者确认实物已回到同一起点并且朝向一致')
                    with node.lock:
                        if node.localization_error:raise ValueError(node.localization_error)
                        if node.message_age('odom') is None or node.message_age('odom')>.7:raise ValueError('定位数据已过期，不能记录闭合检查')
                    self.send_json(node.survey.mark_return());return
                if self.path=='/api/survey/stop':self.send_json(node.survey.stop());return
                x,y=float(data['x']),float(data['y'])
                if not (math.isfinite(x) and math.isfinite(y)):raise ValueError('目标无效')
                self.send_json(node.plan(x,y))
            except (ValueError,KeyError,TypeError) as e:self.send_json({'error':str(e)},400)
    server=ThreadingHTTPServer((args.bind,args.port),Handler);server.timeout=.5
    try:
        while not stop.is_set():server.handle_request()
    finally:
        server.server_close()
        node.survey.stop('传感器服务退出，本段保存；不能跨重启拼接')
        if rclpy.ok():rclpy.shutdown()
        worker.join(timeout=2);node.destroy_node()

if __name__=='__main__':main()
