#!/usr/bin/env python3
"""Local workflow console: mapping, map save/load, pose, goals, speed and stop."""
import argparse, gzip, json, math, os, re, signal, subprocess, threading, time, secrets
from pathlib import Path
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,Twist
from nav_msgs.msg import OccupancyGrid,Path as RosPath
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose,ComputePathToPose,NavigateThroughPoses,ComputePathThroughPoses
from nav2_msgs.msg import SpeedLimit,CollisionMonitorState,CostmapFilterInfo,BehaviorTreeLog,Costmap
from slam_toolbox.srv import SerializePoseGraph
from nav2_msgs.srv import SaveMap
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from map_persistence import configure_save_request,validate_map_metadata
from session import NavigationSession
from urllib.parse import urlsplit,parse_qs

ROOT=Path(__file__).resolve().parent
SAFE_NAME=re.compile(r'[A-Za-z0-9_-]{1,48}\Z')

class GateHealthNode(Node):
    def __init__(self,consume):
        super().__init__('navigation_workflow_health_forwarder')
        # A dedicated executor receives only original gate evaluations. It must
        # not queue behind path/map callbacks or a shared executor worker pool.
        self.create_subscription(String,'/p2_nav/gate_status',consume,1)


class ConsoleExecutors:
    def __init__(self,node):
        self.node=node
        self.work=SingleThreadedExecutor();self.work.add_node(node)
        self.health=SingleThreadedExecutor();self.health.add_node(node.health_node)
        self.health_thread=threading.Thread(target=self.health.spin,daemon=True,name='original_gate_evaluations')
    def spin(self):
        self.health_thread.start()
        self.work.spin()
    def shutdown(self):
        self.work.shutdown();self.health.shutdown()
        self.health_thread.join(timeout=3.)
        self.node.health_node.destroy_node()


class Console(Node):
    def __init__(self,runtime,simulation,minimum_speed,footprint=None,odometry_model='differential',gait_profile='generic',gait_socket=''):
        super().__init__('navigation_workflow_console')
        self.gait=None
        if gait_socket:
            if simulation:raise ValueError('Physical gait transport requires a separate explicit contract fixture, never normal simulation')
            from gait_transport_process import GaitTransportProcess
            self.gait=GaitTransportProcess(gait_socket,trace_path=runtime/'gait-velocity.private.jsonl')
        self.robot_name=None;self.runtime=runtime;self.simulation=simulation;self.mode='idle';self.controller='rpp';self.process=None
        from audio_feedback import AudioSettings
        self.audio=AudioSettings(runtime/'audio-settings.json')
        self.minimum_speed=minimum_speed;self.footprint=footprint;self.odometry_model=odometry_model;self.gait_profile=gait_profile
        self.maximum_speed=.8 if gait_profile=='p2-flat-model' else .30
        self.route_process=None;self.route_directory=None
        self.lock=threading.RLock();self.grid=None;self.path=[];self.gate={};self.truth={};self.goal=None
        self.last_action='等待选择建图或载入地图';self.goal_handle=None;self.feedback={}
        self.path_source_report={};self.goal_generation=0;self.session=NavigationSession();self.map_revision=0
        if self.gait:self.gait.operator_observer=lambda:self.poll_gait_authority(time.monotonic())
        self.plan_handle=None;self.pending_nav_accept=False;self.pending_plan_accept=False;self.cancel_sent_handles=[];self.action_results={};self.accepted_actions={}
        self.collision_stop=False;self.path_revision=0;self.obstacle_path_revision=0
        self.bt_goal_source_ns=0;self.bt_recovery_report={}
        self.virtual_operator=None;self.virtual_sequence=0;self.virtual_source=secrets.randbits(63)+1
        self.request_token=secrets.token_urlsafe(24)
        from flight_recorder import FlightRecorder
        self.flight=FlightRecorder(runtime/'planning-evidence.private.jsonl.gz')
        self.flight_costmap_at=0.
        self.navigation_grid=None;self.overlay_revision=0;self.overlay_report={};self._overlay_key=None;self._overlay_lock=threading.Lock()
        self.speed_value=.8 if gait_profile=='p2-flat-model' else min(.20,self.maximum_speed)
        self.velocity_scale=.5 if gait_profile=='p2-flat-model' else 1.
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.reception_map=self.create_publisher(OccupancyGrid,'/p2_nav/reception_map',qos)
        self.keepout_mask=self.create_publisher(OccupancyGrid,'/p2_nav/keepout_mask',qos)
        self.keepout_info=self.create_publisher(CostmapFilterInfo,'/p2_nav/keepout_filter_info',qos)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,qos)
        self.create_subscription(RosPath,'/plan',self.record_source_path,10)
        self.create_subscription(Costmap,'/global_costmap/costmap_raw',self.record_source_costmap,10)
        if self.gait:self.create_subscription(String,'/p2_nav/path_view',self.on_path_view,10)
        else:self.create_subscription(RosPath,'/plan',self.on_path,10)
        self.create_subscription(CollisionMonitorState,'/p2_nav/collision_state',self.on_collision,10)
        self.create_subscription(BehaviorTreeLog,'/behavior_tree_log',self.on_bt_log,10)
        self.create_subscription(String,'/p2_nav/operator_authority',self.on_authority,10)
        self.health_node=GateHealthNode(self.on_gate)
        self.create_subscription(String,'/p2_nav/sim_truth',lambda m:setattr(self,'truth',json.loads(m.data)),10)
        self.operator=self.create_publisher(String,'/p2_nav/operator',10)
        self.limiter=self.create_publisher(SpeedLimit,'/speed_limit',10)
        self.initial=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.nav=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.planner=ActionClient(self,ComputePathToPose,'compute_path_to_pose')
        self.nav_through=ActionClient(self,NavigateThroughPoses,'navigate_through_poses')
        self.planner_through=ActionClient(self,ComputePathThroughPoses,'compute_path_through_poses')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.create_timer(.5,self.publish_speed)
        self.create_timer(.02,self.poll_session)
        from reception_catalog import Catalog
        from reception_mission import MissionService
        from reception_voice import VoiceCache
        from reception_cloud import CloudStore
        from reception_transfer import ContentTransfer
        self.map_name=''
        self.reception_catalog=Catalog(runtime/'reception')
        self.reception_cloud=CloudStore(runtime/'reception-cloud')
        self.reception_transfer=ContentTransfer(self.reception_catalog,self.reception_cloud)
        from reception_annotations import AnnotationLayer
        from reception_maps import MapLibrary,BundleTransfer
        self.annotation_layer=AnnotationLayer();self.speed_zone_status={'factor':1.,'active':[]}
        self.map_library=MapLibrary(runtime/'maps',self.reception_catalog);self.bundle_transfer=BundleTransfer(self.map_library)
        self.map_update_region=None;self.scan_preview=None
        tts_command=json.loads(os.environ.get('P2_RECEPTION_TTS_COMMAND','null'))
        if tts_command is not None and (not isinstance(tts_command,list) or not tts_command or not all(isinstance(x,str) for x in tts_command)):
            raise ValueError('TTS adapter must be an argv JSON array')
        self.reception_voice=VoiceCache(runtime/'reception-voice-cache',tts_command)
        self.reception_transfer.on_saved=self.reception_voice.auto_prepare
        self.reception=MissionService(self,self.reception_catalog)
        self.trace_active_until=0.;self.create_timer(.2,self.capture_reception_trace)
        self.create_subscription(String,'/p2_nav/door_observations',self.on_door_observation,10)
        from reception_doors import DoorOverlay
        from reception_door_worker import DoorWorker
        self.door_overlay=DoorOverlay()
        self.door_source=os.environ.get('P2_RECEPTION_DOOR_SOURCE','scan')
        self.door_worker=DoorWorker(self.reception.observation_document,self.consume_door_rows,self.publish_reception_map)
        self.pending_reception_scan=None;self.door_scan_rejections={}
        self.create_subscription(LaserScan,'/scan',self.on_reception_scan,qos_profile_sensor_data)
        self.create_timer(.05,self.process_reception_scan)
    def record_source_path(self,msg):
        if self.session.phase not in ('planning','ready','starting','executing','replanning'):return
        try:
            self.flight.submit('path',dict(source_ros_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                frame_id=msg.header.frame_id,generation=self.session.generation,phase=self.session.phase,
                poses=[[p.pose.position.x,p.pose.position.y,p.pose.position.z,p.pose.orientation.x,
                        p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w]for p in msg.poses]))
        except (ValueError,TypeError,AttributeError):self.flight.errors+=1
    def record_source_costmap(self,msg):
        now=time.monotonic()
        if self.session.phase not in ('planning','ready','starting','executing','replanning')or now-self.flight_costmap_at<1.:return
        self.flight_costmap_at=now
        try:
            m=msg.metadata;p=m.origin
            self.flight.submit('costmap',dict(source_ros_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                update_ros_ns=m.update_time.sec*10**9+m.update_time.nanosec,
                frame_id=msg.header.frame_id,generation=self.session.generation,phase=self.session.phase,
                size_x=m.size_x,size_y=m.size_y,resolution=m.resolution,
                origin=[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w],
                cells=bytes(msg.data),provenance='asynchronous raw global costmap; not an atomic planner snapshot'))
        except (ValueError,TypeError,AttributeError):self.flight.errors+=1
    def on_map(self,msg):
        self.grid=msg;self.map_revision+=1
        if hasattr(self,'door_worker'):self.publish_reception_map(self.reception.observation_document())
    def display_revision(self):return self.map_revision+getattr(self,'overlay_revision',0)
    def on_reception_scan(self,msg):
        # Scan and odometry callbacks can arrive in either order. Retain only
        # the newest original scan and wait for its exact-time TF; never use
        # a latest transform or relabel a measurement with a newer timestamp.
        self.pending_reception_scan=msg
    def process_reception_scan(self):
        msg=self.pending_reception_scan
        if msg is None or self.grid is None:return
        try:
            source_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            age=(self.get_clock().now().nanoseconds-source_ns)/1e9
            if source_ns<=0 or not -.05<=age<=.5:
                self.pending_reception_scan=None;raise ValueError('scan_stamp_outside_window')
            t=self.buffer.lookup_transform('map',msg.header.frame_id,rclpy.time.Time.from_msg(msg.header.stamp)).transform
            q=t.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            if abs(q.x)>1e-5 or abs(q.y)>1e-5:
                self.pending_reception_scan=None;raise ValueError('scan_not_horizontal')
            if not 0<len(msg.ranges)<=4096:
                self.pending_reception_scan=None;raise ValueError('scan_size_invalid')
            packet=dict(source_ns=source_ns,source_monotonic=time.monotonic()-age,
                x=t.translation.x,y=t.translation.y,yaw=yaw,ranges=list(msg.ranges),
                angle_min=msg.angle_min,angle_increment=msg.angle_increment,range_min=msg.range_min,range_max=msg.range_max)
            # Compact actual finite return overlay; the original ROS header is
            # retained. UI polling never makes stale scans appear fresh.
            pts=[]
            for i,distance in enumerate(msg.ranges):
                if not math.isfinite(distance)or not msg.range_min<=distance<=msg.range_max:continue
                angle=yaw+msg.angle_min+i*msg.angle_increment
                pts.append([t.translation.x+distance*math.cos(angle),t.translation.y+distance*math.sin(angle)])
            self.scan_preview={'source_ns':source_ns,'frame_id':'map','origin':[t.translation.x,t.translation.y,yaw],
                               'points':pts[::max(1,math.ceil(len(pts)/600))]}
            if self.door_source=='scan':self.door_worker.submit(packet)
            self.pending_reception_scan=None
        except Exception as error:
            key=str(error)if isinstance(error,ValueError)else type(error).__name__
            self.door_scan_rejections[key]=self.door_scan_rejections.get(key,0)+1
    def consume_door_rows(self,document,rows):
        current=self.reception.observation_document()
        if current['map_id']!=document['map_id']or current['doors']!=document['doors']:return
        self.reception.decisions.bind(document)
        from reception_door_worker import accept_rows
        accept_rows(self.reception.decisions,document,rows,time.monotonic())
    def publish_reception_map(self,document):
        import copy
        base=self.grid
        if base is None or not hasattr(self,'door_overlay') or document['map_id']!=self.reception.map_id():return
        states=self.reception.decisions.states(document,time.monotonic())
        annotations=document.get('annotations',[])
        key=(self.map_revision,tuple(sorted(states.items())),tuple((d['id'],d['x'],d['y'],d['width'],d['yaw'])for d in document['doors']),json.dumps(annotations,sort_keys=True))
        with self._overlay_lock:
            if key==self._overlay_key:return
            q=base.info.origin.orientation
            grid={'revision':self.map_revision,'width':base.info.width,'height':base.info.height,'resolution':base.info.resolution,
                  'origin':[base.info.origin.position.x,base.info.origin.position.y],'cells':base.data}
            if abs(q.z)>1e-8 or abs(q.x)>1e-8 or abs(q.y)>1e-8:
                cells=list(base.data);report={'changed_cells':0,'reason':'rotated_map_overlay_unsupported','original_map_modified':False}
            else:
                cells,report=self.door_overlay.apply(grid,document,states,self.reception.decisions.observations(document,time.monotonic()))
                cells,authored=self.annotation_layer.apply(dict(grid,cells=cells),annotations);report.update(authored)
            msg=copy.deepcopy(base);msg.data=cells
            self.navigation_grid=msg;self.overlay_revision+=1;self.overlay_report=report;self._overlay_key=key
            self.reception_map.publish(msg)
            # A filter mask is an authored instruction layer, not a free-space
            # measurement. Use the standard KeepoutFilter in BOTH costmaps so
            # local tracking cannot cut across a globally planned virtual wall.
            mask=copy.deepcopy(base);mask.data=[0]*len(base.data)
            if abs(q.z)<=1e-8 and abs(q.x)<=1e-8 and abs(q.y)<=1e-8:
                mask.data=self.annotation_layer.apply(dict(grid,cells=mask.data),annotations)[0]
            self.keepout_mask.publish(mask)
            info=CostmapFilterInfo();info.header=base.header;info.type=0
            info.filter_mask_topic='/p2_nav/keepout_mask';info.base=0.;info.multiplier=1.;self.keepout_info.publish(info)

    def on_authority(self,msg):
        # An attached gait loop has the verified physical sample. Do not race it
        # against another ROS forwarder with a separate source identity/latency.
        if getattr(self,'gait',None) is not None:return
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
        points=[[p.pose.position.x,p.pose.position.y] for p in msg.poses]
        self.accept_path_view(points,msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)
    def on_path_view(self,msg):
        from path_view import decode_view
        try:points,stamp=decode_view(msg.data)
        except (ValueError,TypeError,KeyError):return
        self.accept_path_view(points,stamp)
    def accept_path_view(self,points,source_ns):
        with self.lock:
            self.path_revision+=1
            age=(self.get_clock().now().nanoseconds-source_ns)/1e9
            self.path_source_report={'source_ros_ns':source_ns,'age_at_receipt_s':age,'revision':self.path_revision}
            if self.session.phase in ('executing','replanning'):
                self.path=points
                age=(self.get_clock().now().nanoseconds-source_ns)/1e9
                fresh=-.05<=age<=.5 and self.path_revision>self.obstacle_path_revision and len(self.path)>1
                self.session.reroute(time.monotonic(),fresh,not self.collision_stop and self.health_ready())
    def on_bt_log(self,msg):
        source_ns=msg.timestamp.sec*10**9+msg.timestamp.nanosec
        now_ns=self.get_clock().now().nanoseconds
        with self.lock:
            if self.session.phase not in ('executing','replanning') or source_ns<self.bt_goal_source_ns or not -.05<=(now_ns-source_ns)/1e9<=.5:return
            for event in msg.event_log:
                if event.node_name=='ObservedRouteWait' and event.current_status=='RUNNING':
                    self.bt_recovery_report={'source_ros_ns':str(source_ns),'generation':self.goal_generation,'node':event.node_name,'age_at_receipt_s':(now_ns-source_ns)/1e9}
                    if self.session.phase=='executing':
                        self.obstacle_path_revision=self.path_revision;self.session.obstacle(time.monotonic())
                        self.session._notice('路径暂不可用：已停车，正在等待或重新规划')
    def on_collision(self,msg):
        with self.lock:
            self.collision_stop=msg.action_type==CollisionMonitorState.STOP
            if self.collision_stop and self.session.phase=='executing':
                self.obstacle_path_revision=self.path_revision;self.session.obstacle(time.monotonic())
    def execution_ready(self):
        gait=getattr(self,'gait',None)
        return self.simulation or bool(gait and gait.ready())
    def on_gate(self,msg):
        with self.lock:
            try:self.gate=json.loads(msg.data)
            except (ValueError,TypeError):self.gate={}
            if self.gate.get('collision_source_valid') is not True and self.session.phase in ('starting','executing','replanning'):
                self.session.cancel();self.stop_actions()
                self.session._notice('避障观测失效，导航已停止；恢复后请重新确认目标')
            # Forward this original health evaluation as soon as it arrives,
            # instead of waiting for a separate session timer and RPC cycle.
            gait=getattr(self,'gait',None)
            if gait:
                status=self.session_state(refresh=False)
                gait.update(self.gate,self.session.phase=='executing' and not status['force_navigation_zero'])
    def health_ready(self):
        g=self.gate
        return bool(g.get('collision_source_valid') is True and g.get('data_fresh') is True and g.get('localized') and g.get('ground_ok') and
            -.05<=g.get('scan_age',999)<=.5 and -.05<=g.get('odom_age',999)<=.5)
    def poll_gait_authority(self,now):
        gait=getattr(self,'gait',None)
        if gait is None:return
        for attempt in range(2):
            d=gait.operator_authority()
            with self.session.lock:
                now=max(now,time.monotonic())
                source_fresh=bool(d and -.005<=now-d['stamp']<=.1)
                reply_fresh=bool(d and(d.get('source_transport')=='native_packet_snapshot' or -.005<=now-d['reply_stamp']<=.1))
                if not source_fresh or not reply_fresh:
                    # One UI read may be descheduled while the independent
                    # receiver keeps advancing. Re-read the actual packet;
                    # never relabel the old sample or extend its deadline.
                    if attempt==0:continue
                    self.session.operator('unknown',False,False,False,0,0,0.,now);return
                if d['source']==self.session.source and d['sequence']<=self.session.sequence:return
                self.session.operator(d['mode'],d['verified'],d['connected'],d['activity'],
                    d['source'],d['sequence'],d['stamp'],now)
                return
    def session_state(self,poll=False,refresh=True):
        # A UI RPC or path callback may delay this decision past its earlier
        # read-only observation. Consume the latest ORIGINAL packet first.
        if refresh and getattr(self,'gait',None):self.poll_gait_authority(time.monotonic())
        # Evaluate after acquiring the same lock as the native observer.
        # A caller timestamp taken before waiting can predate a fresh packet.
        with self.session.lock:
            now=time.monotonic()
            return self.session.poll(now)if poll else self.session.status(now)
    def poll_session(self):
        with self.lock:
            now=time.monotonic()
            if getattr(self,'gait',None) is not None:self.poll_gait_authority(now)
            now=time.monotonic()
            if self.simulation and self.virtual_operator:
                v=self.virtual_operator;self.virtual_sequence+=1
                self.session.operator(v['mode'],True,v['connected'],v['activity'],self.virtual_source,self.virtual_sequence,now,now)
            status=self.session_state(poll=True)
            gait=getattr(self,'gait',None)
            if gait:
                state=gait.status()
                if (state['cancel_pending'] or state['phase'] in ('failed','disconnected')) and self.session.phase in ('starting','executing','replanning'):
                    self.session.cancel();self.session._notice('步态已撤销导航控制，请重新确认目标')
                gait.update(self.gate,self.session.phase=='executing' and not status['force_navigation_zero'])
                if self.session.phase in ('manual','paused','stopped','cancelled','blocked','arrived') and gait.phase not in ('stopped','idle','disconnected'):
                    gait.stop()
                self.acknowledge_if_quiet(self.session.cancel_generation)
            status=self.session_state()
            if status['cancel_pending']:
                self.stop_actions()
            if self.session.phase=='replanning':self.session.reroute(time.monotonic(),False,not self.collision_stop and self.health_ready())
            if self.session.phase in ('starting','executing','replanning'):
                if not self.health_ready():self.session.cancel();self.session._notice('地面、定位或传感器未就绪，导航已停止')
                current=self.session_state()
                self.op(armed=not current['force_navigation_zero'],paused=current['force_navigation_zero'],arm_generation=self.session.generation)
    def stop_actions(self):
        gait=getattr(self,'gait',None)
        if gait and gait.phase!='stopped':gait.stop()
        self.op(armed=False,paused=True)
        # Cancellation is acknowledged by terminal action results, never merely
        # because a cancel RPC was sent. Unknown acceptance keeps the latch.
        for handle in (self.goal_handle,self.plan_handle):
            if handle is not None and handle not in self.cancel_sent_handles:
                self.cancel_sent_handles.append(handle);handle.cancel_goal_async()
        if self.goal_handle is None and self.plan_handle is None and not self.pending_nav_accept and not self.pending_plan_accept:
            self.session.acknowledge_cancel(self.session.cancel_generation)
            if gait:gait.acknowledge_cancel()

    def wait_future(self,future,timeout=15):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:time.sleep(.02)
        if not future.done():raise TimeoutError('服务响应超时')
        return future.result()
    def wait_planner_active(self,timeout=8.):
        # The action endpoint may accept requests while on_activate is still
        # waiting for the first map transform/costmap. Queue that request here
        # instead of returning a misleading "starting point in lethal space".
        client=self.create_client(GetState,'/planner_server/get_state')
        end=time.monotonic()+timeout
        try:
            while time.monotonic()<end:
                remaining=end-time.monotonic()
                if remaining<=0:break
                if not client.wait_for_service(timeout_sec=min(.25,remaining)):continue
                try:response=self.wait_future(client.call_async(GetState.Request()),min(1.,remaining))
                except TimeoutError:continue
                if response.current_state.id==3:return
                time.sleep(min(.05,max(0.,end-time.monotonic())))
            raise TimeoutError('规划器和代价地图仍在初始化，请稍后再试')
        finally:self.destroy_client(client)
    def op(self,**values):
        m=String();m.data=json.dumps(values);self.operator.publish(m)
    def stop(self):
        with self.lock:
            if hasattr(self,'reception') and self.reception.engine.phase in ('dispatching','navigating','presenting','guest_wait','dwell','rerouting','waiting_route'):self.reception.engine.cancel()
            self.session.cancel();self.stop_actions()
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
    def start(self,mode,name,controller,resume=False,region=None):
        if mode not in ('mapping','navigation') or controller not in ('rpp','mppi'):raise ValueError('模式或控制器无效')
        if not SAFE_NAME.fullmatch(name):raise ValueError('地图名仅允许字母、数字、下划线和连字符')
        map_file=self.runtime/'maps'/name/'mapped.yaml'
        if mode=='navigation' and not map_file.is_file():raise ValueError('请先保存该地图')
        if mode=='navigation':validate_map_metadata(map_file)
        resume_pose=None
        if resume:
            if name!=self.map_name:raise ValueError('当前定位属于另一张地图，请先载入所选地图再继续扫描')
            if mode!='mapping' or not all((map_file.parent/f).is_file()for f in ('mapped.data','mapped.posegraph')):
                raise ValueError('这张地图缺少续扫所需的位姿图和扫描数据')
            resume_pose=self.reception_pose()
            if resume_pose is None:raise ValueError('缺少当前地图坐标，请在地图上设置续扫起点')
        if region is not None and not resume:raise ValueError('局部更新需要从原地图续扫')
        with self.lock:
            self.stop_process()
            self.grid=None;self.navigation_grid=None;self._overlay_key=None;self.gate={};self.buffer.clear()
            from configure import config
            import yaml
            params=self.runtime/'nav2.yaml';configured=config(controller,minimum_speed=self.minimum_speed,footprint_vertices=self.footprint,odometry_model=self.odometry_model,gait_profile=self.gait_profile,stop_side_margin=float(os.environ.get('P2_NAV_STOP_SIDE_MARGIN','.20')),planning_side_padding=float(os.environ.get('P2_NAV_PLANNING_SIDE_PADDING','.05')),planning_envelope=os.environ.get('P2_NAV_PLANNING_ENVELOPE','stop_clearance'),footprint_padding=float(os.environ.get('P2_NAV_FOOTPRINT_PADDING','.05')))
            configured['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_topic']='/p2_nav/reception_map'
            for kind in ('global_costmap','local_costmap'):
                cm=configured[kind][kind]['ros__parameters'];cm['filters']=['authored_keepout']
                cm['authored_keepout']={'plugin':'nav2_costmap_2d::KeepoutFilter','enabled':True,
                    'filter_info_topic':'/p2_nav/keepout_filter_info','transform_tolerance':.1}
            params.write_text(yaml.safe_dump(configured,sort_keys=False))
            from process_scheduling import workflow_command
            args=workflow_command(ROOT,mode,self.simulation,params,map_file,bool(self.gait),
                                  map_file.with_suffix('')if resume else None,resume_pose)
            self.process=subprocess.Popen(args,cwd=ROOT,stdout=open(self.runtime/'workflow.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
            self.mode=mode;self.map_name=name;self.controller=controller;self.path=[];self.goal=None
            self.reception_voice.auto_prepare(self.reception_catalog.read(name))
            self.map_update_region=region
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
        import tempfile,shutil
        target=self.runtime/'maps'/name;target.parent.mkdir(parents=True,exist_ok=True)
        temporary=Path(tempfile.mkdtemp(prefix='.save-',dir=target.parent));stem=str(temporary/'mapped')
        try:return self._save_map_version(name,target,temporary,stem)
        finally:shutil.rmtree(temporary,ignore_errors=True)
    def _save_map_version(self,name,target,temporary,stem):
        import shutil
        for service,typ in [('/map_saver/save_map',SaveMap),('/slam_toolbox/serialize_map',SerializePoseGraph)]:
            client=self.create_client(typ,service)
            if not client.wait_for_service(timeout_sec=5):raise ValueError('建图服务尚未就绪')
            req=typ.Request()
            if typ is SaveMap:configure_save_request(req,stem)
            else:req.filename=stem
            response=self.wait_future(client.call_async(req),25)
            if not (response.result if typ is SaveMap else response.result==0):raise ValueError('地图保存失败')
            self.destroy_client(client)
        validate_map_metadata(temporary/'mapped.yaml')
        patch=None
        with self.map_library.lock:
            if (target/'mapped.yaml').is_file():
                self.map_library.snapshot(name,'before_save')
                if self.map_update_region is not None:
                    from reception_maps import local_patch
                    patch=local_patch(target,temporary,self.map_update_region)
            target.mkdir(parents=True,exist_ok=True)
            from reception_maps import FILES
            for filename in FILES:
                if (temporary/filename).is_file():os.replace(temporary/filename,target/filename)
            version=self.map_library.snapshot(name,'local_update'if patch else 'mapping_save')
        self.last_action='地图与位姿图已保存：'+name
        return {'saved':name,'files':[p.name for p in target.iterdir()],'version':version,'local_update':patch}
    def checked_pose(self,data,*,require_free=True):
        x,y,yaw=(float(data[k]) for k in ('x','y','yaw'))
        if not all(math.isfinite(v) for v in (x,y,yaw)):raise ValueError('坐标必须是有限数值')
        grid=getattr(self,'navigation_grid',None)or self.grid
        if grid is None:raise ValueError('地图尚未收到')
        ix=int(math.floor((x-grid.info.origin.position.x)/grid.info.resolution))
        iy=int(math.floor((y-grid.info.origin.position.y)/grid.info.resolution))
        if not 0<=ix<grid.info.width or not 0<=iy<grid.info.height:raise ValueError('目标在地图外')
        cell=grid.data[iy*grid.info.width+ix]
        if require_free and not 0<=cell<20:raise ValueError('该点为未知区域或障碍，请选择已观察地面')
        pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=self.get_clock().now().to_msg()
        pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
        return pose
    def set_pose(self,data):
        self.stop();pose=self.checked_pose(data,require_free=False);msg=PoseWithCovarianceStamped();msg.header=pose.header;msg.pose.pose=pose.pose
        msg.pose.covariance[0]=.04;msg.pose.covariance[7]=.04;msg.pose.covariance[35]=.04
        self.initial.publish(msg);self.last_action='已提交初始位置，等待定位稳定'
        return {'initial_pose_set':True}
    def acknowledge_if_quiet(self,generation):
        if self.goal_handle is None and self.plan_handle is None and not self.pending_nav_accept and not self.pending_plan_accept:
            self.session.acknowledge_cancel(generation)
            gait=getattr(self,'gait',None)
            if gait:gait.acknowledge_cancel()
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
                if self.session.started(generation,time.monotonic()):self.op(armed=True,paused=False,speed=self.speed_value,arm_generation=generation)
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
        raw_route=data.get('_route_goals')
        if raw_route is not None and (not isinstance(raw_route,list)or not 2<=len(raw_route)<=100):raise ValueError('连续途经点数量无效')
        route_poses=[self.checked_pose(p)for p in raw_route]if raw_route else []
        if raw_route and any(abs(float(raw_route[-1][k])-float(data[k]))>1e-6 for k in ('x','y','yaw')):raise ValueError('连续路线终点与任务终点不一致')
        heading_mode=data.get('heading_mode','pose')
        if heading_mode not in ('pose','position'):raise ValueError('未知目标朝向模式')
        if heading_mode=='position' and self.gait_profile!='p2-flat-model':raise ValueError('当前配置不支持位置目标规划')
        if not self.gate.get('localized'):raise ValueError('定位尚未就绪，请先设置初始位置')
        with self.lock:
            generation=self.session.prepare(data,time.monotonic());self.path=[];self.goal=None;self.feedback={}
            self.goal_heading_mode=heading_mode;self.goal_route=route_poses
            self.pending_plan_accept=True
            self.op(armed=False,paused=True)
        try:self.wait_planner_active()
        except (ValueError,TimeoutError):
            with self.lock:
                self.pending_plan_accept=False;self.session.planned(generation,0,time.monotonic());self.acknowledge_if_quiet(generation)
            raise
        planner=self.planner_through if route_poses else self.planner
        if not planner.wait_for_server(timeout_sec=3):
            with self.lock:
                self.pending_plan_accept=False;self.session.planned(generation,0,time.monotonic());self.acknowledge_if_quiet(generation)
            raise ValueError('规划服务尚未就绪')
        if route_poses:
            request=ComputePathThroughPoses.Goal();request.goals=route_poses
        else:
            request=ComputePathToPose.Goal();request.goal=pose
        request.planner_id='PositionOnly' if heading_mode=='position' else 'GridBased'
        with self.lock:
            if self.session.generation!=generation or self.session.phase!='planning':
                self.pending_plan_accept=False;self.acknowledge_if_quiet(generation);raise ValueError('规划已取消')
            future=planner.send_goal_async(request)
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
            if '_mission_epoch' in data and not self.reception.engine.valid(data['_mission_epoch']):
                raise ValueError('导览任务已取消或被人工接管')
            gait=getattr(self,'gait',None);ready=self.execution_ready()
            reason=gait.status().get('blockers','') if gait and not ready else ''
            self.session.start(generation,time.monotonic(),ready,self.health_ready(),execution_reason=reason)
            self.goal_generation=generation;pose=self.checked_pose(self.session.goal)
            self.pending_nav_accept=True
        gait=getattr(self,'gait',None)
        if gait:
            try:gait.arm(generation)
            except (ValueError,TimeoutError):
                with self.lock:
                    self.pending_nav_accept=False;self.session.cancel();self.stop_actions();self.acknowledge_if_quiet(generation)
                raise
        route_poses=getattr(self,'goal_route',[])
        navigator=self.nav_through if route_poses else self.nav
        if not navigator.wait_for_server(timeout_sec=3):
            self.pending_nav_accept=False;self.session.cancel();self.stop_actions();self.acknowledge_if_quiet(generation);raise ValueError('导航服务未就绪')
        if route_poses:
            goal=NavigateThroughPoses.Goal();goal.poses=route_poses
            goal.behavior_tree=str(Path(__file__).with_name('forward_position_through_poses.xml' if getattr(self,'goal_heading_mode','pose')=='position' else 'forward_through_poses.xml'))
        else:
            goal=NavigateToPose.Goal();goal.pose=pose
            if getattr(self,'goal_heading_mode','pose')=='position':goal.behavior_tree=str(Path(__file__).with_name('forward_position_navigation.xml'))
        from recovery_tree import recovery_tree
        base_tree=getattr(goal,'behavior_tree','') or str(Path(__file__).with_name('forward_navigation.xml'))
        goal.behavior_tree=recovery_tree(base_tree,self.runtime/'recovery-trees',self.session.replan_timeout)
        with self.lock:
            self.bt_goal_source_ns=self.get_clock().now().nanoseconds;self.bt_recovery_report={}
            if self.session.generation!=generation or self.session.phase!='starting':
                self.pending_nav_accept=False;self.acknowledge_if_quiet(generation);raise ValueError('开始已取消')
            future=navigator.send_goal_async(goal,feedback_callback=lambda f:self.on_feedback(f,generation))
        try:self.wait_future(future,5)
        except TimeoutError:
            self.session.cancel();self.op(armed=False,paused=True)
            future.add_done_callback(lambda f:self.track_acceptance('nav',generation,f));raise
        handle=self.track_acceptance('nav',generation,future)
        if handle is None or not handle.accepted:
            self.session.cancel();self.acknowledge_if_quiet(generation);raise ValueError('导航请求被拒绝')
        if self.session.phase!='executing':raise ValueError('开始过程中已取消或手动接管')
        return {'accepted':True,'generation':generation,'hardware_execution_enabled':bool(gait and gait.status()['navigation_owned'])}
    def reception_pose(self):
        if not self.health_ready():return None
        try:
            t=self.buffer.lookup_transform('map','base_link',rclpy.time.Time()).transform;q=t.rotation
            return [t.translation.x,t.translation.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z)]
        except Exception:return None
    def capture_reception_trace(self):
        from reception_mission import BUSY
        now=time.monotonic();mission=self.reception.engine.status()
        if mission['phase']in BUSY:self.trace_active_until=now+2.
        if now>self.trace_active_until or not mission.get('run_id'):return
        sample={'kind':'telemetry','run_id':mission['run_id'],'map_id':self.reception.map_id(),
                'wall_time':time.time(),'capture_monotonic_ns':time.monotonic_ns(),
                'mission_phase':mission['phase'],'session_phase':self.session.phase,'pose':None,
                'pose_valid':self.health_ready(),'collision_stop':self.collision_stop,
                'goal_generation':self.goal_generation,'path_revision':self.path_revision,
                'control_recovery':self.bt_recovery_report,'path_source':self.path_source_report,'planning_evidence':self.flight.status(),'feedback':self.feedback,'speed_zones':self.speed_zone_status,
                'gate':{k:self.gate.get(k)for k in ('evaluated_monotonic_ns','velocity_source_monotonic_ns','reason',
                    'command','body_velocity_feedback','velocity_stages','scan_age','odom_age','localized','ground_ok')},
                'session_notice':self.session_state().get('notice')}
        try:
            t=self.buffer.lookup_transform('map','base_link',rclpy.time.Time());q=t.transform.rotation
            sample.update(pose=[t.transform.translation.x,t.transform.translation.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z)],
                          pose_source_ros_ns=t.header.stamp.sec*10**9+t.header.stamp.nanosec)
        except Exception:sample['pose_valid']=False
        if self.scan_preview:
            sample['scan']={**self.scan_preview,'points':self.scan_preview['points'][::12],
                            'age_s':(self.get_clock().now().nanoseconds-self.scan_preview['source_ns'])/1e9}
        if self.gait:
            g=self.gait.status();sample['native']={k:v for k,v in g.items()if k not in ('request_token','socket')}
        self.reception.history.append(sample)
    def cancel_reception_leg(self,epoch):
        with self.lock:
            if not self.reception.engine.valid(epoch):return False
            self.session.cancel();self.stop_actions()
        deadline=time.monotonic()+8.
        while True:
            if not self.reception.engine.valid(epoch):return False
            if not self.session_state()['navigation_permitted']:return False
            with self.lock:
                quiet=not self.session.cancel_pending and self.goal_handle is None and self.plan_handle is None and not self.pending_nav_accept and not self.pending_plan_accept
            if quiet:return True
            if time.monotonic()>deadline:raise ValueError('上段导航取消回执未完成，尚未发送新路线')
            time.sleep(.05)
    def on_door_observation(self,msg):
        if self.door_source!='external':return
        try:
            document=self.reception.observation_document();self.reception.decisions.bind(document)
            data=json.loads(msg.data)
            if data.get('door_id') not in {d['id'] for d in document['doors']}:return
            self.reception.decisions.observe(data,time.monotonic(),document['policy']['clear_hold'])
        except (TypeError,ValueError,KeyError):return
    def reception_speech_result(self):
        try:
            result=json.loads((self.runtime/'reception-speech-result.json').read_text())
            if result.get('epoch')!=self.reception.engine.epoch:return None
            return result
        except (OSError,ValueError):return None
    def reception_status(self,full=False):
        document=self.reception_catalog.read(self.reception.map_id()) if full else self.reception_catalog.summary(self.reception.map_id())
        result={'map_id':self.reception.map_id(),'revision':document['revision'],'mission':self.reception.engine.status(),'voice_progress':self.reception_voice.summary(self.reception.map_id()),'doors':self.reception.decisions.states(self.reception.observation_document(),time.monotonic()),'route_report':self.reception.route_report,'door_evidence':self.door_worker.summary,'door_overlay':self.overlay_report,'door_scan_rejections':dict(self.door_scan_rejections)}
        if full:result.update(document=document,voice=self.reception_voice.status(document))
        return result
    def reception_save(self,data):
        if not isinstance(data,dict):raise ValueError('导览配置必须是对象')
        operation=data.get('operation','save')
        if operation=='trace_runs':return self.reception.history.runs(data['map_id'])
        if operation=='trace_read':return self.reception.history.trace(data['map_id'],data['run_id'],data.get('after',0),data.get('limit',200))
        if operation.startswith('bundle_'):return self.bundle_transfer.request(data)
        if operation=='map_versions':return {'versions':self.map_library.versions(data['map_id'])}
        if operation=='map_snapshot':return self.map_library.snapshot(data['map_id'],'operator_snapshot')
        if operation=='map_restore':
            if data['map_id']==self.map_name:self.stop_process();self.mode='idle'
            return self.map_library.restore(data['map_id'],data['version'])
        if operation=='map_load':return self.start('navigation',data['map_id'],self.controller)
        if operation=='map_resume':return self.start('mapping',data['map_id'],self.controller,True,data.get('region'))
        if operation=='map_save':return self.save_map(data['map_id'])
        if operation=='pose_correct':
            if data.get('map_id',self.map_name)!=self.map_name:raise ValueError('校正点属于另一张地图，请先载入所选地图')
            return self.set_pose(data['pose'])
        if operation=='nearby':
            from reception_annotations import nearby_candidates
            grid=getattr(self,'navigation_grid',None)or self.grid
            if grid is None:raise ValueError('地图尚未收到')
            m={'width':grid.info.width,'height':grid.info.height,'resolution':grid.info.resolution,
               'origin':[grid.info.origin.position.x,grid.info.origin.position.y],'cells':grid.data}
            return {'candidates':nearby_candidates(m,data['target'],self.footprint or [[.275,.275],[.275,-.275],[-.275,-.275],[-.275,.275]],data.get('radius',1.))}
        if operation=='versions':
            if set(data)!={'operation','map_id'}:raise ValueError('内容历史读取字段无效')
            return self.reception_catalog.versions(data['map_id'])
        if operation=='restore':
            if set(data)!={'operation','map_id','source_revision','expected_revision'}:raise ValueError('内容恢复字段无效')
            document=self.reception_catalog.restore(data['map_id'],data['source_revision'],data['expected_revision'])
            self.reception_voice.auto_prepare(document);return document
        if operation=='history':
            if set(data)-{'operation','map_id','before','limit'}:raise ValueError('历史读取字段无效')
            self.reception_catalog.read(data.get('map_id')or self.reception.map_id())
            return self.reception.history.read(data.get('map_id')or self.reception.map_id(),data.get('before'),data.get('limit',100))
        if operation=='stage':return self.reception_transfer.receive(data)
        if operation=='read':
            if set(data)-{'operation','map_id'}:raise ValueError('导览读取字段无效')
            map_id=data.get('map_id')or self.reception.map_id()
            document=self.reception_catalog.read(map_id)
            return {'document':document,'voice':self.reception_voice.status(document)}
        if operation=='cloud_read':
            if set(data)!={'operation','map_id'}:raise ValueError('三维读取字段无效')
            return self.reception_cloud.read(data['map_id'])
        if operation=='cloud_save':
            if set(data)!={'operation','cloud'}:raise ValueError('三维保存字段无效')
            return self.reception_cloud.save(data['cloud'])
        if operation=='prepare_voice':
            if set(data)!={'operation'}:raise ValueError('语音准备字段无效')
            return self.reception_voice.prepare(self.reception_catalog.read(self.reception.map_id()))
        if operation!='save' or set(data)-{'operation','document','expected_revision'}:raise ValueError('导览保存字段无效')
        document=self.reception_catalog.save(data['document'],data['expected_revision'])
        self.reception_voice.auto_prepare(document);return document
    def reception_api_goal(self,data):
        if 'reception_command' in data:raise ValueError('开始任务应通过开始接口提交')
        if self.reception.engine.phase in ('dispatching','navigating','presenting','guest_wait','dwell','rerouting','waiting_route'):
            self.stop()
        return self.set_goal(data)
    def reception_api_confirm(self,data):
        if 'reception_command' in data:
            if set(data)!={'reception_command'}:raise ValueError('导览命令不能与单点开始混合提交')
            return self.reception.command(data['reception_command'])
        return self.confirm_goal(data)
    def on_feedback(self,feedback,generation=None):
        f=feedback.feedback;self.feedback={'distance_remaining':f.distance_remaining,'recoveries':f.number_of_recoveries}
        self.feedback['generation']=generation if generation is not None else self.goal_generation
        if hasattr(f,'number_of_poses_remaining'):self.feedback['poses_remaining']=int(f.number_of_poses_remaining)
    def speed(self,value,velocity_scale=None):
        v=float(value)
        if not math.isfinite(v) or not self.minimum_speed<=v<=self.maximum_speed:raise ValueError(f'此步态配置的速度范围为 {self.minimum_speed:.2f}–{self.maximum_speed:.2f} m/s')
        scale=getattr(self,'velocity_scale',1.) if velocity_scale is None else velocity_scale
        if type(scale) not in (int,float) or scale not in (.5,1.):raise ValueError('速度倍率仅支持 0.5 或 1.0')
        self.speed_value=v;self.velocity_scale=float(scale);self.publish_speed()
        return {'speed_m_s':v,'velocity_scale':self.velocity_scale,'effective_speed_m_s':v*self.velocity_scale}
    def publish_speed(self):
        factor=1.;active=[]
        if hasattr(self,'annotation_layer'):
            from footprint import DEFAULT
            radius=max(math.hypot(x,y)for x,y in (self.footprint or DEFAULT))
            factor,active=self.annotation_layer.speed(self.reception.observation_document().get('annotations',[]),self.reception_pose(),radius)
        self.speed_zone_status={'factor':factor,'active':active}
        self.op(speed=self.speed_value*factor,velocity_scale=getattr(self,'velocity_scale',1.))
        limit=SpeedLimit();limit.percentage=False;limit.speed_limit=self.speed_value*factor;self.limiter.publish(limit)
    def status(self,include_cells=True):
        from feedback_status import read_neck_feedback
        head_feedback=read_neck_feedback(os.getenv('P2_NAV_NECK_FEEDBACK',''),getattr(self,'robot_identity',None))
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
        grid=getattr(self,'navigation_grid',None)or self.grid
        if getattr(self,'gait',None):self.poll_gait_authority(time.monotonic())
        return {'robot_name':self.robot_name,'gait_profile':self.gait_profile,'request_token':self.request_token,'mode':self.mode,'simulation':self.simulation,'controller':self.controller,'message':self.last_action,
            'minimum_speed':self.minimum_speed,'maximum_speed':self.maximum_speed,'velocity_scale':self.velocity_scale,
            'effective_speed_cap':self.speed_value*self.velocity_scale,'simulation_kind':'native_dynamics' if os.getenv('P2_NAV_NATIVE_EXEC') else 'planar',
            'process_alive':self.process is not None and self.process.poll() is None,'gate':self.gate,'pose':pose,
            'session':self.session_state(),'map_revision':self.display_revision(),'neck':head_feedback,
            'execution_ready':self.execution_ready(),'gait_transport':getattr(self,'gait',None).status() if getattr(self,'gait',None) else None,'map':None if grid is None else {'width':grid.info.width,'height':grid.info.height,'resolution':grid.info.resolution,
                'origin':[grid.info.origin.position.x,grid.info.origin.position.y],**({'cells':list(grid.data)} if include_cells else {})},
            'path':self.path,'goal':self.goal,'feedback':self.feedback,'speed_zones':self.speed_zone_status,
            'scan_overlay':None if not self.scan_preview else dict(self.scan_preview,age_s=(self.get_clock().now().nanoseconds-self.scan_preview['source_ns'])/1e9),
            **({'truth':self.truth} if self.simulation else {}),
            'maps':sorted(p.parent.name for p in (self.runtime/'maps').glob('*/mapped.yaml')),
            'hardware_execution_enabled':bool(getattr(self,'gait',None) and self.gait.status()['navigation_owned']),
            'audio_feedback':self.audio.status(),
            'reception':self.reception_status(),'control_recovery':self.bt_recovery_report,'path_source':self.path_source_report,'planning_evidence':self.flight.status(),
            'scope':'平面导航仿真' if self.simulation else ('雷达中心坐标导航；定位、传感器和步态状态见实时数据' if self.gate.get('navigation_reference')=='lidar_projection' else ('实机步态接口已连接；坐标与控制状态见实时数据' if self.gait else '实机导航预演'))}

def console_executor(node):
    return ConsoleExecutors(node)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--simulation',action='store_true');parser.add_argument('--port',type=int,default=8930)
    parser.add_argument('--minimum-speed',type=float,default=.03)
    parser.add_argument('--footprint-profile',default='')
    parser.add_argument('--odometry-model',choices=['differential','omni'],default='differential')
    parser.add_argument('--gait-handoff',action='store_true');parser.add_argument('--robot-profile',default='')
    parser.add_argument('--gait-profile',choices=['generic','p2-flat-model'],default='generic')
    parser.add_argument('--gait-socket',default='');parser.add_argument('--runtime',default='');args=parser.parse_args()
    if args.gait_socket:os.environ['P2_NAV_MOTION_BUFFER']='true'
    if args.gait_handoff or args.gait_profile=='p2-flat-model':os.environ['P2_NAV_GAIT_HANDOFF']='true'
    selected=args.robot_profile or os.getenv('P2_NAV_ROBOT_PROFILE','')
    if not selected and not args.simulation and (ROOT.parent/'config/robot.local.json').is_file():selected=str(ROOT.parent/'config/robot.local.json')
    if selected and args.simulation:raise ValueError('Physical robot profiles are not used by simulation; choose an independent runtime')
    if selected:
        import sys
        sys.path.insert(0,str(ROOT.parent))
        from robot_profile import load,check_machine,bind_runtime,workflow_paths,identity
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
    if not .03<=args.minimum_speed<=(.8 if args.gait_profile=='p2-flat-model' else .25):raise ValueError('Invalid minimum speed profile')
    from footprint import load as load_footprint
    vertices=load_footprint(args.footprint_profile) if args.footprint_profile else None
    if selected and args.footprint_profile:
        data=json.loads(Path(args.footprint_profile).read_text())
        if data.get('hardware_verified') is True:
            from robot_profile import check_calibration
            check_calibration(profile,data)
    rclpy.init();node=Console(runtime,args.simulation,args.minimum_speed,vertices,args.odometry_model,args.gait_profile,args.gait_socket);node.robot_name=profile['name'] if selected else None;node.robot_identity=identity(profile) if selected else None;executor=console_executor(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    from audio_feedback import save_resource
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*a):pass
        def send(self,body,status=200,content_type='application/json'):
            raw=body if isinstance(body,bytes) else json.dumps(body,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(status);self.send_header('Content-Type',content_type);self.send_header('Cache-Control','no-store')
            if 'gzip' in self.headers.get('Accept-Encoding','') and len(raw)>1024:
                raw=gzip.compress(raw);self.send_header('Content-Encoding','gzip')
            self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
        def do_GET(self):
            if urlsplit(self.path).path in ('/','/reception'):
                guided=urlsplit(self.path).path=='/reception' or parse_qs(urlsplit(self.path).query).get('view') in (['reception'],['operator'])
                from reception_ui import reception_html,console_html
                html=reception_html(ROOT)if guided else console_html(ROOT)
                self.send(html.encode(),content_type='text/html; charset=utf-8')
            elif urlsplit(self.path).path=='/api/status':self.send(node.status(include_cells=parse_qs(urlsplit(self.path).query).get('summary')!=['1']))
            elif urlsplit(self.path).path=='/api/reception':self.send(node.reception_status(full=True))
            elif self.path=='/api/map':self.send({'revision':node.display_revision(),'map':node.status()['map']})
            else:self.send({'error':'Not found'},404)
        def do_POST(self):
            try:
                if not secrets.compare_digest(self.headers.get('X-Nav-Token',''),node.request_token):
                    self.send({'error':'请刷新工作台后重试'},403);return
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=600000:raise ValueError('请求大小无效')
                d=json.loads(self.rfile.read(length))
                if not isinstance(d,dict):raise ValueError('请求必须是JSON对象')
                routes={'/api/audio':lambda:node.audio.update(d),'/api/start':lambda:node.start(d['mode'],d.get('name','office'),d.get('controller','rpp')),
                    '/api/sim_mapping_route':lambda:node.simulated_mapping_route(d['name']),
                    '/api/save':lambda:save_resource(node,d),'/api/goal':lambda:node.reception_api_goal(d),
                    '/api/confirm':lambda:node.reception_api_confirm(d),'/api/sim_operator':lambda:node.simulation_operator(d),
                    '/api/initialpose':lambda:node.set_pose(d),'/api/speed':lambda:node.speed(d['speed'],d.get('velocity_scale')),
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
        node.door_worker.close()
        node.reception.close()
        node.stop_process()
        if node.gait:node.gait.close()
        server.server_close();executor.shutdown();node.flight.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
