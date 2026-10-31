#!/usr/bin/env python3
"""Read sensors in their ROS domain; publish isolated navigation preview inputs.

base_link here is a LiDAR ground-projection proxy, NOT a calibrated robot base.
No motor, joystick, or vendor shared-memory interface is opened.
"""
import argparse,json,math,threading,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,LaserScan,JointState
from biped_reference import BodyReference,rigid
from odometry_velocity import PlanarVelocity
from std_msgs.msg import Bool,String
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from terrain import local_floor,classify,observed_front_support

def xyz(msg):
    fields={f.name:f.offset for f in msg.fields}
    dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,
        'offsets':[fields[k] for k in ('x','y','z')],'itemsize':msg.point_step})
    a=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step)).reshape(-1)
    return np.column_stack([a[k] for k in ('x','y','z')]).astype(float)

class Adapter:
    def __init__(self,args):
        self.lock=threading.Lock();self.cloud=None;self.odom=None;self.processed_stamp=None;self.published_odom_stamp=None
        self.alignment_path=Path(args.alignment);self.alignment_mtime=self.alignment_path.stat().st_mtime_ns
        self.reference_fault='';self.previous=None;self.velocity=PlanarVelocity(window=getattr(args,'velocity_window',.4))
        alignment=json.loads(Path(args.alignment).read_text())
        selected=getattr(args,'robot_profile','')
        if selected:
            import sys
            sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
            from robot_profile import load,check_machine,identity,check_calibration
            profile=load(selected);check_machine(profile)
            if alignment.get('robot_identity')!=identity(profile):raise ValueError('Sensor alignment belongs to another robot/installation')
            if args.body_reference:check_calibration(profile,json.loads(Path(args.body_reference).read_text()))
        self.rotation=np.asarray(alignment['rotation'])
        self.initial_height=float(alignment['initial_height_m'])
        if self.rotation.shape!=(3,3) or not np.allclose(self.rotation@self.rotation.T,np.eye(3),atol=1e-4):raise ValueError('Invalid gravity alignment')
        self.source_context=Context();self.sink_context=Context()
        rclpy.init(context=self.source_context,domain_id=args.source_domain)
        rclpy.init(context=self.sink_context,domain_id=args.target_domain)
        self.source=Node('navigation_sensor_reader',context=self.source_context)
        self.sink=Node('navigation_sensor_preview_adapter',context=self.sink_context)
        self.body_reference=BodyReference(args.body_reference) if args.body_reference else None
        if self.body_reference:self.source.create_subscription(JointState,self.body_reference.topic,self.on_joints,qos_profile_sensor_data)
        self.source.create_subscription(PointCloud2,'/cloud_registered',self.on_cloud,qos_profile_sensor_data)
        self.source.create_subscription(Odometry,'/Odometry',self.on_odom,qos_profile_sensor_data)
        self.odom_pub=self.sink.create_publisher(Odometry,'/odom',10)
        self.scan_pub=self.sink.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.ground_pub=self.sink.create_publisher(Bool,getattr(args,'ground_topic','/p2_nav/ground_ok'),10)
        self.status_pub=self.sink.create_publisher(String,'/p2_nav/terrain_status',10)
        self.tf=TransformBroadcaster(self.sink);self.static=StaticTransformBroadcaster(self.sink)
        t=TransformStamped();t.header.frame_id='base_link';t.child_frame_id='laser';t.transform.rotation.w=1.;self.static.sendTransform(t)
        # Forward each new source-stamped pose promptly. Do not add another
        # scan-period wait before TF, and do not renew cached pose timestamps.
        self.sink.create_timer(.02,self.tick)
        self.executor=SingleThreadedExecutor(context=self.source_context);self.executor.add_node(self.source)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
    def on_joints(self,msg):
        try:
            if len(msg.name)!=len(msg.position):raise ValueError("Joint names/positions length mismatch")
            self.body_reference.history.add(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,dict(zip(msg.name,msg.position)))
        except (ValueError,KeyError) as e:self.reference_fault="Joint feedback invalid: "+str(e)
    def on_cloud(self,msg):
        with self.lock:self.cloud=msg
    def on_odom(self,msg):
        with self.lock:self.odom=msg
    def tick(self):
        with self.lock:cloud,odom=self.cloud,self.odom
        ground=Bool();ground.data=False
        if cloud is None or odom is None:self.ground_pub.publish(ground);return
        now=self.sink.get_clock().now().nanoseconds*1e-9
        stamp=lambda m:m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        try:
            if self.alignment_path.stat().st_mtime_ns!=self.alignment_mtime:self.reference_fault='sensor reference changed; start a new mapping/localization session'
        except OSError:self.reference_fault='sensor reference unavailable; start a new mapping/localization session'
        if self.reference_fault:
            self.ground_pub.publish(ground)
            msg=String();msg.data=json.dumps({'error':self.reference_fault,'ground_ok':False,'hardware_execution_enabled':False});self.status_pub.publish(msg)
            return
        if max(now-stamp(cloud),now-stamp(odom))>.5 or min(now-stamp(cloud),now-stamp(odom))<-.05:
            self.ground_pub.publish(ground);return
        q=odom.pose.pose.orientation;p=odom.pose.pose.position
        values=np.array([p.x,p.y,p.z,q.x,q.y,q.z,q.w])
        if not np.isfinite(values).all() or not .8<np.linalg.norm(values[3:])<1.2:
            self.reference_fault='invalid odometry pose';self.ground_pub.publish(ground);return
        imu=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        origin=self.rotation@(np.array([p.x,p.y,p.z])+imu@np.array([-.011,-.02329,.04412]))
        optical=origin.copy()
        orientation=self.rotation@imu@np.diag([1.,-1.,-1.])
        if self.body_reference:
            try:
                world_lidar=rigid(self.rotation@imu,origin+np.array([0.,0.,self.initial_height]))
                body=self.body_reference.transform(world_lidar,stamp(odom))
                origin=body[:3,3];orientation=body[:3,:3]
            except (ValueError,OSError) as e:
                self.ground_pub.publish(ground)
                status=String();status.data=json.dumps({'error':str(e),'ground_ok':False,'body_reference_ready':False,'hardware_execution_enabled':False});self.status_pub.publish(status)
                return
        if self.previous is not None and stamp(odom)!=self.previous[0]:
            dt=stamp(odom)-self.previous[0];distance=np.linalg.norm(origin[:2]-self.previous[1])
            if dt<=0 or distance>max(.4,2.*dt):
                self.reference_fault='odometry discontinuity; recheck localization'
                self.ground_pub.publish(ground);return
        self.previous=(stamp(odom),origin[:2].copy())
        yaw=math.atan2(orientation[1,0],orientation[0,0])
        try:velocity=self.velocity.update(stamp(odom),(origin[0],origin[1],yaw))
        except ValueError as e:
            self.reference_fault='invalid navigation velocity: '+str(e);self.ground_pub.publish(ground);return
        if velocity is None:self.ground_pub.publish(ground);return
        if self.published_odom_stamp!=stamp(odom):
            t=TransformStamped();t.header.stamp=odom.header.stamp;t.header.frame_id='odom';t.child_frame_id='base_link'
            t.transform.translation.x=float(origin[0]);t.transform.translation.y=float(origin[1]);t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2);self.tf.sendTransform(t)
            out=Odometry();out.header=t.header;out.child_frame_id='base_link';out.pose.pose.position.x=float(origin[0]);out.pose.pose.position.y=float(origin[1]);out.pose.pose.orientation=t.transform.rotation
            out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.angular.z=velocity
            self.odom_pub.publish(out);self.published_odom_stamp=stamp(odom)
        if self.processed_stamp==stamp(cloud):return
        self.processed_stamp=stamp(cloud)
        try:
            points=xyz(cloud);points=points[np.isfinite(points).all(axis=1)]
            points=points@self.rotation.T;points[:,2]+=self.initial_height
            plane,report=local_floor(points,origin[:2],expected_height=float(optical[2]))
            _,obstacles=classify(points,origin[:2],plane)
            delta=points[obstacles,:2]-origin[:2]
            bearing=(np.arctan2(delta[:,1],delta[:,0])-yaw+math.pi)%(2*math.pi)-math.pi
            ranges=np.linalg.norm(delta,axis=1);indices=np.clip(((bearing+math.pi)/(2*math.pi)*360).astype(int),0,359)
            output=np.full(360,np.inf);np.minimum.at(output,indices,ranges)
            # Empty height-slice bins are unknown, not evidence of a clear ray.
            output[~np.isfinite(output)]=np.nan
            scan=LaserScan();scan.header.stamp=cloud.header.stamp;scan.header.frame_id='laser'
            scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/360;scan.angle_max=-math.pi+359*scan.angle_increment
            scan.range_min=.12;scan.range_max=14.;scan.scan_time=.1;scan.ranges=output.tolist();self.scan_pub.publish(scan)
            observed=bool(observed_front_support(points,origin[:2],yaw,plane))
            verified=bool(self.body_reference and self.body_reference.verified)
            ground.data=observed and (verified if self.body_reference else True);self.ground_pub.publish(ground)
            report.update(ground_ok=ground.data,finite_scan_bins=int(np.isfinite(output).sum()),
                source_age_s=now-stamp(cloud),base_reference='timestamped body kinematics' if self.body_reference else 'uncalibrated lidar ground projection',
                body_reference_ready=verified,kinematic_feedback_ready=bool(self.body_reference),velocity_ready=True,velocity_window_s=self.velocity.window,observed_front_support=observed,hardware_execution_enabled=False)
        except ValueError as e:
            self.ground_pub.publish(ground);report={'error':str(e),'ground_ok':False,'hardware_execution_enabled':False}
        msg=String();msg.data=json.dumps(report);self.status_pub.publish(msg)
    def run(self):
        sink_executor=SingleThreadedExecutor(context=self.sink_context);sink_executor.add_node(self.sink)
        try:sink_executor.spin()
        finally:
            sink_executor.shutdown()
            self.executor.shutdown();self.source.destroy_node();self.sink.destroy_node()
            rclpy.shutdown(context=self.source_context);rclpy.shutdown(context=self.sink_context)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source-domain',type=int,default=188);p.add_argument('--target-domain',type=int,default=194);p.add_argument('--alignment',required=True)
    p.add_argument('--ground-topic',default='/p2_nav/ground_ok');p.add_argument('--robot-profile',default='');p.add_argument('--body-reference',default='');p.add_argument('--velocity-window',type=float,default=.4)
    Adapter(p.parse_args()).run()
