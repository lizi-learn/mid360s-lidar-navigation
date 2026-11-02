#!/usr/bin/env python3
"""Read sensors in their ROS domain; publish isolated navigation preview inputs.

base_link can explicitly represent the LiDAR center ground projection, or a
timestamped body transform. LiDAR mode never asserts body/joint calibration.
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
from scan_ranges import polar_ranges
from pose_history import PoseHistory
from std_msgs.msg import Bool,String
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from terrain import local_floor,classify,observed_front_support,forward_lower_surface,ground_decision
from navigation_reference import select_reference,reference_status
from terrain_history import SupportHistory

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
        self.pose_history=PoseHistory()
        self.rotation=np.asarray(alignment['rotation'])
        self.initial_height=float(alignment['initial_height_m'])
        if self.rotation.shape!=(3,3) or not np.allclose(self.rotation@self.rotation.T,np.eye(3),atol=1e-4):raise ValueError('Invalid gravity alignment')
        self.source_context=Context();self.sink_context=Context()
        rclpy.init(context=self.source_context,domain_id=args.source_domain)
        rclpy.init(context=self.sink_context,domain_id=args.target_domain)
        self.source=Node('navigation_sensor_reader',context=self.source_context)
        self.sink=Node('navigation_sensor_preview_adapter',context=self.sink_context)
        self.body_reference=BodyReference(args.body_reference) if args.body_reference else None
        self.navigation_reference=select_reference(getattr(args,'navigation_reference','auto'),bool(self.body_reference))
        self.ground_policy=getattr(args,'ground_policy','observed_support')
        ground_decision(self.ground_policy,False,False)
        self.support_history=SupportHistory() if getattr(args,'support_history',False) else None
        if self.body_reference:self.source.create_subscription(JointState,self.body_reference.topic,self.on_joints,qos_profile_sensor_data)
        self.source.create_subscription(PointCloud2,'/cloud_registered',self.on_cloud,qos_profile_sensor_data)
        self.source.create_subscription(Odometry,'/Odometry',self.on_odom,qos_profile_sensor_data)
        self.odom_pub=self.sink.create_publisher(Odometry,'/odom',10)
        self.scan_pub=self.sink.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.ground_pub=self.sink.create_publisher(Bool,getattr(args,'ground_topic','/p2_nav/ground_ok'),10)
        self.status_pub=self.sink.create_publisher(String,'/p2_nav/terrain_status',10)
        self.reference_pub=self.sink.create_publisher(String,'/p2_nav/reference_status',10)
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
        source_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        with self.lock:
            if self.odom is None or source_ns>=self.odom.header.stamp.sec*10**9+self.odom.header.stamp.nanosec:self.odom=msg
            try:self.pose_history.add(source_ns,(p.x,p.y,p.z),(q.x,q.y,q.z,q.w))
            except ValueError:pass  # Existing tick validation handles an invalid latest pose.
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
        # Validate the current pose/velocity independently of cloud processing.
        # This is a fresh decision about the ORIGINAL source stamps checked
        # above; it does not publish another pose or refresh a sensor timestamp.
        reference=reference_status(self.navigation_reference,
            body_verified=bool(self.body_reference and self.body_reference.verified),
            body_feedback=bool(self.body_reference))
        reference.update(velocity_ready=True,reference_source_stamp_s=stamp(odom),
            reference_source_age_s=now-stamp(odom),hardware_execution_enabled=False)
        message=String();message.data=json.dumps(reference);self.reference_pub.publish(message)
        if self.processed_stamp==stamp(cloud):return
        cloud_ns=cloud.header.stamp.sec*10**9+cloud.header.stamp.nanosec
        with self.lock:sample=self.pose_history.at(cloud_ns)
        # Wait for the measured bracket instead of attaching a newer pose to an
        # older cloud. The last source-stamped scan remains subject to its
        # unchanged expiry; this wait never renews it or inserts a new health gate.
        if sample is None:return
        try:
            cloud_imu=Rotation.from_quat(sample.quaternion).as_matrix()
            cloud_optical=self.rotation@(np.asarray(sample.position)+cloud_imu@np.array([-.011,-.02329,.04412]))
            cloud_origin=cloud_optical.copy();cloud_orientation=self.rotation@cloud_imu@np.diag([1.,-1.,-1.])
            if self.body_reference:
                body=self.body_reference.transform(rigid(self.rotation@cloud_imu,cloud_optical+np.array([0.,0.,self.initial_height])),cloud_ns*1e-9)
                cloud_origin=body[:3,3];cloud_orientation=body[:3,:3]
            cloud_yaw=math.atan2(cloud_orientation[1,0],cloud_orientation[0,0])
            # A source-stamped projection frame makes every consumer use exactly
            # this cloud pose. It does not change the latest odometry output.
            projection=TransformStamped();projection.header.stamp=cloud.header.stamp
            projection.header.frame_id='odom';projection.child_frame_id='laser_projection'
            projection.transform.translation.x=float(cloud_origin[0]);projection.transform.translation.y=float(cloud_origin[1])
            projection.transform.rotation.z=math.sin(cloud_yaw/2);projection.transform.rotation.w=math.cos(cloud_yaw/2)
            self.tf.sendTransform(projection)
            self.processed_stamp=stamp(cloud)
            points=xyz(cloud);points=points[np.isfinite(points).all(axis=1)]
            points=points@self.rotation.T;points[:,2]+=self.initial_height
            plane,report=local_floor(points,cloud_origin[:2],expected_height=float(cloud_optical[2]))
            _,obstacles=classify(points,cloud_origin[:2],plane)
            output=polar_ranges(points[obstacles,:2]-cloud_origin[:2],cloud_yaw)
            scan=LaserScan();scan.header.stamp=cloud.header.stamp;scan.header.frame_id='laser_projection'
            scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/len(output);scan.angle_max=-math.pi+(len(output)-1)*scan.angle_increment
            scan.range_min=.12;scan.range_max=14.;scan.scan_time=.1;scan.ranges=output.tolist();self.scan_pub.publish(scan)
            instant_observed=bool(observed_front_support(points,cloud_origin[:2],cloud_yaw,plane))
            observed=instant_observed;support_report={}
            if self.support_history:
                observed,support_report=self.support_history.update(points,cloud_origin[:2],cloud_yaw,plane,stamp(cloud),now)
            verified=bool(self.body_reference and self.body_reference.verified)
            lower_surface=forward_lower_surface(points,cloud_origin[:2],cloud_yaw,plane)
            ground.data=ground_decision(self.ground_policy,observed,lower_surface) and (verified if self.body_reference else True);self.ground_pub.publish(ground)
            report.update(ground_ok=ground.data,ground_policy=self.ground_policy,lower_surface_observed=lower_surface,
                near_support_advisory=not observed,finite_scan_bins=int(np.isfinite(output).sum()),
                source_age_s=now-stamp(cloud),base_reference='timestamped body kinematics' if self.body_reference else 'LiDAR center ground projection',
                velocity_ready=True,velocity_window_s=self.velocity.window,observed_front_support=observed,instant_observed_front_support=instant_observed,
                reference_source_stamp_s=stamp(odom),hardware_execution_enabled=False)
            report.update(reference_status(self.navigation_reference,body_verified=verified,body_feedback=bool(self.body_reference)))
            report.update(projection_pose_stamp_ns=cloud_ns,projection_pose_bracket_ns=list(sample.bracket_ns),
                latest_odom_minus_cloud_ns=odom.header.stamp.sec*10**9+odom.header.stamp.nanosec-cloud_ns,
                projection_frame='laser_projection',scan_bins=len(output))
            report.update(support_report)
        except ValueError as e:
            if self.support_history:self.support_history.clear()
            self.ground_pub.publish(ground);report={'error':str(e),'ground_ok':False,'hardware_execution_enabled':False}
        msg=String();msg.data=json.dumps(report);self.status_pub.publish(msg)
    def run(self):
        sink_executor=SingleThreadedExecutor(context=self.sink_context);sink_executor.add_node(self.sink)
        try:sink_executor.spin()
        except KeyboardInterrupt:pass
        finally:
            sink_executor.shutdown()
            self.executor.shutdown();self.source.destroy_node();self.sink.destroy_node()
            rclpy.shutdown(context=self.source_context);rclpy.shutdown(context=self.sink_context)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source-domain',type=int,default=188);p.add_argument('--target-domain',type=int,default=194);p.add_argument('--alignment',required=True)
    p.add_argument('--ground-topic',default='/p2_nav/ground_ok');p.add_argument('--robot-profile',default='');p.add_argument('--body-reference',default='');p.add_argument('--velocity-window',type=float,default=.4)
    p.add_argument('--navigation-reference',choices=['auto','lidar_projection','body_kinematics','preview'],default='auto')
    p.add_argument('--support-history',action='store_true',help='Use up to250ms of original registered ground observations')
    p.add_argument('--ground-policy',choices=['observed_support','flat_floor'],default='observed_support')
    Adapter(p.parse_args()).run()
