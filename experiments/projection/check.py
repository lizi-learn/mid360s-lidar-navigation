#!/usr/bin/env python3
"""Actual two-domain ROS adapter regression with delayed synthetic cloud stamps.

Only sensor/tf topics in caller-specified local domains; no robot or motion API.
"""
import argparse,json,math,os,signal,subprocess,sys,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,PointField,LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener

ROOT=Path(__file__).resolve().parents[2]
def set_stamp(header,ns):header.stamp.sec,header.stamp.nanosec=divmod(int(ns),10**9)
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--adapter',default=str(ROOT/'navigation/live_adapter.py'));ap.add_argument('--source-domain',type=int,default=219);ap.add_argument('--target-domain',type=int,default=220);ap.add_argument('--expect',choices=['correct','legacy'],default='correct');args=ap.parse_args()
    out=Path(args.output).resolve().parent
    if out.exists() and any(out.iterdir()):raise ValueError('Use a new evidence directory')
    out.mkdir(parents=True,exist_ok=True)
    alignment=out/'synthetic-alignment.json';alignment.write_text(json.dumps({'rotation':np.eye(3).tolist(),'initial_height_m':1.2}))
    command=[sys.executable,args.adapter,'--alignment',str(alignment),'--source-domain',str(args.source_domain),'--target-domain',str(args.target_domain),'--navigation-reference','lidar_projection','--ground-policy','flat_floor']
    log=(out/'adapter.log').open('w');child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    a=Context();b=Context();rclpy.init(context=a,domain_id=args.source_domain);rclpy.init(context=b,domain_id=args.target_domain)
    source=Node('projection_synthetic_source',context=a);sink=Node('projection_independent_witness',context=b)
    cloud_pub=source.create_publisher(PointCloud2,'/cloud_registered',qos_profile_sensor_data);odom_pub=source.create_publisher(Odometry,'/Odometry',qos_profile_sensor_data)
    executor=SingleThreadedExecutor(context=b);executor.add_node(sink)
    pending=[];scans=[];reports=[];tf=Buffer(node=sink);listener=TransformListener(tf,sink)
    sink.create_subscription(LaserScan,'/scan',lambda m:pending.append(m),qos_profile_sensor_data)
    sink.create_subscription(String,'/p2_nav/terrain_status',lambda m:reports.append(json.loads(m.data)),10)
    floor=np.array([[x,y,-1.2]for x in np.linspace(-3.5,4.,48)for y in np.linspace(-3.,3.,40)],dtype=np.float32)
    # One fixed above-floor reflector gives an unambiguous reconstruction error.
    points=np.vstack([floor,np.array([[6.,1.,-.3]],dtype=np.float32)])
    sent=set();observed=[];start=None;report={};error=None
    def spin():
        for _ in range(12):executor.spin_once(timeout_sec=0.)
        for message in list(pending):
            stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
            try:t=tf.lookup_transform('odom',message.header.frame_id,rclpy.time.Time.from_msg(message.header.stamp)).transform
            except Exception:continue
            pending.remove(message);q=t.rotation;angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));valid=np.flatnonzero(np.isfinite(message.ranges))
            if len(valid)!=1:continue
            i=int(valid[0]);theta=angle+message.angle_min+i*message.angle_increment;distance=message.ranges[i]
            point=[t.translation.x+distance*math.cos(theta),t.translation.y+distance*math.sin(theta)]
            observed.append({'source_ns':stamp,'source_was_published':stamp in sent,'frame':message.header.frame_id,'bins':len(message.ranges),'x':point[0],'y':point[1],'error_m':math.hypot(point[0]-6.,point[1]-1.)})
    try:
        deadline=time.monotonic()+12
        while time.monotonic()<deadline and (cloud_pub.get_subscription_count()<1 or odom_pub.get_subscription_count()<1):spin();time.sleep(.02)
        if cloud_pub.get_subscription_count()<1 or odom_pub.get_subscription_count()<1:raise RuntimeError('Adapter subscriptions not discovered')
        start=time.time_ns();last_cloud=0.;end=time.monotonic()+3.5
        while time.monotonic()<end:
            if child.poll()is not None:raise RuntimeError('Adapter exited')
            ns=time.time_ns();elapsed=(ns-start)/1e9
            message=Odometry();set_stamp(message.header,ns);message.header.frame_id='camera_init';message.child_frame_id='body'
            message.pose.pose.position.x=.4*elapsed;message.pose.pose.orientation.z=math.sin(.8*elapsed/2);message.pose.pose.orientation.w=math.cos(.8*elapsed/2);odom_pub.publish(message)
            if time.monotonic()-last_cloud>=.10:
                last_cloud=time.monotonic();stamp=ns-100_000_000;message=PointCloud2();set_stamp(message.header,stamp);message.header.frame_id='camera_init';message.height=1;message.width=len(points);message.fields=[PointField(name=k,offset=4*i,datatype=PointField.FLOAT32,count=1)for i,k in enumerate(['x','y','z'])];message.is_bigendian=False;message.point_step=12;message.row_step=12*len(points);message.is_dense=True;message.data=points.tobytes();sent.add(stamp);cloud_pub.publish(message)
            spin();time.sleep(.02)
        drain=time.monotonic()+.3
        while time.monotonic()<drain:spin();time.sleep(.01)
        if len(observed)<12:raise RuntimeError('Insufficient original-stamped reconstructed scans: '+str(len(observed)))
        errors=[x['error_m']for x in observed];passed=(max(errors)<.04 if args.expect=='correct'else np.median(errors)>.20)
        if not all(x['source_was_published']for x in observed):passed=False
        if args.expect=='correct'and not all(x['frame']=='laser_projection'and x['bins']==720 for x in observed):passed=False
        timings=[d for d in reports if 'projection_pose_stamp_ns'in d]
        if args.expect=='correct'and (not timings or not all(
            math.isfinite(d.get('processing_duration_ms',float('nan')))and d['processing_duration_ms']>=0 and
            d.get('source_age_s',-1)>=d.get('input_source_age_s',float('inf')) and
            d.get('adapter_receive_to_report_ms',-1)>=d['processing_duration_ms']
            for d in timings)):passed=False
        report={'passed':bool(passed),'expect':args.expect,'scans':len(observed),'mean_error_m':float(np.mean(errors)),'median_error_m':float(np.median(errors)),'max_error_m':max(errors),'all_original_cloud_stamps':all(x['source_was_published']for x in observed),'input_cloud_delay_s':.1,'input_translation_m_s':.4,'input_yaw_rad_s':.8,'observed_projection_reports':[{k:v for k,v in d.items()if k in ['projection_pose_stamp_ns','projection_pose_bracket_ns','latest_odom_minus_cloud_ns','projection_frame','scan_bins']}for d in reports if 'projection_pose_stamp_ns'in d], 'hardware_commands':0,'scope':'Actual ROS2 Adapter subprocess and independent TF witness; synthetic stationary reflector/ground and delayed messages. No robot/gait or physical acceptance.'}
    except Exception as e:report={'passed':False,'error':str(e),'hardware_commands':0,'scans':len(observed),'scope':'Isolated synthetic ROS source/adapter witness; failure retained.'}
    finally:
        if child.poll()is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        executor.shutdown();source.destroy_node();sink.destroy_node();rclpy.shutdown(context=a);rclpy.shutdown(context=b);log.close()
        (out/'observations.json').write_text(json.dumps(observed,indent=2));Path(args.output).write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items()if k!='observed_projection_reports'}),flush=True)
    return 0 if report.get('passed')else 1
if __name__=='__main__':sys.exit(main())
