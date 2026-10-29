#!/usr/bin/env python3
"""Replay derived scans to an isolated SLAM Toolbox domain using recorded time."""
import argparse,json,math,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from slam_toolbox.srv import SaveMap,SerializePoseGraph
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster,Buffer,TransformListener

def stamp(t):
    from builtin_interfaces.msg import Time
    msg=Time();msg.sec=int(t);msg.nanosec=int((t-int(t))*1e9);return msg

class Replay(Node):
    def __init__(self):
        super().__init__('recorded_virtual_scan_replay')
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.clock_pub=self.create_publisher(Clock,'/clock',10)
        self.scan_pub=self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.odom_pub=self.create_publisher(Odometry,'/odom',10)
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.grid=None;self.transforms=[]
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,'/map',lambda m:setattr(self,'grid',m),qos)
        t=TransformStamped();t.header.frame_id='base_link';t.child_frame_id='laser';t.transform.rotation.w=1.;self.static.sendTransform(t)
    def spin(self,duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=min(.02,max(0.,end-time.monotonic())))
    def send(self,t,pose,ranges):
        clock=Clock();clock.clock=stamp(t);self.clock_pub.publish(clock)
        transform=TransformStamped();transform.header.stamp=stamp(t);transform.header.frame_id='odom';transform.child_frame_id='base_link'
        transform.transform.translation.x=float(pose[0]);transform.transform.translation.y=float(pose[1])
        transform.transform.rotation.z=math.sin(pose[2]/2);transform.transform.rotation.w=math.cos(pose[2]/2)
        self.tf.sendTransform(transform)
        odom=Odometry();odom.header=transform.header;odom.child_frame_id='base_link';odom.pose.pose.position.x=float(pose[0]);odom.pose.pose.position.y=float(pose[1]);odom.pose.pose.orientation=transform.transform.rotation;self.odom_pub.publish(odom)
        scan=LaserScan();scan.header.stamp=stamp(t);scan.header.frame_id='laser';scan.angle_min=-math.pi
        scan.angle_increment=2*math.pi/len(ranges);scan.angle_max=-math.pi+(len(ranges)-1)*scan.angle_increment
        scan.range_min=.12;scan.range_max=14.;scan.scan_time=.1;scan.ranges=[float(v) for v in ranges];self.scan_pub.publish(scan)
    def correction(self,t):
        try:
            v=self.buffer.lookup_transform('map','odom',rclpy.time.Time()).transform
            self.transforms.append([float(t),v.translation.x,v.translation.y,math.atan2(2*v.rotation.w*v.rotation.z,1-2*v.rotation.z*v.rotation.z)])
        except Exception:pass
    def save(self,directory):
        directory=Path(directory);directory.mkdir(parents=True,exist_ok=True);stem=str(directory/'mapped')
        responses={}
        for name,typ in [('/slam_toolbox/save_map',SaveMap),('/slam_toolbox/serialize_map',SerializePoseGraph)]:
            client=self.create_client(typ,name)
            if not client.wait_for_service(timeout_sec=10):raise TimeoutError('SLAM service unavailable')
            req=typ.Request()
            if typ is SaveMap:req.name.data=stem
            else:req.filename=stem
            f=client.call_async(req);end=time.monotonic()+30
            while not f.done() and time.monotonic()<end:self.spin(.05)
            if not f.done():raise TimeoutError('Map save did not finish')
            responses[name]=int(f.result().result)
        return responses

def main():
    p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--output',required=True);p.add_argument('--rate',type=float,default=3.)
    args=p.parse_args();data=np.load(args.input);rclpy.init();node=Replay();node.spin(2);wall=time.monotonic()
    try:
        timestamps=data['stamps'];poses=data['poses'];scans=data['ranges'];valid=0
        # Establish simulated time and TF before the first measurement.
        for _ in range(5):node.send(timestamps[0],poses[0],scans[0]);node.spin(.05)
        for i,(t,pose,ranges) in enumerate(zip(timestamps,poses,scans)):
            if np.isfinite(ranges).sum()<20:continue
            node.send(t,pose,ranges);valid+=1
            delay=float(timestamps[i+1]-t)/args.rate if i+1<len(timestamps) else .03
            node.spin(min(.1,max(.001,delay)))
            if i%25==0:node.correction(t)
            if i%500==0:print(json.dumps({'frame':i,'published':valid,'wall_s':time.monotonic()-wall}),flush=True)
        for j in range(25):node.send(timestamps[-1]+.1*(j+1),poses[-1],scans[-1]);node.spin(.05)
        node.correction(timestamps[-1]);responses=node.save(args.output)
        g=node.grid
        report={'input_scans':len(scans),'published_scans':valid,'duration_s':float(timestamps[-1]-timestamps[0]),
            'replay_rate':args.rate,'wall_s':time.monotonic()-wall,'save_responses':responses,
            'map':None if g is None else {'width':g.info.width,'height':g.info.height,'resolution':g.info.resolution,
                'known_cells':sum(v>=0 for v in g.data),'occupied_cells':sum(v>=65 for v in g.data)},
            'map_to_odom_corrections':node.transforms,'hardware_commands':0,
            'scope':'Office scan replay with recorded LIO odometry priors. Loop closing is enabled; no independent pose truth or physical navigation acceptance.'}
        Path(args.output,'replay-report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k!='map_to_odom_corrections'}),flush=True)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
