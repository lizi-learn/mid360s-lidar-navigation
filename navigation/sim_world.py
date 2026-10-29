#!/usr/bin/env python3
"""Headless planar plant and ray-cast LiDAR, never a humanoid balance simulation."""
import json, math, time, os
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

RES = .05
def make_world():
    grid=np.zeros((200,240),dtype=np.uint8)
    grid[:3,:]=100;grid[-3:,:]=100;grid[:,:3]=100;grid[:,-3:]=100
    for x0,y0,x1,y1 in [(5.8,0,6.0,1.2),(5.8,3.8,6.0,7.4),(5.8,9.2,6.,10),
                         (3.4,5.,4.4,6.0),(8.0,5.4,9.,6.8),(9.4,8.,10.2,8.6)]:
        grid[int(y0/RES):int(y1/RES),int(x0/RES):int(x1/RES)]=100
    return grid

def save_truth(directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    grid=make_world();pixels=np.where(grid==100,0,254).astype(np.uint8)
    (directory/'truth.pgm').write_bytes(f'P5\n{grid.shape[1]} {grid.shape[0]}\n255\n'.encode()+np.flipud(pixels).tobytes())
    (directory/'truth.yaml').write_text('image: truth.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')

class World(Node):
    def __init__(self):
        super().__init__('planar_navigation_simulator')
        from footprint import checked,DEFAULT
        self.footprint=np.asarray(checked(json.loads(os.environ['P2_NAV_FOOTPRINT']) if os.getenv('P2_NAV_FOOTPRINT') else DEFAULT))
        self.grid=make_world();self.x=2.;self.y=2.5;self.yaw=0.;self.cmd=(0.,0.);self.cmd_at=0.
        self.last=time.monotonic();self.start=self.last;self.collision_count=0;self.travel=0.;self.path=[]
        self.native=None
        if os.environ.get('P2_NAV_NATIVE_EXEC'):
            from native_backend import NativeBackend
            self.native=NativeBackend()
        self.drop_scan=False;self.drop_odom=False;self.ground_ok=True;self.obstacle=None;self.pause=False
        self.odom=self.create_publisher(Odometry,'/odom',10)
        self.scan=self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.truth=self.create_publisher(String,'/p2_nav/sim_truth',10)
        self.ground=self.create_publisher(Bool,'/p2_nav/ground_ok',10)
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        t=TransformStamped();t.header.frame_id='base_link';t.child_frame_id='laser';t.transform.rotation.w=1.;self.static.sendTransform(t)
        self.create_subscription(Twist,'/p2_nav/cmd_safe',self.on_command,10)
        self.create_subscription(String,'/p2_nav/sim_control',self.on_control,10)
        self.create_timer(.02,self.step);self.create_timer(.1,self.sense)
        self.angles=np.linspace(-math.pi,math.pi,360,endpoint=False)
        self.radii=np.arange(.15,14.,RES)
    def on_command(self,msg):
        self.cmd=(float(msg.linear.x),float(msg.angular.z));self.cmd_at=time.monotonic()
        if self.native:self.native.send(*self.cmd)
    def on_control(self,msg):
        v=json.loads(msg.data)
        for k in ('drop_scan','drop_odom','ground_ok','pause'):
            if k in v:setattr(self,k,bool(v[k]))
        if 'obstacle' in v:self.obstacle=v['obstacle']
        if 'pose' in v:self.x,self.y,self.yaw=map(float,v['pose'])
    def occupied(self,x,y):
        ix=np.clip((x/RES).astype(int),0,self.grid.shape[1]-1)
        iy=np.clip((y/RES).astype(int),0,self.grid.shape[0]-1)
        result=self.grid[iy,ix]>50
        if self.obstacle is not None:
            ox,oy,r=self.obstacle;result|=(x-ox)**2+(y-oy)**2<r*r
        return result
    def footprint_collision(self,x,y,yaw):
        # Same rectangle as Nav2, densely sampled including its interior.
        lo,hi=self.footprint.min(axis=0),self.footprint.max(axis=0)
        xx,yy=np.meshgrid(np.linspace(lo[0],hi[0],max(11,int((hi[0]-lo[0])/.04)+1)),
                          np.linspace(lo[1],hi[1],max(13,int((hi[1]-lo[1])/.04)+1)))
        c,s=math.cos(yaw),math.sin(yaw)
        return bool(np.any(self.occupied(x+c*xx-s*yy,y+s*xx+c*yy)))
    def step(self):
        now=time.monotonic();dt=min(.05,now-self.last);self.last=now
        v,w=self.cmd if now-self.cmd_at<.35 and not self.pause else (0.,0.)
        actual_vy=0.
        if self.native:
            state,received=self.native.latest()
            if state is None or now-received>.25:return
            _,nx,ny,nyaw,nv,nvy,nw,nz=state
            x=nx+2.;y=ny+2.5;yaw=nyaw;v=nv;actual_vy=nvy;w=nw
            self.travel+=math.hypot(x-self.x,y-self.y)
            if self.footprint_collision(x,y,yaw):self.collision_count+=1
            self.x,self.y,self.yaw=x,y,yaw
        else:
            x=self.x+v*math.cos(self.yaw+w*dt/2)*dt;y=self.y+v*math.sin(self.yaw+w*dt/2)*dt;yaw=self.yaw+w*dt
            if self.footprint_collision(x,y,yaw):
                if abs(v)+abs(w)>1e-5:self.collision_count+=1
                v=w=0.
            else:self.travel+=math.hypot(x-self.x,y-self.y);self.x,self.y,self.yaw=x,y,yaw
        stamp=self.get_clock().now().to_msg()
        if not self.drop_odom:
            t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='odom';t.child_frame_id='base_link'
            t.transform.translation.x=self.x;t.transform.translation.y=self.y
            t.transform.rotation.z=math.sin(self.yaw/2);t.transform.rotation.w=math.cos(self.yaw/2);self.tf.sendTransform(t)
            o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='base_link'
            o.pose.pose.position.x=self.x;o.pose.pose.position.y=self.y;o.pose.pose.orientation=t.transform.rotation
            o.twist.twist.linear.x=v;o.twist.twist.linear.y=actual_vy;o.twist.twist.angular.z=w;self.odom.publish(o)
        b=Bool();b.data=self.ground_ok;self.ground.publish(b)
    def sense(self):
        if self.native:
            state,received=self.native.latest()
            if state is None or time.monotonic()-received>.25:
                b=Bool();b.data=False;self.ground.publish(b)
                msg=String();msg.data=json.dumps({'pose':[self.x,self.y,self.yaw],'pose_valid':False,'backend_alive':False,'simulation':'native backend unavailable; pose is stale'})
                self.truth.publish(msg);return
        if not self.drop_scan:
            angles=self.angles+self.yaw
            x=self.x+np.cos(angles[:,None])*self.radii[None,:]
            y=self.y+np.sin(angles[:,None])*self.radii[None,:]
            hits=self.occupied(x,y);any_hit=hits.any(axis=1)
            ranges=np.where(any_hit,self.radii[np.argmax(hits,axis=1)],np.inf)
            scan=LaserScan();scan.header.stamp=self.get_clock().now().to_msg();scan.header.frame_id='laser'
            scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/360;scan.angle_max=scan.angle_min+359*scan.angle_increment
            scan.range_min=.12;scan.range_max=14.;scan.scan_time=.1;scan.ranges=ranges.astype(float).tolist();self.scan.publish(scan)
        msg=String();msg.data=json.dumps({'pose':[self.x,self.y,self.yaw],'collision_attempts':self.collision_count,
            'travelled_m':self.travel,'command':self.cmd,'elapsed_s':time.monotonic()-self.start,
            'simulation':'native dynamics + ray-cast scan' if self.native else 'planar kinematics + ray-cast scan; not a humanoid dynamics validation'})
        self.truth.publish(msg)
    def destroy_node(self):
        if self.native:self.native.close()
        return super().destroy_node()

def main():
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--write-map');args,_=p.parse_known_args()
    if args.write_map:save_truth(args.write_map);return
    rclpy.init();node=World()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
