#!/usr/bin/env python3
"""Optional body-motion buffer upstream of the unchanged collision monitor."""
import json,math,time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.msg import CollisionMonitorState
from std_msgs.msg import String
from motion_buffer import MotionBuffer, MeasuredMotionMean
from navigation_reference import reference_ready

class MotionBufferNode(Node):
    def __init__(self):
        super().__init__('gait_motion_buffer')
        native_ramp=self.declare_parameter('native_input_ramp',False).value
        self.buffer=MotionBuffer(accel=.4,decel=.8,angular_accel=.5,ramp_enabled=not native_ramp);self.target=(0.,0.,0.)
        self.motion_mean=MeasuredMotionMean(translation_window=.5 if native_ramp else 0.);self.measured=(0.,0.,0.);self.command_time=0.;self.odom_stamp=0.
        self.diagnostic=self.get_logger().info;self.diagnostic_time=0.;self.was_suppressed=False
        self.armed=False;self.paused=True;self.collision=True;self.reference=False;self.reference_time=0.
        self.output=self.create_publisher(Twist,'/p2_nav/cmd_buffered',10)
        self.create_subscription(Twist,'/p2_nav/cmd_raw',self.command,10)
        self.create_subscription(Odometry,'/odom',self.odom,10)
        self.create_subscription(String,'/p2_nav/operator',self.operator,10)
        self.create_subscription(String,'/p2_nav/terrain_status',self.terrain,10)
        self.create_subscription(String,'/p2_nav/reference_status',self.terrain,10)
        self.create_subscription(CollisionMonitorState,'/p2_nav/collision_state',self.collision_state,10)
        self.create_timer(.02,self.tick)
    def command(self,m):
        # RPP curvature corrections can exceed the configured gait turn range.
        # Bound finite yaw requests before the buffer contract; rejecting the
        # whole vector would repeatedly erase an otherwise valid forward input.
        w=m.angular.z
        limit=1.
        self.target=(m.linear.x,m.linear.y,max(-limit,min(limit,w)) if math.isfinite(w) else w)
        self.command_time=time.monotonic()
    def odom(self,m):
        self.odom_stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        self.measured=self.motion_mean.step((m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z),self.odom_stamp)
    def operator(self,m):
        try:
            d=json.loads(m.data)
            if 'armed' in d:self.armed=d['armed'] is True
            if 'paused' in d:self.paused=d['paused'] is True
        except (ValueError,TypeError):self.armed=False
    def terrain(self,m):
        try:
            d=json.loads(m.data);self.reference=reference_ready(d)
            self.reference_time=time.monotonic()
        except (ValueError,TypeError):self.reference=False
    def collision_state(self,m):self.collision=m.action_type==CollisionMonitorState.STOP
    def tick(self):
        now=time.monotonic();age=self.get_clock().now().nanoseconds/1e9-self.odom_stamp
        hard=(not self.armed or self.paused or self.collision or not 0<=now-self.command_time<=.3)
        out=self.buffer.step(self.target,self.measured,now,age,hard_stop=hard,
            # Nav2 raw zero has no stop-reason contract. Do not silently
            # turn a protective zero into another braking interval.
            normal_stop=False,
            frame_verified=self.reference and now-self.reference_time<.2)
        m=Twist();m.linear.x,m.linear.y,m.angular.z=out;self.output.publish(m)
        suppressed=any(abs(v)>1e-6 for v in self.target) and not any(abs(v)>1e-6 for v in out)
        diagnostic=getattr(self,'diagnostic',None)
        if diagnostic and now-getattr(self,'diagnostic_time',0.)>=1. and (suppressed or getattr(self,'was_suppressed',False)):
            diagnostic('Motion buffer evidence: '+json.dumps(dict(suppressed=suppressed,phase=self.buffer.phase,
                target=self.target,output=out,measured_mean=self.measured,odom_source_s=self.odom_stamp,
                odom_age_s=age,command_age_s=now-self.command_time,reference_age_s=now-self.reference_time,
                armed=self.armed,paused=self.paused,collision=self.collision,reference=self.reference),separators=(',',':')))
            self.diagnostic_time=now
        self.was_suppressed=suppressed
def main():
    rclpy.init();node=MotionBufferNode()
    try:rclpy.spin(node)
    finally:node.output.publish(Twist());node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
