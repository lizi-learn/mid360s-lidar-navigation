#!/usr/bin/env python3
"""Optional body-motion buffer upstream of the unchanged collision monitor."""
import json,math,time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.msg import CollisionMonitorState
from std_msgs.msg import String
from motion_buffer import MotionBuffer

class MotionBufferNode(Node):
    def __init__(self):
        super().__init__('gait_motion_buffer');self.buffer=MotionBuffer();self.target=(0.,0.,0.)
        self.measured=(0.,0.,0.);self.command_time=0.;self.odom_stamp=0.
        self.armed=False;self.paused=True;self.collision=True;self.reference=False;self.reference_time=0.
        self.output=self.create_publisher(Twist,'/p2_nav/cmd_buffered',10)
        self.create_subscription(Twist,'/p2_nav/cmd_raw',self.command,10)
        self.create_subscription(Odometry,'/odom',self.odom,10)
        self.create_subscription(String,'/p2_nav/operator',self.operator,10)
        self.create_subscription(String,'/p2_nav/terrain_status',self.terrain,10)
        self.create_subscription(CollisionMonitorState,'/p2_nav/collision_state',self.collision_state,10)
        self.create_timer(.02,self.tick)
    def command(self,m):self.target=(m.linear.x,m.linear.y,m.angular.z);self.command_time=time.monotonic()
    def odom(self,m):
        self.odom_stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        self.measured=(m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z)
    def operator(self,m):
        try:
            d=json.loads(m.data)
            if 'armed' in d:self.armed=d['armed'] is True
            if 'paused' in d:self.paused=d['paused'] is True
        except (ValueError,TypeError):self.armed=False
    def terrain(self,m):
        try:
            d=json.loads(m.data);self.reference=d.get('body_reference_ready') is True and d.get('kinematic_feedback_ready') is True
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
def main():
    rclpy.init();node=MotionBufferNode()
    try:rclpy.spin(node)
    finally:node.output.publish(Twist());node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
