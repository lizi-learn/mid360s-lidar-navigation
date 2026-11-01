#!/usr/bin/env python3
"""Receive nested planner paths outside the heartbeat/UI Python process."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String
from path_view import encode_view

class PathView(Node):
    def __init__(self):
        super().__init__('workbench_path_view')
        self.output=self.create_publisher(String,'/p2_nav/path_view',10)
        self.create_subscription(Path,'/plan',self.path,10)
    def path(self,msg):
        points=[[p.pose.position.x,p.pose.position.y] for p in msg.poses]
        source_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        try:raw=encode_view(points,source_ns)
        except (ValueError,TypeError,KeyError):return
        out=String();out.data=raw;self.output.publish(out)

def main():
    rclpy.init();node=PathView()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
