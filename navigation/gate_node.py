#!/usr/bin/env python3
import json, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from gate import ForwardGate

class GateNode(Node):
    def __init__(self):
        super().__init__('forward_command_gate')
        self.simulation = self.declare_parameter('simulation', False).value
        self.require_localization = self.declare_parameter('require_localization', True).value
        self.gate = ForwardGate()
        self.last = time.monotonic();self.command_time = 0.
        self.scan_stamp = self.odom_stamp = None
        self.command = (0., 0., 0.);self.ground = False;self.ground_time = 0.;self.paused = False
        self.covariance_ok = False
        self.covariance = None
        self.buffer = Buffer();self.listener = TransformListener(self.buffer, self)
        self.output = self.create_publisher(Twist, '/p2_nav/cmd_safe' if self.simulation else '/p2_nav/cmd_preview', 10)
        self.status = self.create_publisher(String, '/p2_nav/gate_status', 10)
        self.create_subscription(Twist, '/p2_nav/cmd_collision', self.on_command, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Bool, '/p2_nav/ground_ok', self.on_ground, 10)
        self.create_subscription(String, '/p2_nav/operator', self.on_operator, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.create_timer(.05, self.tick)
        self.counter = 0

    @staticmethod
    def stamp(msg):return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    def on_command(self, msg):
        self.command = (msg.linear.x, msg.linear.y, msg.angular.z);self.command_time = time.monotonic()
    def on_scan(self, msg):self.scan_stamp = self.stamp(msg)
    def on_odom(self, msg):self.odom_stamp = self.stamp(msg)
    def on_ground(self, msg):self.ground = msg.data;self.ground_time = time.monotonic()
    def on_amcl(self, msg):
        c = msg.pose.covariance
        self.covariance = [float(c[i]) for i in (0,7,35)]
        self.covariance_ok = bool(all(math.isfinite(v) for v in (c[0],c[7],c[35])) and max(c[0],c[7]) < .25 and c[35] < .25)
    def on_operator(self, msg):
        try:
            v = json.loads(msg.data)
            if 'speed' in v:self.gate.set_speed(float(v['speed']))
            if 'armed' in v:self.gate.armed = bool(v['armed'])
            if 'paused' in v:self.paused = bool(v['paused'])
        except (ValueError, TypeError, KeyError):self.gate.armed = False

    def tick(self):
        now = time.monotonic();stamp = self.get_clock().now().nanoseconds * 1e-9
        dt = now - self.last;self.last = now
        localized = not self.require_localization
        age = None
        if self.require_localization:
            try:
                t = self.buffer.lookup_transform('map', 'odom', rclpy.time.Time())
                age = stamp - (t.header.stamp.sec + t.header.stamp.nanosec * 1e-9)
                localized = bool(-.8 < age < .8 and self.covariance_ok and self.buffer.can_transform('map', 'base_link', rclpy.time.Time()))
            except Exception:localized = False
        scan_age = stamp - self.scan_stamp if self.scan_stamp is not None else 999.
        odom_age = stamp - self.odom_stamp if self.odom_stamp is not None else 999.
        result = self.gate.step(self.command, dt, now - self.command_time, scan_age, odom_age,
            ground_ok=self.ground and now - self.ground_time < .5, localized=localized, paused=self.paused)
        out = Twist();out.linear.x, out.linear.y, out.angular.z = result;self.output.publish(out)
        self.counter += 1
        if self.counter % 2 == 0:
            status = String();status.data = json.dumps({'armed':self.gate.armed,'simulation':self.simulation,
                'hardware_execution_enabled':False,'speed_cap':self.gate.speed,'reason':self.gate.reason,
                'command':result,'scan_age':scan_age,'odom_age':odom_age,'localized':localized,
                'ground_ok':self.ground and now - self.ground_time < .5,
                'localization_covariance':self.covariance,'map_transform_age':age})
            self.status.publish(status)

def main():
    rclpy.init();node=GateNode()
    try:rclpy.spin(node)
    finally:node.output.publish(Twist());node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
