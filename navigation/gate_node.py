#!/usr/bin/env python3
import json, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.msg import CollisionMonitorState
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
        self.handoff=None;self.intent=(0.,0.,0.);self.intent_time=0.;self.measured=None
        self.recovery=None;self.recovery_wait=False;self.collision_stop=False
        if self.declare_parameter('gait_handoff',False).value:
            from gait_handoff import GaitHandoff
            self.handoff=GaitHandoff()
            from clearance_recovery import ClearanceRecovery
            self.recovery=ClearanceRecovery()
            self.create_subscription(CollisionMonitorState,'/p2_nav/collision_state',self.on_collision_state,10)
            self.create_subscription(Twist,'/p2_nav/cmd_raw',self.on_intent,10)
        self.last = time.monotonic();self.command_time = 0.
        self.invalid_command_until=0.;self.invalid_command=(0.,0.,0.)
        self.scan_stamp = self.odom_stamp = None
        self.calibration_ready=False;self.calibration_time=0.
        self.command = (0., 0., 0.);self.ground = False;self.ground_time = 0.;self.paused = False
        self.covariance_ok = False
        self.covariance = None
        self.collision_source_valid=False;self.collision_source_time=0.
        self.arm_generation=-1;self.collision_fault_generation=-1
        self.buffer = Buffer();self.listener = TransformListener(self.buffer, self)
        self.output = self.create_publisher(Twist, '/p2_nav/cmd_safe' if self.simulation else '/p2_nav/cmd_preview', 10)
        self.status = self.create_publisher(String, '/p2_nav/gate_status', 10)
        self.create_subscription(Twist, '/p2_nav/cmd_collision', self.on_command, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Bool, '/p2_nav/ground_ok', self.on_ground, 10)
        self.create_subscription(String,'/p2_nav/terrain_status',self.on_terrain,10)
        self.create_subscription(Bool,'/p2_nav/collision_source_valid',self.on_collision_source,10)
        self.create_subscription(String, '/p2_nav/operator', self.on_operator, 10)
        localization_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, localization_qos)
        self.create_timer(.05, self.tick)
        self.counter = 0

    @staticmethod
    def stamp(msg):return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    def on_command(self, msg):
        self.command = (msg.linear.x, msg.linear.y, msg.angular.z);self.command_time = time.monotonic()
        if self.gate.command_fault(self.command):
            self.invalid_command=self.command;self.invalid_command_until=self.command_time+.75
    def on_intent(self,msg):
        self.intent=(msg.linear.x,msg.linear.y,msg.angular.z);self.intent_time=time.monotonic()
    def on_collision_state(self,msg):self.collision_stop=msg.action_type==CollisionMonitorState.STOP
    def on_collision_source(self,msg):
        self.collision_source_valid=msg.data is True;self.collision_source_time=time.monotonic()
        if not self.collision_source_valid:self.revoke_collision_authority()
    def revoke_collision_authority(self):
        self.gate.armed=False
        self.collision_fault_generation=max(self.collision_fault_generation,self.arm_generation)
    def on_scan(self, msg):self.scan_stamp = self.stamp(msg)
    def on_odom(self, msg):
        self.odom_stamp = self.stamp(msg);v=(msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z);self.measured=v if all(math.isfinite(x) for x in v) else None
    def on_ground(self, msg):self.ground = msg.data;self.ground_time = time.monotonic()
    def on_terrain(self,msg):
        try:
            d=json.loads(msg.data);self.calibration_ready=d.get('body_reference_ready') is True and d.get('kinematic_feedback_ready') is True
            self.calibration_time=time.monotonic()
        except (ValueError,TypeError):self.calibration_ready=False
    def on_amcl(self, msg):
        c = msg.pose.covariance
        self.covariance = [float(c[i]) for i in (0,7,35)]
        self.covariance_ok = bool(all(math.isfinite(v) for v in (c[0],c[7],c[35])) and max(c[0],c[7]) < .25 and c[35] < .25)
        if not all(math.isfinite(v) for v in self.covariance):self.covariance=None
    def on_operator(self, msg):
        try:
            v = json.loads(msg.data)
            if 'speed' in v:self.gate.set_speed(float(v['speed']))
            if 'armed' in v:
                generation=v.get('arm_generation')
                fresh_generation=(type(generation) is int and generation>=0
                    and generation>=self.arm_generation and generation>self.collision_fault_generation)
                self.gate.armed=bool(v['armed']) and fresh_generation
                if self.gate.armed:self.arm_generation=generation
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
        collision_source_ok=self.collision_source_valid and 0<=now-self.collision_source_time<.3
        if not collision_source_ok:self.revoke_collision_authority()
        # A subsequent valid message must not erase an unsafe input between ticks.
        command=self.invalid_command if now<self.invalid_command_until else self.command
        if self.recovery:
            clear=(collision_source_ok and not self.collision_stop and self.ground and now-self.ground_time<.5 and localized
                and -.05<=scan_age<=self.gate.limits.sensor_timeout and -.05<=odom_age<=self.gate.limits.sensor_timeout
                and 0<=now-self.command_time<=self.gate.limits.command_timeout and not self.gate.command_fault(command))
            self.recovery_wait=not self.recovery.step(now,clear)
        if self.handoff:
            if self.gate.command_fault(command):self.handoff.reset();self.handoff.phase='command_rejected'
            elif self.recovery_wait:self.handoff.reset();command=(0.,0.,0.)
            elif not self.gate.armed or self.paused:self.handoff.reset()
            else:command=self.handoff.step(command,self.intent,self.measured or (float('nan'),)*3,now,now-self.intent_time,odom_age)
        result = self.gate.step(command, dt, now - self.command_time, scan_age, odom_age,
            ground_ok=self.ground and now - self.ground_time < .5, localized=localized, paused=self.paused)
        out = Twist();out.linear.x, out.linear.y, out.angular.z = result;self.output.publish(out)
        self.counter += 1
        if True: #20Hz original evaluations; transport does not restamp cached health
            status = String();status.data = json.dumps({'armed':self.gate.armed,'simulation':self.simulation,
                'evaluated_monotonic_ns':time.monotonic_ns(),'data_fresh':collision_source_ok and -.05<=min(scan_age,odom_age) and max(scan_age,odom_age)<=self.gate.limits.sensor_timeout,'collision_source_valid':collision_source_ok,'calibration_ready':self.calibration_ready and now-self.calibration_time<.2,
                'hardware_execution_enabled':False,'speed_cap':self.gate.speed,'reason':self.gate.reason,
                'motion_phase':'waiting_clearance' if self.recovery_wait else self.handoff.phase if self.handoff else 'direct','body_velocity_feedback':self.measured,'command':result,'scan_age':scan_age,'odom_age':odom_age,'localized':localized,
                'ground_ok':self.ground and now - self.ground_time < .5,
                'localization_covariance':self.covariance,'map_transform_age':age})
            self.status.publish(status)

def main():
    rclpy.init();node=GateNode()
    try:rclpy.spin(node)
    finally:node.output.publish(Twist());node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
