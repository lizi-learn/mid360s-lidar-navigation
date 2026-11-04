#!/usr/bin/env python3
"""Actual isolated Nav2 rolling-costmap comparison with partial scan coverage.

Synthetic map and rays only. No gait, navigation action, or hardware endpoint.
"""
import argparse,json,math,os,signal,subprocess,sys,time
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'navigation'))

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);args=ap.parse_args()
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.srv import GetCostmap,ClearEntireCostmap
    from configure import config
    rclpy.init();n=rclpy.create_node('synthetic_partial_scan_context');proc=None;obstacle=False
    mp=n.create_publisher(OccupancyGrid,'/p2_nav/reception_map',QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    sp=n.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
    tf=StaticTransformBroadcaster(n);transforms=[]
    for parent,child in [('map','odom'),('odom','base_link')]:
        t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp=n.get_clock().now().to_msg();t.transform.rotation.w=1.;transforms.append(t)
    tf.sendTransform(transforms)
    m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg();m.info.resolution=.05;m.info.width=160;m.info.height=160;m.info.origin.position.x=-4.;m.info.origin.position.y=-4.;m.info.origin.orientation.w=1.;m.data=[0]*25600
    def index(x,y):return int((y+4)/.05)*160+int((x+4)/.05)
    m.data[index(0.,1.5)]=-1
    for iy in range(160):m.data[iy*160+120]=100
    mp.publish(m)
    def scan():
        s=LaserScan();s.header.frame_id='base_link';s.header.stamp=n.get_clock().now().to_msg();s.angle_min=-math.pi;s.angle_increment=2*math.pi/720;s.angle_max=math.pi-s.angle_increment;s.range_min=.12;s.range_max=14.;s.ranges=[float('nan')if abs(-math.pi+i*s.angle_increment)<.3 or abs(-math.pi+i*s.angle_increment-math.pi/2)<.2 else 3. for i in range(720)]
        if obstacle:s.ranges[360]=.6
        sp.publish(s)
    timer=n.create_timer(.05,scan)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
    def request(name,typ,req):
        c=n.create_client(typ,name)
        end=time.monotonic()+8
        while not c.service_is_ready()and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
        assert c.service_is_ready(),name
        f=c.call_async(req);end=time.monotonic()+10
        while not f.done()and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
        assert f.done()and f.result()is not None,name
        result=f.result();n.destroy_client(c);return result
    def transition(id):
        q=ChangeState.Request();q.transition.id=id
        assert request('/controller_server/change_state',ChangeState,q).success
    def cell(x,y):
        g=request('/local_costmap/get_costmap',GetCostmap,GetCostmap.Request()).map;d=g.metadata
        ix=int((x-d.origin.position.x)/d.resolution);iy=int((y-d.origin.position.y)/d.resolution)
        return int(g.data[iy*d.size_x+ix])
    def obstacle_seen():
        g=request('/local_costmap/get_costmap',GetCostmap,GetCostmap.Request()).map;d=g.metadata
        # A laser endpoint is rasterized to its containing cell; read the
        # actual lethal cell within the half-diagonal quantization bound.
        matches=[]
        for i,c in enumerate(g.data):
            if c!=254:continue
            x=d.origin.position.x+(i%d.size_x+.5)*d.resolution
            y=d.origin.position.y+(i//d.size_x+.5)*d.resolution
            if math.hypot(x-.6,y)<=d.resolution/math.sqrt(2)+1e-5:matches.append([x,y])
        assert matches,'Synthetic obstacle endpoint was not marked lethal'
        return matches
    rows=[]
    try:
        for variant in ['baseline','candidate']:
            p=config(gait_profile='p2-flat-model',footprint_vertices=[[.275,.275],[.275,-.275],[-.275,-.275],[-.275,.275]],footprint_padding=0.)
            if variant=='baseline':p['local_costmap']['local_costmap']['ros__parameters']['plugins'].remove('static_layer')
            f=out.parent/(variant+'.yaml');f.write_text(yaml.safe_dump(p,sort_keys=False));log=(out.parent/(variant+'.log')).open('w')
            proc=subprocess.Popen(['/opt/ros/humble/lib/nav2_controller/controller_server','--ros-args','--params-file',str(f)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            spin(1.);transition(1);transition(3);spin(1.5)
            before=cell(.525,.025);wall=cell(2.025,1.025);unknown=cell(.025,1.525)
            if variant=='baseline':assert before==255,before
            else:
                assert before<254,before
                assert wall==254,wall
                assert unknown==255,unknown
            obstacle=True;spin(1.)
            hit=obstacle_seen()
            request('/local_costmap/clear_entirely_local_costmap',ClearEntireCostmap,ClearEntireCostmap.Request());spin(1.)
            retained=obstacle_seen()
            obstacle=False;spin(.5)
            request('/local_costmap/clear_entirely_local_costmap',ClearEntireCostmap,ClearEntireCostmap.Request());spin(1.)
            recovered=cell(.525,.025)
            assert recovered==(255 if variant=='baseline'else 0),recovered
            rows.append(dict(variant=variant,partial_scan_known_floor=before,static_wall=wall,static_unknown=unknown,real_obstacle=hit,real_obstacle_rebuilt_after_cache_reset=retained,recovered_floor=recovered))
            transition(4);transition(2);os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=8);proc=None;log.close();spin(1.)
        result=dict(passed=True,cases=rows,source='Actual Nav2 controller_server and rolling local costmap, fixed synthetic TF/map and partial height-slice rays',static_unknown_preserved=True,actual_obstacle_retained_after_reset=True,hardware_commands=0,navigation_actions=0)
        out.write_text(json.dumps(result,indent=2));print(json.dumps(result))
    finally:
        if proc is not None and proc.poll()is None:
            os.killpg(proc.pid,signal.SIGTERM)
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
