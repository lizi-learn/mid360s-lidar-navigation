#!/usr/bin/env python3
"""Exercise actual Nav2/SLAM servers against the separate planar simulator."""
import argparse, json, math, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav2_msgs.msg import SpeedLimit
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import String

def angle(x):return (x+math.pi)%(2*math.pi)-math.pi

class Bench(Node):
    def __init__(self):
        super().__init__('navigation_validation_client')
        self.truth={};self.gate={};self.map=None;self.amcl=None
        self.commands=[];self.events=[];self.gate_history=[];self.raw=[]
        self.create_subscription(String,'/p2_nav/sim_truth',lambda m:setattr(self,'truth',json.loads(m.data)),10)
        self.create_subscription(String,'/p2_nav/gate_status',self.on_gate,10)
        self.create_subscription(Twist,'/p2_nav/cmd_safe',self.on_command,100)
        self.create_subscription(Twist,'/p2_nav/cmd_raw',lambda m:self.raw.append((time.monotonic(),m.linear.x,m.linear.y,m.angular.z)),100)
        from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,'/map',lambda m:setattr(self,'map',m),qos)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:setattr(self,'amcl',m),10)
        self.operator=self.create_publisher(String,'/p2_nav/operator',10)
        self.sim=self.create_publisher(String,'/p2_nav/sim_control',10)
        self.twist=self.create_publisher(Twist,'/p2_nav/cmd_raw',10)
        self.inject=self.create_publisher(Twist,'/p2_nav/cmd_collision',10)
        self.speed_pub=self.create_publisher(SpeedLimit,'/speed_limit',10)
        self.initial=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.nav=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.planner=ActionClient(self,ComputePathToPose,'compute_path_to_pose')
    def on_gate(self,m):self.gate=json.loads(m.data);self.gate_history.append((time.monotonic(),self.gate.copy()))
    def on_command(self,m):self.commands.append((time.monotonic(),m.linear.x,m.linear.y,m.angular.z))
    def wait(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=min(.05,max(0.,end-time.monotonic())))
    def future(self,f,timeout=10):
        end=time.monotonic()+timeout
        while not f.done() and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.05)
        if not f.done():raise TimeoutError('ROS future timed out')
        return f.result()
    def publish(self,publisher,values):
        msg=String();msg.data=json.dumps(values);publisher.publish(msg)
    def arm(self,speed=.25):
        self.publish(self.operator,{'armed':True,'paused':False,'speed':speed})
        limit=SpeedLimit();limit.percentage=False;limit.speed_limit=speed;self.speed_pub.publish(limit)
    def pose(self,x,y,yaw):
        p=PoseStamped();p.header.frame_id='map';p.header.stamp=self.get_clock().now().to_msg()
        p.pose.position.x=float(x);p.pose.position.y=float(y)
        p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2);return p
    def moving(self):
        cut=time.monotonic()-.3
        return any(abs(v)+abs(w)>.005 for t,v,_,w in self.commands if t>=cut)
    def event(self,name,**kw):
        item={'name':name,'time':time.monotonic(),**kw};self.events.append(item);print(json.dumps(item),flush=True)
    def navigate(self,goal,timeout=100,inject=None):
        assert self.nav.wait_for_server(timeout_sec=20.),'Navigator action unavailable'
        self.arm();self.wait(.3)
        handle=self.future(self.nav.send_goal_async(NavigateToPose.Goal(pose=self.pose(*goal))),5)
        if not handle.accepted:return {'accepted':False}
        result=handle.get_result_async();start=time.monotonic();done_inject=False
        while not result.done() and time.monotonic()-start<timeout:
            self.wait(.05)
            if inject and not done_inject and time.monotonic()-start>4:
                inject(self);done_inject=True
        if not result.done():self.future(handle.cancel_goal_async(),5);self.wait(1)
        status=result.result().status if result.done() else -1
        truth=self.truth.get('pose',[0,0,0]);error=math.hypot(truth[0]-goal[0],truth[1]-goal[1])
        item={'goal':goal,'status':status,'elapsed_s':time.monotonic()-start,'truth_goal_error_m':error,
              'truth_yaw_error_rad':abs(angle(truth[2]-goal[2])),'truth_pose':truth,'gate':self.gate}
        self.event('navigation_result',**item);return item

def mapping(bench,directory):
    bench.wait(4);bench.arm(.25)
    assert bench.truth,'Simulator did not publish truth'
    start=bench.truth['pose'];targets=[(4.5,2.5),(4.5,4.),(2.,4.),(2.,2.5)]
    for tx,ty in targets:
        end=time.monotonic()+80
        while time.monotonic()<end:
            bench.wait(.04);x,y,yaw=bench.truth['pose'];dist=math.hypot(tx-x,ty-y)
            if dist<.08:break
            err=angle(math.atan2(ty-y,tx-x)-yaw)
            cmd=Twist();cmd.angular.z=max(-.30,min(.30,err*1.8))
            cmd.linear.x=min(.25,dist*.7) if abs(err)<.2 else 0.
            bench.twist.publish(cmd)
        else:raise RuntimeError('Mapping route could not progress')
    # Return to the original heading without any lateral translation.
    for _ in range(250):
        bench.wait(.04);err=angle(start[2]-bench.truth['pose'][2]);cmd=Twist()
        if abs(err)<.035:break
        cmd.angular.z=max(-.30,min(.30,err*1.8));bench.twist.publish(cmd)
    bench.twist.publish(Twist());bench.wait(3)
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True);stem=str(directory/'mapped')
    for service,typ in [('/slam_toolbox/save_map',SaveMap),('/slam_toolbox/serialize_map',SerializePoseGraph)]:
        client=bench.create_client(typ,service);assert client.wait_for_service(timeout_sec=15.)
        req=typ.Request()
        if typ==SaveMap:req.name.data=stem
        else:req.filename=stem
        response=bench.future(client.call_async(req),20)
        bench.event('map_saved',service=service,result=int(response.result))
    grid=bench.map
    assert grid and (directory/'mapped.yaml').is_file() and (directory/'mapped.pgm').is_file()
    final=bench.truth['pose']
    return {'workflow':'SLAM Toolbox mapping -> map + serialized pose graph',
        'map_dimensions':[grid.info.width,grid.info.height],'resolution':grid.info.resolution,
        'known_cells':sum(v>=0 for v in grid.data),'occupied_cells':sum(v>=65 for v in grid.data),
        'closed_route_position_error_m':math.hypot(final[0]-start[0],final[1]-start[1]),
        'closed_route_yaw_error_rad':abs(angle(final[2]-start[2])),
        'artifacts':[p.name for p in directory.iterdir()],'truth':bench.truth}

def navigation(bench):
    bench.wait(6)
    assert bench.truth,'Simulator unavailable'
    bench.arm(.25);bench.wait(2)
    results=[]
    # Actual state-lattice paths + actual controller, all velocities pass the final gate.
    results.append(bench.navigate([4.7,2.5,0.],timeout=65))
    def speed_and_faults(b):
        b.arm(.10);b.wait(1.5);cut=time.monotonic()
        b.wait(1.)
        cap_commands=[abs(c[1]) for c in b.commands if c[0]>=cut]
        b.event('speed_cap',requested=.10,observed_max=max(cap_commands,default=0.),exercised=bool(cap_commands) and max(cap_commands)>.02,passed=bool(cap_commands) and .02<max(cap_commands)<=.100001)
        exercised=b.moving();b.publish(b.sim,{'ground_ok':False});begin=time.monotonic();b.wait(.4)
        ground_stop=[c for c in b.commands if c[0]>begin+.15]
        b.event('ground_missing_stop',exercised=exercised,passed=exercised and bool(ground_stop) and all(abs(c[1])+abs(c[3])<1e-8 for c in ground_stop))
        b.publish(b.sim,{'ground_ok':True});b.wait(.5)
        exercised=b.moving();b.publish(b.sim,{'drop_scan':True});begin=time.monotonic();b.wait(.8)
        stale=[c for c in b.commands if c[0]>begin+.65]
        b.event('scan_dropout_stop',exercised=exercised,passed=exercised and bool(stale) and all(abs(c[1])+abs(c[3])<1e-8 for c in stale))
        b.publish(b.sim,{'drop_scan':False});b.wait(1.);b.arm(.25)
        exercised=b.moving();b.publish(b.sim,{'drop_odom':True});begin=time.monotonic();b.wait(.85)
        stale=[c for c in b.commands if c[0]>begin+.65]
        b.event('odometry_dropout_stop',exercised=exercised,passed=exercised and bool(stale) and all(abs(c[1])+abs(c[3])<1e-8 for c in stale))
        b.publish(b.sim,{'drop_odom':False});b.wait(1.)
        exercised=b.moving();x,y,yaw=b.truth['pose'];b.publish(b.sim,{'obstacle':[x+.70*math.cos(yaw),y+.70*math.sin(yaw),.12]})
        begin=time.monotonic();b.wait(.7)
        stopped=[c for c in b.commands if c[0]>begin+.25]
        b.event('dynamic_obstacle_stop',exercised=exercised,passed=exercised and bool(stopped) and all(abs(c[1])+abs(c[3])<1e-8 for c in stopped))
        b.publish(b.sim,{'obstacle':None});b.wait(1.)
        begin=time.monotonic()
        for _ in range(8):
            bad=Twist();bad.linear.x=.1;bad.linear.y=.08;b.inject.publish(bad);b.wait(.07)
        rejected=any(s.get('reason')=='lateral_command_rejected' for t,s in b.gate_history if t>=begin)
        b.event('lateral_fault_rejected',passed=rejected and all(abs(c[2])<1e-8 for c in b.commands if c[0]>=begin))
    results.append(bench.navigate([9.3,2.5,0.],timeout=95,inject=speed_and_faults))
    # A goal behind the base must be reached by turning then moving forward.
    results.append(bench.navigate([7.2,2.2,math.pi],timeout=95))
    bench.publish(bench.operator,{'armed':False});bench.wait(.3)
    return {'workflow':'saved map -> AMCL -> state lattice -> controller -> smoother -> collision monitor -> forward gate -> planar plant',
        'goals':results,'events':bench.events,'all_goals_succeeded':all(x.get('status')==4 for x in results),
        'max_forward_m_s':max([c[1] for c in bench.commands],default=0.),
        'minimum_forward_m_s':min([c[1] for c in bench.commands],default=0.),
        'max_lateral_m_s':max([abs(c[2]) for c in bench.commands],default=0.),
        'max_angular_rad_s':max([abs(c[3]) for c in bench.commands],default=0.),
        'safe_command_messages':len(bench.commands),'truth':bench.truth,'raw_commands':bench.raw,
        'physics_scope':bench.truth.get('simulation','unknown')+'; no physical robot acceptance'}

def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['mapping','navigation']);p.add_argument('--directory',required=True)
    args=p.parse_args();rclpy.init();bench=Bench();report={}
    try:
        report=mapping(bench,args.directory) if args.mode=='mapping' else navigation(bench)
    except Exception as e:
        report={'error_type':type(e).__name__,'error':str(e),'events':bench.events,'truth':bench.truth,'gate':bench.gate}
        raise
    finally:
        bench.publish(bench.operator,{'armed':False});bench.twist.publish(Twist())
        report['gate_history']=bench.gate_history[-100:]
        report['commands']=bench.commands
        target=Path(args.directory);target.mkdir(parents=True,exist_ok=True)
        (target/(args.mode+'-report.json')).write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('commands','raw_commands','gate_history','events')},ensure_ascii=False),flush=True)
        bench.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
