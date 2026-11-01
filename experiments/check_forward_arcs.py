"""Actual Nav2 controller component on a synthetic map; no plant or actuator I/O."""
import argparse,json,math,os,signal,subprocess,sys,time
from pathlib import Path
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import PoseStamped,TransformStamped,Twist
from nav_msgs.msg import OccupancyGrid,Odometry,Path as NavPath
from nav2_msgs.action import FollowPath
from nav2_msgs.srv import GetCostmap
from lifecycle_msgs.srv import ChangeState
from tf2_ros import StaticTransformBroadcaster

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'navigation'))
from configure import config

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);a=ap.parse_args()
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1' and int(os.environ.get('ROS_DOMAIN_ID','0'))>=200
    output=Path(a.output);output.parent.mkdir(parents=True,exist_ok=True)
    params=config(gait_profile='p2-flat-model',footprint_vertices=[[.275,.275],[.275,-.275],[-.275,-.275],[-.275,.275]])
    params['controller_server']['ros__parameters']['odom_topic']='/turn_arc_fixture/odom'
    local=params['local_costmap']['local_costmap']['ros__parameters']
    local.update(global_frame='fixture_odom',robot_base_frame='fixture_base',plugins=['static_layer'],width=8,height=8,
        static_layer={'plugin':'nav2_costmap_2d::StaticLayer','map_topic':'/turn_arc_fixture/map','map_subscribe_transient_local':True})
    paramfile=output.with_suffix('.params.yaml');paramfile.write_text(yaml.safe_dump(params))
    rclpy.init();n=rclpy.create_node('isolated_forward_arc_fixture')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    maps=n.create_publisher(OccupancyGrid,'/turn_arc_fixture/map',qos)
    odom=n.create_publisher(Odometry,'/turn_arc_fixture/odom',10)
    trace=[];results=[];phase={'wall':False,'boxed':False}
    n.create_subscription(Twist,'/turn_arc_fixture/command',lambda m:trace.append((time.monotonic(),m.linear.x,m.linear.y,m.angular.z)),10)
    tf=StaticTransformBroadcaster(n);t=TransformStamped();t.header.stamp=n.get_clock().now().to_msg()
    t.header.frame_id='fixture_odom';t.child_frame_id='fixture_base';t.transform.rotation.w=1.;tf.sendTransform(t)
    def publish():
        stamp=n.get_clock().now().to_msg()
        m=OccupancyGrid();m.header.stamp=stamp;m.header.frame_id='fixture_odom';m.info.resolution=.05
        m.info.width=m.info.height=240;m.info.origin.position.x=m.info.origin.position.y=-6.;m.info.origin.orientation.w=1.
        cells=[0]*(240*240)
        if phase['wall']:
            for iy in range(80,161):
                for ix in (131,132):cells[iy*240+ix]=100
        if phase['boxed']:
            for i in range(112,129):
                for j in (112,127):cells[i*240+j]=100;cells[j*240+i]=100
        m.data=cells;maps.publish(m)
        v=Odometry();v.header.stamp=stamp;v.header.frame_id='fixture_odom';v.child_frame_id='fixture_base';v.pose.pose.orientation.w=1.;odom.publish(v)
    n.create_timer(.1,publish);publish()
    log=output.with_suffix('.node.log').open('w')
    process=subprocess.Popen(['/opt/ros/humble/lib/nav2_controller/controller_server','--ros-args','--params-file',str(paramfile),
        '-r','cmd_vel:=/turn_arc_fixture/command'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def spin(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            assert process.poll() is None,'controller process exited'
            rclpy.spin_once(n,timeout_sec=.02)
    def complete(f,timeout=8):
        until=time.monotonic()+timeout
        while not f.done() and time.monotonic()<until:spin(.02)
        assert f.done(),'component operation timed out'
        return f.result()
    try:
        client=n.create_client(ChangeState,'/controller_server/change_state');assert client.wait_for_service(timeout_sec=10)
        for transition in (1,3):
            q=ChangeState.Request();q.transition.id=transition;assert complete(client.call_async(q)).success
        action=ActionClient(n,FollowPath,'/follow_path');assert action.wait_for_server(timeout_sec=8)
        spin(1.)
        for degrees in (-65,-45,-30,0,30,45,65,90):
            angle=math.radians(degrees);path=NavPath();path.header.frame_id='fixture_odom';path.header.stamp=n.get_clock().now().to_msg()
            for i in range(71):
                p=PoseStamped();p.header=path.header;p.pose.position.x=i*.05*math.cos(angle);p.pose.position.y=i*.05*math.sin(angle)
                p.pose.orientation.z=math.sin(angle/2);p.pose.orientation.w=math.cos(angle/2);path.poses.append(p)
            goal=FollowPath.Goal();goal.path=path;goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker'
            if degrees==45:wall_goal=goal
            handle=complete(action.send_goal_async(goal));assert handle.accepted
            spin(.35);start=time.monotonic();spin(.55)
            rows=[row for row in trace if row[0]>=start];assert rows,'missing actual controller commands'
            row={'heading_degrees':degrees,'samples':len(rows),'min_x':min(x[1] for x in rows),'max_x':max(x[1] for x in rows),
                 'max_abs_y':max(abs(x[2]) for x in rows),'min_z':min(x[3] for x in rows),'max_z':max(x[3] for x in rows)}
            results.append(row)
            assert row['max_abs_y']==0 and max(abs(row['min_z']),abs(row['max_z']))<=1.000001,row
            if abs(degrees)<=65:
                assert row['min_x']>=.599999,row
                if degrees:assert all(x[3]*degrees>0 for x in rows),row
            else:assert row['max_x']==0 and row['min_z']>.99,row
            complete(handle.cancel_goal_async());spin(.1)
        phase['wall']=True;publish();spin(1.)
        service=n.create_client(GetCostmap,'/local_costmap/get_costmap');assert service.wait_for_service(timeout_sec=5)
        m=complete(service.call_async(GetCostmap.Request())).map
        ix=int((.60-m.metadata.origin.position.x)/m.metadata.resolution);iy=int((0.-m.metadata.origin.position.y)/m.metadata.resolution)
        # Confirm the injected wall reached the actual controller costmap.
        costs=[m.data[iy*m.metadata.size_x+k] for k in range(ix-2,ix+3)]
        assert max(costs)>=253,{'wall_costs':costs}
        start=time.monotonic();handle=complete(action.send_goal_async(wall_goal));assert handle.accepted;spin(.8)
        rows=[r for r in trace if r[0]>=start]
        assert rows and all(r[1]==r[2]==0 and abs(r[3])<=1. for r in rows),rows
        # Forward motion is blocked, but an in-place pivot may be clear. Verify
        # its entire padded square against the ACTUAL map over the unchanged1.5s
        # horizon, rather than equating every nonzero yaw with failed avoidance.
        half=.275+local['footprint_padding'];res=m.metadata.resolution
        for command in rows:
            for k in range(151):
                yaw=command[3]*k*.01;c=math.cos(yaw);s=math.sin(yaw)
                for iy in range(int((-half*1.5-m.metadata.origin.position.y)/res),int((half*1.5-m.metadata.origin.position.y)/res)+1):
                    for ix in range(int((-half*1.5-m.metadata.origin.position.x)/res),int((half*1.5-m.metadata.origin.position.x)/res)+1):
                        if m.data[iy*m.metadata.size_x+ix]<254:continue
                        dx=m.metadata.origin.position.x+(ix+.5)*res;dy=m.metadata.origin.position.y+(iy+.5)*res
                        overlap=abs(dx)<=half*(abs(c)+abs(s))+res/2 and abs(dy)<=half*(abs(c)+abs(s))+res/2 and abs(c*dx+s*dy)<=half+res/2*(abs(c)+abs(s)) and abs(-s*dx+c*dy)<=half+res/2*(abs(c)+abs(s))
                        assert not overlap,{'unsafe_pivot':command,'yaw':yaw,'occupied_cell':[ix,iy]}
        results.append({'case':'confirmed_wall_blocks_forward_and_pivot_is_clear','wall_costs':costs,'samples':len(rows),'full_padded_body_sweep_clear':True,'sweep_horizon_s':1.5})
        complete(handle.cancel_goal_async());spin(.1)
        phase['boxed']=True;publish();spin(1.)
        box=complete(service.call_async(GetCostmap.Request())).map
        def actual_cost(x,y):
            ix=int((x-box.metadata.origin.position.x)/box.metadata.resolution);iy=int((y-box.metadata.origin.position.y)/box.metadata.resolution)
            return box.data[iy*box.metadata.size_x+ix]
        box_costs=[actual_cost(x,y)for x,y in[(.375,0),(-.375,0),(0,.375),(0,-.375)]]
        assert min(box_costs)>=254,box_costs
        start=time.monotonic();handle=complete(action.send_goal_async(wall_goal));assert handle.accepted;spin(.8)
        rows=[r for r in trace if r[0]>=start]
        assert rows and all(r[1]==r[2]==r[3]==0 for r in rows),rows
        results.append({'case':'confirmed_box_stops_actual_controller','box_costs':box_costs,'samples':len(rows),'all_zero':True})
        complete(handle.cancel_goal_async())
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
        log.close();n.destroy_node();rclpy.shutdown()
        output.write_text(json.dumps({'scope':'Actual Nav2 RPP plugin, synthetic static map and stationary pose; no gait, plant, robot hardware or actuator socket.',
            'seed':0,'parameters':params,'cases':results,'complete':len(results)==10,'hardware_commands':0},indent=2))
    assert len(results)==10
    print(json.dumps({'complete':True,'cases':len(results),'hardware_commands':0}))

if __name__=='__main__':main()
