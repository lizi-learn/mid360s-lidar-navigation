"""Actual native Nav2 trajectory monitor; synthetic points, no robot commands."""
import argparse,json,math,os,pathlib,signal,subprocess,sys,time
import yaml,rclpy
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.srv import ChangeState
from geometry_msgs.msg import Twist,TransformStamped,PolygonStamped,Point32
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import CollisionMonitorState
from tf2_ros import StaticTransformBroadcaster
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'navigation'))
from configure import config,apply_directional_stop

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--binary',required=True);ap.add_argument('--output',required=True);args=ap.parse_args()
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1' and int(os.environ.get('ROS_DOMAIN_ID','0'))>=200
    output=pathlib.Path(args.output);output.parent.mkdir(parents=True,exist_ok=True);paramfile=output.with_suffix('.params.yaml')
    body=[[.275,.275],[.275,-.275],[-.275,-.275],[-.275,.275]]
    paramfile.write_text(yaml.safe_dump(config(gait_profile='p2-flat-model',footprint_vertices=body,footprint_padding=0.)))
    apply_directional_stop(paramfile);m=yaml.safe_load(paramfile.read_text())['collision_monitor']['ros__parameters']
    prefix='/directional_stop_fixture';m.update(base_frame_id='fixture_base',odom_frame_id='fixture_odom',cmd_vel_in_topic=prefix+'/request',cmd_vel_out_topic=prefix+'/result',state_topic=prefix+'/state')
    m['stop_pub_timeout']=30. # Fixture holds repeated STOP outputs throughout all consecutive fault cases.
    m['scan']['topic']=prefix+'/scan';m['SweptZone']['footprint_topic']=prefix+'/footprint';m['StopZone']['visualize']=m['SweptZone']['visualize']=False
    paramfile.write_text(yaml.safe_dump({'collision_monitor':{'ros__parameters':m}}))
    rclpy.init();n=rclpy.create_node('isolated_directional_stop_fixture');cmd=n.create_publisher(Twist,prefix+'/request',10)
    scans=n.create_publisher(LaserScan,prefix+'/scan',qos_profile_sensor_data);feet=n.create_publisher(PolygonStamped,prefix+'/footprint',10)
    trace=[];states=[];n.create_subscription(Twist,prefix+'/result',lambda v:trace.append((time.monotonic(),v.linear.x,v.linear.y,v.angular.z)),10)
    n.create_subscription(CollisionMonitorState,prefix+'/state',lambda v:states.append((time.monotonic(),v.action_type)),10)
    tf=StaticTransformBroadcaster(n);transforms=[]
    for parent,child in [('fixture_odom','fixture_base'),('fixture_base','fixture_laser')]:
        t=TransformStamped();t.header.stamp=n.get_clock().now().to_msg();t.header.frame_id=parent;t.child_frame_id=child;t.transform.rotation.w=1.;transforms.append(t)
    tf.sendTransform(transforms);phase={'name':'idle_right_wall','v':(0.,0.,0.)}
    # Synthetic slanted wall at the right-front; no office scan or map is used.
    wall=[(.32+i*.01,-.31+i*.0045)for i in range(23)]
    def publish():
        name=phase['name'];p=PolygonStamped();p.header.stamp=n.get_clock().now().to_msg();p.header.frame_id='fixture_base'
        if name!='empty_footprint':
            for x,y in body:q=Point32();q.x=x;q.y=y;p.polygon.points.append(q)
        feet.publish(p)
        scan=LaserScan();scan.header.stamp=n.get_clock().now().to_msg();scan.header.frame_id='fixture_laser';scan.angle_min=-math.pi;scan.angle_increment=math.pi/180;scan.angle_max=scan.angle_min+359*scan.angle_increment
        scan.range_min=.12;scan.range_max=14.;ranges=[6.]*360
        points=wall if 'wall' in name or name in ('stale','bad_tf','empty_footprint')else[]
        if name=='straight_future_obstacle':points=[(.9,k*.01)for k in range(-4,5)]
        if name=='body_intrusion':points=[(.15,k*.01)for k in range(-4,5)]
        if name=='front_corner_turn':points=[(.33,k*.005)for k in range(-4,5)]
        if name=='rear_corner_turn':points=[(-.33,k*.005)for k in range(-4,5)]
        if name=='left_arc_obstacle':points=[(.5,.25+k*.005)for k in range(-4,5)]
        for x,y in points:
            index=round((math.atan2(y,x)-scan.angle_min)/scan.angle_increment)%360;ranges[index]=min(ranges[index],math.hypot(x,y))
        scan.ranges=ranges
        if name=='stale':scan.header.stamp=rclpy.time.Time(nanoseconds=n.get_clock().now().nanoseconds-800_000_000).to_msg()
        if name=='bad_tf':scan.header.frame_id='fixture_missing_laser'
        scans.publish(scan);v=Twist();v.linear.x,v.linear.y,v.angular.z=phase['v'];cmd.publish(v)
    n.create_timer(.05,publish);log=output.with_suffix('.node.log').open('w');process=subprocess.Popen([args.binary,'--ros-args','--params-file',str(paramfile)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True);results=[]
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            assert process.poll()is None,'monitor exited';rclpy.spin_once(n,timeout_sec=.02)
    cases=[('idle_right_wall',(0.,0.,0.),False),('left_arc_wall',(.6,0.,1.),False),('straight_wall',(.75,0.,0.),True),('right_arc_wall',(.6,0.,-1.),True),
        ('straight_future_obstacle',(.75,0.,0.),True),('front_corner_turn',(0.,0.,1.),True),('rear_corner_turn',(0.,0.,1.),True),('left_arc_obstacle',(.6,0.,1.),True),
        ('body_intrusion',(0.,0.,0.),True),('empty_footprint',(.6,0.,1.),True),('stale',(.6,0.,1.),True),('bad_tf',(.6,0.,1.),True),('left_arc_wall',(.6,0.,1.),False)]
    try:
        client=n.create_client(ChangeState,'/collision_monitor/change_state');assert client.wait_for_service(timeout_sec=8)
        for transition in (1,3):
            request=ChangeState.Request();request.transition.id=transition;f=client.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=5);assert f.result()and f.result().success
        for name,velocity,stop in cases:
            phase.update(name=name,v=velocity);spin(.55);start=time.monotonic();spin(.45)
            values=[v for v in trace if v[0]>=start];decisions=[s[1]for s in states if s[0]>=start];assert values and decisions,(name,'no actual outputs')
            row={'case':name,'requested':velocity,'samples':len(values),'state_values':sorted(set(decisions)),'outputs':sorted(set(tuple(v[1:])for v in values))};results.append(row)
            if stop:assert all(v[1:]==(0.,0.,0.)for v in values)and all(s==CollisionMonitorState.STOP for s in decisions),row
            else:
                assert all(max(abs(a-b)for a,b in zip(v[1:],velocity))<1e-8 for v in values),row
                assert all(s!=CollisionMonitorState.STOP for s in decisions),row
            row['passed']=True
    finally:
        if process.poll()is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
        log.close();n.destroy_node();rclpy.shutdown();output.write_text(json.dumps({'scope':'Actual native Nav2 monitor, synthetic slanted wall and body/corner/source faults, static TF; no physical plant or actuator/socket commands.',
            'seed':0,'parameters':m,'cases':results,'complete':len(results)==len(cases)and all(r.get('passed')for r in results),'hardware_commands':0},indent=2))
    print(json.dumps({'complete':True,'cases':len(results),'hardware_commands':0}))

if __name__=='__main__':main()
