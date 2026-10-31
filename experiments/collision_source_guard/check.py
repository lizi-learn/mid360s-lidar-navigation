"""Isolated synthetic source/TF fault check. Publishes only test-prefixed topics."""
import argparse,json,math,os,signal,subprocess,tempfile,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.srv import ChangeState
from geometry_msgs.msg import Twist,TransformStamped
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import CollisionMonitorState
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster
import yaml

def main():
    p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);p.add_argument('--expect-old-defect',action='store_true');a=p.parse_args()
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1' and int(os.environ.get('ROS_DOMAIN_ID','0'))>=200,'isolated loopback test domain required'
    prefix='/collision_source_guard'
    params={'collision_monitor':{'ros__parameters':{'base_frame_id':'fixture_base','odom_frame_id':'fixture_odom','transform_tolerance':.15,'source_timeout':.5,'base_shift_correction':True,'stop_pub_timeout':2.,'cmd_vel_in_topic':prefix+'/request','cmd_vel_out_topic':prefix+'/result','state_topic':prefix+'/state','source_valid_topic':prefix+'/source_valid','polygons':['StopZone'],'StopZone.type':'polygon','StopZone.points':[1.,.72,1.,-.72,-.58,-.72,-.58,.72],'StopZone.action_type':'stop','StopZone.max_points':2,'StopZone.visualize':False,'observation_sources':['scan'],'scan.type':'scan','scan.topic':prefix+'/scan','scan.enabled':True}}}
    destination=Path(a.output);destination.parent.mkdir(parents=True,exist_ok=True)
    paramfile=destination.with_suffix('.params.yaml');paramfile.write_text(yaml.safe_dump(params))
    rclpy.init();n=rclpy.create_node('isolated_collision_source_fixture')
    cmd=n.create_publisher(Twist,prefix+'/request',10);scan=n.create_publisher(LaserScan,prefix+'/scan',qos_profile_sensor_data)
    trace=[];states=[];health=[];phase={'name':'missing'}
    n.create_subscription(Twist,prefix+'/result',lambda m:trace.append((time.monotonic(),m.linear.x,m.linear.y,m.angular.z)),10)
    n.create_subscription(CollisionMonitorState,prefix+'/state',lambda m:states.append((time.monotonic(),int(m.action_type))),10)
    n.create_subscription(Bool,prefix+'/source_valid',lambda m:health.append((time.monotonic(),m.data)),10)
    tf=StaticTransformBroadcaster(n);transforms=[]
    for parent,child in [('fixture_odom','fixture_base'),('fixture_base','fixture_laser')]:
        t=TransformStamped();t.header.stamp=n.get_clock().now().to_msg();t.header.frame_id=parent;t.child_frame_id=child;t.transform.rotation.w=1.;transforms.append(t)
    tf.sendTransform(transforms)
    def publish():
        m=Twist();m.linear.x=.1;cmd.publish(m)
        if phase['name']=='missing':return
        s=LaserScan();now=n.get_clock().now();s.header.stamp=now.to_msg();s.header.frame_id='fixture_laser';s.angle_min=-math.pi;s.angle_increment=2*math.pi/360;s.angle_max=s.angle_min+359*s.angle_increment;s.range_min=.12;s.range_max=14.;s.scan_time=.1;s.ranges=[3.]*360
        if phase['name']=='obstacle':
            ranges=list(s.ranges);ranges[175:186]=[.3]*11;s.ranges=ranges
        elif phase['name']=='unknown':s.ranges=[float('nan')]*360
        elif phase['name']=='stale':s.header.stamp=rclpy.time.Time(nanoseconds=now.nanoseconds-800_000_000).to_msg()
        elif phase['name']=='bad_tf':s.header.frame_id='fixture_missing_laser'
        scan.publish(s)
    n.create_timer(.05,publish)
    log=destination.with_suffix('.node.log').open('w')
    process=subprocess.Popen([a.binary,'--ros-args','--params-file',str(paramfile)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    results=[]
    def spin(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            assert process.poll() is None,'Monitor exited'
            rclpy.spin_once(n,timeout_sec=.02)
    try:
        client=n.create_client(ChangeState,'/collision_monitor/change_state');assert client.wait_for_service(timeout_sec=8)
        for transition in [1,3]:
            q=ChangeState.Request();q.transition.id=transition;future=client.call_async(q);rclpy.spin_until_future_complete(n,future,timeout_sec=5);assert future.result() and future.result().success
        for name in ['missing','clear','obstacle','unknown','stale','bad_tf','clear_recovery']:
            phase['name']='clear' if name=='clear_recovery' else name
            spin(.8);start=time.monotonic();spin(.65)
            values=[x for x in trace if x[0]>=start];decision=[x[1] for x in states if x[0]>=start];valid=[x[1] for x in health if x[0]>=start]
            assert values,'No actual velocity output for '+name
            peak=max(abs(x[k]) for x in values for k in [1,2,3]);is_fault=name in ['missing','unknown','stale','bad_tf']
            result={'case':name,'output_samples':len(values),'max_abs_velocity':peak,'state_values':sorted(set(decision)),'source_health_values':sorted(set(valid))}
            results.append(result)
            if name in ['clear','clear_recovery']:assert peak>.05,result
            elif name=='obstacle':assert peak==0 and CollisionMonitorState.STOP in decision,result
            elif a.expect_old_defect:assert peak>.05,result
            else:assert peak==0 and decision and all(x==CollisionMonitorState.STOP for x in decision) and valid and not any(valid),result
            result['passed_expected_behavior']=True
    finally:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=4)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
        log.close();n.destroy_node();rclpy.shutdown()
        destination.write_text(json.dumps({'scope':'Synthetic isolated monitor component, no robot model/plant or gait socket, actuator writes0. Static TF fixture, original source stamps except explicitly stale fault injection.','seed':0,'parameters':params,'expected_old_defect':a.expect_old_defect,'cases':results,'complete':len(results)==7,'hardware_commands':0},indent=2))
    assert len(results)==7
    print(json.dumps({'complete':True,'expected_old_defect':a.expect_old_defect,'cases':len(results),'hardware_commands':0}))

if __name__=='__main__':main()
