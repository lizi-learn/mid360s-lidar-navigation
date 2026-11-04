"""Actual buffer->monitor->gate callbacks plus production pivot/input arithmetic.

Synthetic10Hz source/2Hz velocity ripple and planar yaw response; no robot,
gait socket or actuator. The native monitor separately checks real ray inputs.
This test deliberately does not claim a full Nav2 route or biped validation.
"""
import argparse,ctypes,hashlib,json,math,os,signal,subprocess,sys,time
from pathlib import Path
import rclpy,yaml
from geometry_msgs.msg import Twist,TransformStamped,PolygonStamped,Point32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool,String
from lifecycle_msgs.srv import ChangeState
from tf2_ros import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data
NAV=Path(__file__).resolve().parents[2]/'navigation'

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--binary',required=True);ap.add_argument('--library',required=True);ap.add_argument('--output',required=True);ap.add_argument('--duplicate-owner',action='store_true');a=ap.parse_args()
    assert os.getenv('ROS_LOCALHOST_ONLY')=='1'and int(os.getenv('ROS_DOMAIN_ID','0'))>=200
    out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);assert not out.exists()
    D=ctypes.c_double;P=ctypes.c_void_p;lib=ctypes.CDLL(str(Path(a.library).resolve()))
    lib.p2_handoff_create.restype=P;lib.p2_handoff_destroy.argtypes=[P]
    lib.p2_handoff_pivot.argtypes=[P,D,D,D,D];lib.p2_handoff_pivot.restype=D
    lib.p2_handoff_input.argtypes=[P,D,D,D,ctypes.c_ulonglong];lib.p2_handoff_input.restype=D
    native=lib.p2_handoff_create()
    params={'collision_monitor':{'ros__parameters':dict(base_frame_id='base_link',odom_frame_id='odom',transform_tolerance=.15,source_timeout=.5,base_shift_correction=True,stop_pub_timeout=2.,cmd_vel_in_topic='/p2_nav/cmd_buffered',cmd_vel_out_topic='/p2_nav/cmd_collision',state_topic='/p2_nav/collision_state',source_valid_topic='/p2_nav/collision_source_valid',polygons=['StopZone','SweptZone'],StopZone=dict(type='polygon',points=[.275,.275,.275,-.275,-.275,-.275,-.275,.275],action_type='stop',max_points=2,visualize=False),SweptZone=dict(type='polygon',action_type='approach',max_points=2,time_before_collision=1.,simulation_time_step=.02,footprint_topic='/fixture/footprint',visualize=False),stop_on_predicted_collision=True,observation_sources=['scan'],scan=dict(type='scan',topic='/scan',enabled=True))}}
    paramfile=out.with_suffix('.yaml');paramfile.write_text(yaml.safe_dump(params))
    rclpy.init();node=rclpy.create_node('isolated_actual_handoff_fixture');tf=TransformBroadcaster(node)
    rawpub=node.create_publisher(Twist,'/p2_nav/cmd_raw',10);odpub=node.create_publisher(Odometry,'/odom',10);scanpub=node.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
    footpub=node.create_publisher(PolygonStamped,'/fixture/footprint',10);groundpub=node.create_publisher(Bool,'/p2_nav/ground_ok',10);refpub=node.create_publisher(String,'/p2_nav/reference_status',10);oppub=node.create_publisher(String,'/p2_nav/operator',10)
    state=dict(yaw=0.,rate=0.,safe=0.,raw=0.,input=0.,buffered=0.,collision=0.,armed=False);trace=[];gates=[];goal=-.14;started=None;last=None;quiet=0.;sim=0.;last_sensor=-99.
    def safe(msg):state['safe']=msg.angular.z
    node.create_subscription(Twist,'/p2_nav/cmd_safe',safe,10)
    node.create_subscription(Twist,'/p2_nav/cmd_buffered',lambda m:state.update(buffered=m.angular.z),10)
    node.create_subscription(Twist,'/p2_nav/cmd_collision',lambda m:state.update(collision=m.angular.z),10)
    node.create_subscription(String,'/p2_nav/gate_status',lambda m:gates.append(json.loads(m.data)),10)
    def publish():
        nonlocal last,sim,last_sensor,quiet
        now=time.monotonic();dt=.02 if last is None else now-last;last=now
        if dt>.1:raise RuntimeError('Fixture scheduling gap; do not hide timing failure')
        dt=max(.001,dt);sim+=dt
        stamp=node.get_clock().now();source=stamp.nanoseconds
        request=0. if started is None else lib.p2_handoff_pivot(native,goal-state['yaw'],state['yaw'],state['rate'],sim)
        state['raw']=request
        m=Twist();m.angular.z=request;rawpub.publish(m)
        # Production ramp arithmetic; this planar plant has a named deadzone.
        input_yaw=lib.p2_handoff_input(native,state['safe'],state['rate'],dt,source);state['input']=input_yaw
        desired=0. if abs(input_yaw)<.3 else .55*input_yaw
        state['rate']+=(desired-state['rate'])*(1-math.exp(-dt/.15));state['yaw']+=state['rate']*dt
        op=String();op.data=json.dumps(dict(armed=started is not None,paused=False,arm_generation=1,speed=.75,velocity_scale=1.));oppub.publish(op)
        b=Bool();b.data=True;groundpub.publish(b)
        ref=String();ref.data=json.dumps(dict(navigation_reference='lidar_projection',navigation_reference_ready=True,velocity_ready=True));refpub.publish(ref)
        foot=PolygonStamped();foot.header.stamp=stamp.to_msg();foot.header.frame_id='base_link';foot.polygon.points=[Point32(x=x,y=y)for x,y in[(.275,.275),(.275,-.275),(-.275,-.275),(-.275,.275)]];footpub.publish(foot)
        if sim-last_sensor>=.095:
            last_sensor=sim;ripple=.12*math.sin(4*math.pi*sim);body_x=-.12/(4*math.pi)*math.cos(4*math.pi*sim)
            tr=TransformStamped();tr.header.stamp=stamp.to_msg();tr.header.frame_id='odom';tr.child_frame_id='base_link';tr.transform.translation.x=body_x;tr.transform.rotation.z=math.sin(state['yaw']/2);tr.transform.rotation.w=math.cos(state['yaw']/2);tf.sendTransform(tr)
            od=Odometry();od.header=tr.header;od.child_frame_id='base_link';od.pose.pose.position.x=body_x;od.pose.pose.orientation=tr.transform.rotation;od.twist.twist.linear.x=ripple*math.cos(state['yaw']);od.twist.twist.linear.y=-ripple*math.sin(state['yaw']);od.twist.twist.angular.z=state['rate'];odpub.publish(od)
            scan=LaserScan();scan.header.stamp=stamp.to_msg();scan.header.frame_id='base_link';scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/360;scan.angle_max=scan.angle_min+359*scan.angle_increment;scan.range_min=.12;scan.range_max=14.;scan.ranges=[3.]*360;scanpub.publish(scan)
        trace.append(dict(t=sim,**state,gate_armed=gates[-1].get('armed')if gates else False,gate_reason=gates[-1].get('reason')if gates else None,phase=gates[-1].get('motion_phase')if gates else None))
    node.create_timer(.02,publish);children=[];logs=[];result={}
    def child(cmd,name):
        f=out.with_name(out.stem+'-'+name+'.log').open('w');logs.append(f);children.append(subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            assert all(c.poll()is None for c in children),'fixture child exited'
            rclpy.spin_once(node,timeout_sec=.01)
    try:
        child([a.binary,'--ros-args','--params-file',str(paramfile)],'monitor')
        child([sys.executable,'-S',str(NAV/'motion_buffer_node.py'),'--ros-args','-p','native_input_ramp:=true'],'buffer')
        child([sys.executable,'-S',str(NAV/'gate_node.py'),'--ros-args','-p','simulation:=true','-p','require_localization:=false','-p','gait_handoff:=true','-p','upstream_motion_handoff:='+str(not a.duplicate_owner).lower()],'gate')
        client=node.create_client(ChangeState,'/collision_monitor/change_state');assert client.wait_for_service(timeout_sec=8)
        for transition in [1,3]:
            q=ChangeState.Request();q.transition.id=transition;f=client.call_async(q);rclpy.spin_until_future_complete(node,f,timeout_sec=5);assert f.result()and f.result().success
        spin(3.);assert gates and gates[-1].get('collision_source_valid'),'Fresh native source required before the fixture starts';started=time.monotonic();quiet=0.
        while time.monotonic()-started<22:
            spin(.05)
            quiet=quiet+.05 if abs(goal-state['yaw'])<.03 and abs(state['rate'])<.03 else 0.
            if quiet>.3:break
        reached=quiet>.3
        armed_samples=sum(bool(x['gate_armed'])for x in trace)
        identities={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()for p in [Path(a.binary),Path(a.library),NAV/'motion_buffer_node.py',NAV/'gate_node.py',NAV/'motion_buffer.py']}
        result=dict(passed=armed_samples>20 and(not reached if a.duplicate_owner else reached),armed_samples=armed_samples,duplicate_owner=a.duplicate_owner,goal_reached=reached,goal=goal,actual_yaw=state['yaw'],max_native_input=max(abs(x['input'])for x in trace),duration=time.monotonic()-started,scope=__doc__,actual_processes=[p.args for p in children],binary_and_source_sha256=identities,parameters=params,seed=0,source_hz=10,ripple_mps=.12,body_total_side=.55,actuator_commands=0)
    finally:
        for p in reversed(children):
            if p.poll()is None:os.killpg(p.pid,signal.SIGINT)
        for p in children:
            try:p.wait(timeout=4)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for f in logs:f.close()
        lib.p2_handoff_destroy(native);node.destroy_node();rclpy.shutdown();out.with_suffix('.trace.json').write_text(json.dumps(trace));out.write_text(json.dumps(result,indent=2))
    print(json.dumps(result));assert result['passed'],result
if __name__=='__main__':main()
