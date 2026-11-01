"""Actual ROS relay roundtrip of a synthetic long path; never actuator IO."""
import argparse,json,os,pathlib,signal,subprocess,sys,time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
root=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'navigation'))
from path_view import decode_view

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    env={**os.environ,'PYTHONPATH':str(root/'navigation')+':'+os.environ.get('PYTHONPATH','')}
    rclpy.init();node=Node('path_view_component_probe')
    process=subprocess.Popen([sys.executable,str(root/'navigation/path_view_node.py')],env=env,
        stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
    points=[[i*.05,(i%17)*.01] for i in range(812)];sent={};received=[];delays=[]
    output=node.create_publisher(Path,'/plan',10)
    def receive(msg):
        actual,stamp=decode_view(msg.data)
        if stamp not in sent:return
        assert actual==points, 'Relay changed or dropped a path point'
        received.append(stamp);delays.append((node.get_clock().now().nanoseconds-stamp)/1e6)
    node.create_subscription(String,'/p2_nav/path_view',receive,10)
    started=time.monotonic();last=0.;error=None
    try:
        while time.monotonic()-started<7:
            rclpy.spin_once(node,timeout_sec=.005)
            now=time.monotonic()
            if now-last>.1 and len(sent)<30:
                message=Path();message.header.frame_id='map';message.header.stamp=node.get_clock().now().to_msg()
                stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
                for x,y in points:
                    pose=PoseStamped();pose.header=message.header;pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.orientation.w=1.;message.poses.append(pose)
                sent[stamp]=points;output.publish(message);last=now
            if len(received)>=10:break
        assert len(received)>=10, 'Actual relay did not deliver the synthetic paths'
    except Exception as exc:error=repr(exc)
    finally:
        node.destroy_node();rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        log=process.stdout.read()
    result={'scope':'Synthetic actual ROS Path -> isolated relay -> String roundtrip; no robot data or commands',
        'sent':len(sent),'received':len(received),'all_812_points_retained':len(received)>=10 and error is None,
        'original_ros_source_times_retained':all(stamp in sent for stamp in received),
        'max_source_age_at_delivery_ms':max(delays,default=None),'error':error,
        'child_stopped':process.poll() is not None}
    pathlib.Path(args.output).write_text(json.dumps(result,indent=2));pathlib.Path(args.output+'.log').write_text(log)
    print(json.dumps(result));assert error is None,error

if __name__=='__main__':main()
