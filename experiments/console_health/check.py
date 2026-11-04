"""Actual Console callbacks under blocked non-control work; no gait or robot I/O."""
import argparse,json,sys,tempfile,threading,time
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'navigation'))
from console import Console,console_executor

class Receipts:
    phase='stopped'
    def __init__(self):self.rows=[]
    def update(self,gate,enabled):
        if gate.get('evaluated_monotonic_ns'):self.rows.append((time.monotonic_ns(),gate['evaluated_monotonic_ns']))
    def operator_authority(self):return None
    def status(self):return {'cancel_pending':False,'phase':'stopped'}
    def acknowledge_cancel(self):pass
    def stop(self):pass
    def ready(self):return False

ap=argparse.ArgumentParser();ap.add_argument('--legacy',action='store_true');args=ap.parse_args()
rclpy.init();temporary=tempfile.TemporaryDirectory();node=Console(Path(temporary.name),True,.03);receipts=Receipts();node.gait=receipts
executor=SingleThreadedExecutor()if args.legacy else console_executor(node)
if args.legacy:
    executor.add_node(node);executor.add_node(node.health_node)
source=Node('console_health_fixture_source');health=source.create_publisher(String,'/p2_nav/gate_status',10);block=source.create_publisher(String,'/console_health_fixture_block',10)
blocked=[]
def blocking(msg):
    blocked.append(time.monotonic_ns());time.sleep(.25)
node.create_subscription(String,'/console_health_fixture_block',blocking,10)
thread=threading.Thread(target=executor.spin);thread.start();sent=set()
try:
    deadline=time.monotonic()+3
    while health.get_subscription_count()==0 and time.monotonic()<deadline:time.sleep(.02)
    assert health.get_subscription_count()>0
    begin=time.monotonic();blocks=set()
    while time.monotonic()-begin<2.5:
        now=time.monotonic();stamp=time.monotonic_ns();sent.add(stamp);m=String();m.data=json.dumps(dict(evaluated_monotonic_ns=stamp,data_fresh=True,calibration_ready=True,ground_ok=True,localized=True,collision_source_valid=True,command=[0.,0.,0.]))
        health.publish(m)
        for k,at in enumerate((.5,1.3)):
            if now-begin>=at and k not in blocks:block.publish(String(data='block unrelated callback'));blocks.add(k)
        time.sleep(.02)
    unique=[];seen=set()
    for received,stamp in receipts.rows:
        if stamp not in seen:unique.append((received,stamp));seen.add(stamp)
    assert len(blocked)==2 and len(unique)>40
    assert all(stamp in sent for received,stamp in unique),'Original evaluation stamp changed'
    gaps=[(b[0]-a[0])/1e6 for a,b in zip(unique,unique[1:])];ages=[(received-stamp)/1e6 for received,stamp in unique]
    maximum=max(gaps)
    result=dict(passed=(maximum>=200 if args.legacy else maximum<100),legacy_executor=args.legacy,actual_console=True,blocked_callbacks=len(blocked),original_stamps_preserved=True,samples=len(unique),maximum_forwarding_gap_ms=maximum,maximum_evaluation_age_ms=max(ages),motion_publishers_used=False,scope='Actual ROS Console and source-stamped synthetic health; receipt stub has no socket, gait or actuator access.')
    print(json.dumps(result),flush=True)
    assert maximum>=200 if args.legacy else maximum<100,(args.legacy,maximum)
finally:
    executor.shutdown();thread.join(timeout=3);
    if args.legacy:node.health_node.destroy_node()
    node.gait=None;node.door_worker.close();node.reception.close();node.flight.close();node.destroy_node();source.destroy_node();rclpy.shutdown();temporary.cleanup()
