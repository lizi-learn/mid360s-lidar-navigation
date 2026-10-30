#!/usr/bin/env python3
"""Observe raw receiver ROS telemetry in one domain; publish authority in another.

No serial port, motor topic or vendor shared memory is opened. A commissioning
record must match this machine, installation and receiver executable hash.
"""
import argparse,hashlib,json,secrets,signal,sys,time
from pathlib import Path
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import Float64MultiArray,String
from operator_authority import ReceiverAuthority

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profile',required=True);p.add_argument('--commissioning',required=True)
    p.add_argument('--receiver-executable',default='')
    p.add_argument('--source-domain',type=int,required=True);p.add_argument('--target-domain',type=int,required=True)
    a=p.parse_args();sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
    from robot_profile import load,check_machine,identity
    profile=load(a.profile);check_machine(profile)
    record=json.loads(Path(a.commissioning).read_text())
    digest=hashlib.sha256(Path(a.receiver_executable or record['receiver_executable_path']).read_bytes()).hexdigest()
    verified=(record.get('robot_identity')==identity(profile) and record.get('receiver_sha256')==digest
              and record.get('physical_positions')=={'left':282,'middle':1002,'right':1722}
              and record.get('operator_confirmed') is True)
    if not verified:raise ValueError('Physical switch commissioning does not match this installation/receiver')
    source_context=Context();sink_context=Context()
    rclpy.init(context=source_context,domain_id=a.source_domain)
    rclpy.init(context=sink_context,domain_id=a.target_domain)
    source=Node('navigation_receiver_readonly',context=source_context)
    sink=Node('navigation_operator_authority',context=sink_context)
    pub=sink.create_publisher(String,'/p2_nav/operator_authority',1)
    decoder=ReceiverAuthority(verified);boot=secrets.token_bytes(32)
    endpoint={'gid':b''}
    def discover():
        peers=source.get_publishers_info_by_topic('/g12_sbus/output')
        endpoint['gid']=bytes(peers[0].endpoint_gid) if len(peers)==1 else b''
    source.create_timer(.1,discover)
    def receive(msg):
        # Humble rclpy on this installation has a one-argument subscription
        # callback. Discovery identifies the sole publisher for UI provenance;
        # physical gait arbitration must consume the receiver in its own loop.
        gid=endpoint['gid']
        instance=int.from_bytes(hashlib.sha256(boot+gid).digest()[:8],'little')&((1<<63)-1) or 1
        result=decoder.decode(msg.data,gid,time.time(),time.monotonic(),instance)
        out=String();out.data=json.dumps(result,allow_nan=False);pub.publish(out)
    source.create_subscription(Float64MultiArray,'/g12_sbus/output',receive,
        QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    executor=SingleThreadedExecutor(context=source_context);executor.add_node(source)
    stopping=False
    def stop(*_):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    print('Read-only receiver authority ready; hardware execution remains disabled',flush=True)
    try:
        while not stopping:executor.spin_once(timeout_sec=.05)
    finally:
        executor.shutdown();source.destroy_node();sink.destroy_node()
        rclpy.shutdown(context=source_context);rclpy.shutdown(context=sink_context)
if __name__=='__main__':main()
