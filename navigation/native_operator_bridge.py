#!/usr/bin/env python3
"""Bounded/continuous read-only reuse of the already commissioned G12 data."""
import argparse,hashlib,json,secrets,signal,sys,time
from pathlib import Path
from native_operator_source import NativePublisher

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--code-root',required=True);p.add_argument('--profile',required=True)
    p.add_argument('--commissioning',required=True);p.add_argument('--socket',required=True)
    p.add_argument('--source-domain',type=int,default=106);p.add_argument('--duration',type=float,default=0)
    a=p.parse_args();sys.path[:0]=[a.code_root,str(Path(a.code_root)/'navigation')]
    from robot_profile import load,check_machine,identity
    from operator_authority import ReceiverAuthority
    profile=load(a.profile);check_machine(profile);record=json.loads(Path(a.commissioning).read_text())
    digest=hashlib.sha256(Path(record['receiver_executable_path']).read_bytes()).hexdigest()
    if not(record.get('robot_identity')==identity(profile) and record.get('receiver_sha256')==digest
           and record.get('physical_positions')=={'left':282,'middle':1002,'right':1722}
           and record.get('operator_confirmed') is True):raise ValueError('Existing G12 commissioning mismatch')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from std_msgs.msg import Float64MultiArray
    rclpy.init(domain_id=a.source_domain);node=Node('p2_gait_native_operator_readonly')
    decoder=ReceiverAuthority(True);source=NativePublisher(a.socket);boot=secrets.token_bytes(32)
    endpoint={'gid':b''};stats={'messages':0,'connected':0,'sent_modes':{},'serial_opens':0,'motor_writes':0}
    def discover():
        peers=node.get_publishers_info_by_topic('/g12_sbus/output')
        endpoint['gid']=bytes(peers[0].endpoint_gid) if len(peers)==1 else b''
    node.create_timer(.1,discover)
    def receive(msg):
        gid=endpoint['gid'];instance=int.from_bytes(hashlib.sha256(boot+gid).digest()[:8],'little')&((1<<63)-1) or 1
        result=decoder.decode(msg.data,gid,time.time(),time.monotonic(),instance)
        source.send(result,msg.data);stats['messages']+=1
        stats['connected']+=int(result.get('receiver_connected') is True)
        mode=result.get('switch_mode','unknown');stats['sent_modes'][mode]=stats['sent_modes'].get(mode,0)+1
    node.create_subscription(Float64MultiArray,'/g12_sbus/output',receive,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    stop=False
    def halt(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGTERM,halt);signal.signal(signal.SIGINT,halt);start=time.monotonic()
    try:
        while not stop and(not a.duration or time.monotonic()-start<a.duration):rclpy.spin_once(node,timeout_sec=.02)
    finally:source.close();node.destroy_node();rclpy.shutdown()
    print(json.dumps(stats),flush=True)
if __name__=='__main__':main()
