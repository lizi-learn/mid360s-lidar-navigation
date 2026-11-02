"""Actual C++ lease and ROS source, with a UI process whose GIL is blocked.

All sockets are temporary. The fixture neither loads a policy nor controls hardware.
The source is a separate process, so blocking the UI does not also block the test
producer. Source expiry, the physical switch and parent death are checked too.
"""
import argparse,ctypes,json,os,pathlib,socket,subprocess,sys,tempfile,threading,time

ROOT=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'navigation'))

def wait(predicate,timeout=2.):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.01)
    raise AssertionError('fixture wait timeout')

def producer(args):
    import select,rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from native_operator_source import NATIVE
    from native_operator_snapshot import NativeSnapshotWriter
    rclpy.init();node=Node('isolated_gait_fixture_source')
    publisher=node.create_publisher(String,'/p2_nav/gate_status',1)
    source=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET)
    source.connect(args.path+'.operator');snapshot=NativeSnapshotWriter(args.path+'.operator.snapshot');state={'mode':1,'health':True,'native':True,'activity':False};sequence=0
    with open(args.receipts,'w')as receipts:
        try:
            while True:
                if select.select([sys.stdin],[],[],0)[0]:
                    line=sys.stdin.readline()
                    if not line:break
                    state.update(json.loads(line))
                sequence+=1;stamp=time.monotonic_ns()
                if state['native']:
                    packet=NATIVE.pack(0x4e4f5031,1,40,sequence,stamp,time.time_ns()//1000,state['mode'],3|(4 if state['activity']else 0),.5 if state['activity']else 0.,0.,0.,0);source.send(packet);snapshot.publish(packet)
                if state['health']:
                    gate=dict(evaluated_monotonic_ns=stamp,data_fresh=True,calibration_ready=True,ground_ok=True,localized=True,armed=True,command=[.6,0.,0.],body_velocity_feedback=[0.,0.,0.],velocity_source_monotonic_ns=stamp)
                    publisher.publish(String(data=json.dumps(gate)));receipts.write(str(stamp)+'\n');receipts.flush()
                time.sleep(.01)
        finally:source.close();snapshot.close();node.destroy_node();rclpy.shutdown()

def scenario(args):
    from gait_transport_process import GaitTransportProcess
    from gait_transport import GaitTransport
    result={'legacy':args.legacy,'hardware_io':False,'passed':False};transport=None;source=None;server=None;executor=None;node=None;thread=None
    with tempfile.TemporaryDirectory(prefix='p2-control-process-')as directory:
        path=directory+'/gait.sock';trace=pathlib.Path(directory)/'trace.jsonl';receipts=pathlib.Path(directory)/'source-stamps.txt'
        server=subprocess.Popen([args.binary,path,'--synthetic-commissioned'],stdout=subprocess.PIPE,text=True)
        try:
            wait(lambda:pathlib.Path(path+'.operator').exists())
            # Wait for listen(), not only bind(). No native source is faked in
            # the transport; this producer supplies the actual fixture endpoint.
            time.sleep(.03)
            source=subprocess.Popen([sys.executable,'-S',__file__,'--producer','--path',path,'--receipts',str(receipts)],stdin=subprocess.PIPE,text=True)
            def control(**state):source.stdin.write(json.dumps(state)+'\n');source.stdin.flush()
            if args.legacy:
                import rclpy
                from rclpy.node import Node
                from rclpy.executors import SingleThreadedExecutor
                from std_msgs.msg import String
                rclpy.init();node=Node('legacy_gait_control_fixture');transport=GaitTransport(path,trace_path=trace)
                node.create_subscription(String,'/p2_nav/gate_status',lambda m:transport.update(json.loads(m.data),transport.phase=='active'),1)
                executor=SingleThreadedExecutor();executor.add_node(node);thread=threading.Thread(target=executor.spin);thread.start()
            else:transport=GaitTransportProcess(path,trace_path=trace)
            wait(transport.ready,4.);transport.arm(1)
            wait(lambda:(transport.status().get('speed_tracking')or{}).get('policy_input_x',0)>.599,3.)
            if not args.legacy:
                authority=transport.operator_authority();assert authority['source_transport']=='native_packet_snapshot'
                proxy=transport.proxy
                class SlowAuthorityProxy:
                    def __getattr__(self,name):return getattr(proxy,name)
                    def operator_authority(self):
                        reply=proxy.operator_authority();ctypes.PyDLL(None).usleep(250000);return reply
                transport.proxy=SlowAuthorityProxy();begin=time.monotonic();authority=transport.operator_authority()
                assert time.monotonic()-begin<.1 and -.005<=time.monotonic()-authority['stamp']<=.1
                assert authority['reply_stamp']is None
                result['original_switch_observation_independent_of_delayed_rpc']=True
            # ctypes.PyDLL holds the current process GIL. time.sleep would
            # release it and would miss the actual UI starvation failure.
            sleep=ctypes.PyDLL(None).usleep;sleep.argtypes=[ctypes.c_uint];sleep.restype=ctypes.c_int
            samples=[]
            for _ in range(6):
                sleep(250000);samples.append(transport.status());time.sleep(.08)
            stable=all(s['navigation_owned']and not s['cancel_pending']and(s.get('speed_tracking')or{}).get('policy_input_x',0)>.599 for s in samples)
            result.update(ui_gil_blocks=6,each_block_ms=250,continuous_owned_and_policy_input=stable,
                minimum_policy_input_x=min((s.get('speed_tracking')or{}).get('policy_input_x',0)for s in samples))
            if args.legacy:
                assert not stable,'Legacy unexpectedly passed the reproducer'
                result.update(passed=True,expected_legacy_failure=True);return result
            assert stable,samples
            control(health=False)
            wait(lambda:transport.status()['cancel_pending']and not transport.status()['navigation_owned'])
            control(health=True);time.sleep(.25);assert not transport.status()['navigation_owned']
            result.update(original_health_expiry_revokes=True,recovery_never_auto_rearms=True)
            def rearm(generation):
                transport.stop();time.sleep(.08);transport.acknowledge_cancel();wait(lambda:not transport.status()['cancel_pending']);wait(transport.ready);transport.arm(generation);wait(lambda:transport.status()['navigation_owned'])
            rearm(2);control(mode=2)
            wait(lambda:transport.status()['operator_mode']==2 and transport.status()['cancel_pending']and not transport.status()['navigation_owned'])
            result['middle_revokes']=True
            control(mode=1);time.sleep(.1);assert not transport.status()['navigation_owned'];rearm(3);control(activity=True)
            wait(lambda:transport.status()['manual_owned']and not transport.status()['navigation_owned'])
            assert abs(transport.status()['velocity'][0]-.5)<1e-6;result['stick_manual_priority']=True
            control(activity=False,mode=3);wait(lambda:transport.status()['operator_mode']==3 and not transport.status()['force_zero']);result['right_factory_manual']=True
            control(mode=1);time.sleep(.1);rearm(4);control(native=False)
            wait(lambda:transport.status()['cancel_pending']and not transport.status()['navigation_owned']);result['original_native_expiry_revokes']=True
            # Close the actual parent lifeline without an orderly worker RPC.
            os.close(transport.lifeline);transport.lifeline=None;transport.process.wait(timeout=3)
            assert transport.process.returncode==0;result['parent_exit_closes_native_socket']=True
            sent={int(line)for line in receipts.read_text().splitlines()};rows=[json.loads(line)for line in trace.read_text().splitlines()]
            stamps=[r['health_source_ns']for r in rows if r.get('health_source_ns')]
            assert len(stamps)>20 and all(stamp in sent for stamp in stamps)
            result.update(original_evaluation_stamps_preserved=True,trace_samples=len(rows),passed=True)
        finally:
            if transport:transport.close()
            if executor:
                executor.shutdown();thread.join(timeout=2);node.destroy_node()
                import rclpy
                rclpy.shutdown()
            if source:
                source.stdin.close()
                try:source.wait(timeout=2)
                except subprocess.TimeoutExpired:source.terminate();source.wait(timeout=2)
            server.terminate();server.communicate(timeout=2)
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('binary',nargs='?');p.add_argument('--legacy',action='store_true');p.add_argument('--producer',action='store_true');p.add_argument('--path');p.add_argument('--receipts');args=p.parse_args()
    if args.producer:producer(args)
    else:print(json.dumps(scenario(args)),flush=True)
