import json,pathlib,subprocess,socket,time,threading,tempfile
from native_operator_source import NATIVE
from gait_transport import GaitTransport

def wait(predicate,timeout=1.):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.01)
    raise AssertionError('wait timeout')

def scenario(binary,commissioned):
    result={};source=None;transport=None;process=None;stopping=threading.Event()
    with tempfile.TemporaryDirectory(prefix='gait-v2-') as directory:
        path=directory+'/gait.sock';process=subprocess.Popen([binary,path,'--synthetic-commissioned' if commissioned else '--uncommissioned'],stdout=subprocess.PIPE,text=True)
        try:
            wait(lambda:pathlib.Path(path+'.operator').exists())
            source=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);source.connect(path+'.operator')
            state={'mode':1,'activity':False,'pause':False,'instance':40,'stale_command':False}
            def publish():
                seq=0
                while not stopping.is_set():
                    if not state['pause']:
                        seq+=1;flags=3|(4 if state['activity'] else 0)
                        packet=NATIVE.pack(0x4e4f5031,1,state['instance'],seq,time.monotonic_ns(),time.time_ns()//1000,state['mode'],flags,
                            .5 if state['activity'] else 0.,0.,0.,0)
                        source.send(packet)
                    if transport:
                        stamp=time.monotonic_ns()
                        transport.update({'evaluated_monotonic_ns':stamp,'data_fresh':True,'calibration_ready':True,'ground_ok':True,'localized':True,'command':(.1,0.,0.)},True)
                        if state['stale_command']:
                            with transport.lock:transport.command_stamp=stamp-400_000_000
                    stopping.wait(.01)
            writer=threading.Thread(target=publish);writer.start();transport=GaitTransport(path)
            wait(lambda:transport.feedback is not None)
            if not commissioned:
                time.sleep(.1);assert not transport.ready()
                try:transport.arm(1,.1)
                except ValueError:result['client_health_cannot_open_local_gate']=True
                else:raise AssertionError('uncommissioned ARM accepted')
                return result
            wait(transport.ready);transport.arm(101);wait(lambda:transport.feedback.navigation_owned and transport.feedback.vx>.09)
            result['python_rpc_to_cpp_lease_physical_velocity']=True
            state['mode']=2
            wait(lambda:transport.feedback and transport.feedback.operator_mode==2 and transport.feedback.cancel_pending and not transport.feedback.navigation_owned)
            assert transport.feedback.force_zero;result['physical_middle_wins']=True
            state['mode']=1;transport.stop()
            wait(lambda:transport.feedback and transport.feedback.cancel_pending)
            transport.acknowledge_cancel();wait(lambda:not transport.feedback.cancel_pending);wait(transport.ready)
            transport.arm(102);wait(lambda:transport.feedback.navigation_owned)
            state['activity']=True
            wait(lambda:transport.feedback.manual_owned and not transport.feedback.navigation_owned)
            assert transport.feedback.vx==.5;result['physical_left_stick_manual_handoff']=True
            state['activity']=False;time.sleep(.1);assert not transport.feedback.navigation_owned
            result['neutral_does_not_resume']=True
            state['mode']=3;wait(lambda:transport.feedback.operator_mode==3 and not transport.feedback.force_zero)
            result['right_retains_factory_manual']=True
            state['mode']=1;transport.stop();wait(lambda:transport.feedback.cancel_pending)
            transport.acknowledge_cancel();wait(lambda:not transport.feedback.cancel_pending);wait(transport.ready)
            transport.arm(103);wait(lambda:transport.feedback.navigation_owned)
            state['pause']=True
            wait(lambda:transport.feedback and transport.feedback.cancel_pending and not transport.feedback.navigation_owned)
            result['native_source_expiry_cannot_be_restamped_by_rpc']=True
            state['pause']=False;transport.stop();wait(lambda:transport.feedback.operator_verified)
            transport.acknowledge_cancel();wait(lambda:not transport.feedback.cancel_pending);wait(transport.ready)
            transport.arm(104);wait(lambda:transport.feedback.navigation_owned)
            state['stale_command']=True
            wait(lambda:transport.feedback.cancel_pending and not transport.feedback.navigation_owned)
            result['original_command_age_preserved']=True
            return result
        finally:
            stopping.set()
            if 'writer' in locals():writer.join(timeout=1)
            if transport:transport.close()
            if source:source.close()
            if process:
                process.terminate()
                try:process.communicate(timeout=1)
                except subprocess.TimeoutExpired:process.kill();process.communicate()

if __name__=='__main__':
    import sys
    binary=str(pathlib.Path(sys.argv[1]).resolve())
    result=scenario(binary,True)|scenario(binary,False)
    print(json.dumps(result));assert all(result.values())
