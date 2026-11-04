"""Bounded actual C++ transport with delayed RPCs; never addresses a robot."""
import argparse,json,pathlib,socket,subprocess,sys,tempfile,threading,time
ap=argparse.ArgumentParser();ap.add_argument('binary');ap.add_argument('--module-dir',type=pathlib.Path);ap.add_argument('--expect-baseline-failure',action='store_true');ap.add_argument('--measured-delay-ms',type=float,default=0.);ap.add_argument('--velocity-delay-ms',type=float,default=65.);args=ap.parse_args()
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[2]/'navigation'))
if args.module_dir:sys.path.insert(0,str(args.module_dir))
import gait_transport
from gait_socket_client import GaitClient,Operation
from native_operator_source import NATIVE
from native_operator_snapshot import NativeSnapshotWriter
class Delayed(GaitClient):
    def exchange(self,operation,*a,**kw):
        # Each request stays within the existing100ms contract. Serial work
        # must not carry an old receiver echo across both delayed operations.
        if operation==Operation.MEASURED_VELOCITY:time.sleep(args.measured_delay_ms/1000.)
        elif operation==Operation.VELOCITY:time.sleep(args.velocity_delay_ms/1000.)
        return super().exchange(operation,*a,**kw)
gait_transport.GaitClient=Delayed
result={'scope':'Actual compiled C++ lease fixture and source-stamped Python transport with delayed requests; no ROS or hardware','hardware_commands':0,'delays_ms':{'measured':args.measured_delay_ms,'velocity':args.velocity_delay_ms},'passed':False};transport=None;source=None;proc=None;stopping=threading.Event();writer=None
with tempfile.TemporaryDirectory(prefix='p2-rpc-cadence-')as directory:
    path=directory+'/fixture.sock';proc=subprocess.Popen([args.binary,path,'--synthetic-commissioned'],stdout=subprocess.PIPE,text=True)
    try:
        deadline=time.monotonic()+2
        while not pathlib.Path(path+'.operator').exists()and time.monotonic()<deadline:time.sleep(.01)
        source=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET)
        while True:
            try:source.connect(path+'.operator');break
            except(FileNotFoundError,ConnectionRefusedError):
                if time.monotonic()>deadline:raise
                time.sleep(.01)
        snapshot=NativeSnapshotWriter(path+'.operator.snapshot');state={'healthy':True};original_stamps=set()
        def produce():
            seq=0
            while not stopping.is_set():
                seq+=1;stamp=time.monotonic_ns();packet=NATIVE.pack(0x4e4f5031,1,40,seq,stamp,time.time_ns()//1000,1,3,0.,0.,0.,0);source.send(packet);snapshot.publish(packet);original_stamps.add(stamp)
                if transport:
                    transport.update(dict(evaluated_monotonic_ns=stamp,data_fresh=state['healthy'],calibration_ready=True,ground_ok=True,localized=True,command=(.6,0.,0.),body_velocity_feedback=(0.,0.,0.),velocity_source_monotonic_ns=stamp),True)
                stopping.wait(.01)
        writer=threading.Thread(target=produce);writer.start();transport=gait_transport.GaitTransport(path)
        deadline=time.monotonic()+2
        while not transport.ready()and time.monotonic()<deadline:time.sleep(.01)
        assert transport.ready()
        try:transport.arm(1)
        except ValueError as e:result['arm_error']=str(e)
        samples=[];begin=time.monotonic()
        while time.monotonic()-begin<3:
            row=transport.status();samples.append(row)
            if row['phase']=='failed'or row['cancel_pending']:break
            time.sleep(.02)
        stable=(len(samples)>30 and all(s['navigation_owned']for s in samples)and transport.speed_tracking and transport.speed_tracking['policy_input_x']>.55)
        result.update(stable_nonzero=bool(stable),samples=len(samples),maximum_observed_echo_age_ms=max((s.get('last_authority_sample')or{}).get('operator_echo_age_at_reply_ms',((s.get('last_authority_sample')or{}).get('reply_source_ns',0)-(s.get('last_authority_sample')or{}).get('operator_source_ns',0))/1e6)or 0 for s in samples),terminal_reason=samples[-1]['native_reason'],policy_input=(transport.speed_tracking or{}).get('policy_input_x'))
        if args.expect_baseline_failure:
            result['expected_baseline_failure']=not stable;result['passed']=not stable
        else:
            assert stable,result
            assert any((s.get('last_authority_sample')or{}).get('operator_echo_origin')=='native_packet_snapshot'for s in samples),'Native mirror was not actually consumed'
            assert all((s.get('last_authority_sample')or{})['operator_source_ns']in original_stamps for s in samples),'Original native sample timestamp changed'
            result['original_native_packet_snapshot_used']=True
            state['healthy']=False;deadline=time.monotonic()+.6
            while not transport.status()['cancel_pending']and time.monotonic()<deadline:time.sleep(.01)
            assert transport.status()['cancel_pending']and not transport.status()['navigation_owned']
            state['healthy']=True;time.sleep(.25);assert not transport.status()['navigation_owned']
            result.update(passed=True,actual_missing_health_revokes=True,recovery_does_not_auto_rearm=True)
    finally:
        stopping.set()
        if writer:writer.join(timeout=1)
        if transport:transport.close()
        if source:source.close()
        if 'snapshot'in locals():snapshot.close()
        proc.terminate();proc.communicate(timeout=2)
print(json.dumps(result),flush=True)
assert result['passed'],result
