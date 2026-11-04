"""Bounded joint handoff replay, actual pure-arithmetic components.

The pivot and final input ramp are the production C++ implementations. Python
buffer, clear-hold, handoff and final gate are production classes. The monitor
receipt-suppression model and planar response are explicit synthetic models,
separately covered by the actual ROS monitor component check. No biped or
physical command acceptance is claimed. Old failures remain in the report.
"""
import argparse,collections,ctypes,hashlib,json,math,sys
from pathlib import Path
NAV=Path(__file__).resolve().parents[2]/'navigation';sys.path.insert(0,str(NAV))
from motion_buffer import MotionBuffer,MeasuredMotionMean
from gait_handoff import GaitHandoff
from clearance_recovery import ClearanceRecovery
from gate import ForwardGate,Limits

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--library',required=True);ap.add_argument('--output',required=True);a=ap.parse_args()
    lib=ctypes.CDLL(str(Path(a.library).resolve()));D=ctypes.c_double;P=ctypes.c_void_p
    lib.p2_handoff_create.restype=P;lib.p2_handoff_destroy.argtypes=[P]
    lib.p2_handoff_pivot.argtypes=[P,D,D,D,D];lib.p2_handoff_pivot.restype=D
    lib.p2_handoff_input.argtypes=[P,D,D,D,ctypes.c_ulonglong];lib.p2_handoff_input.restype=D
    root=Path(a.output);root.mkdir(parents=True,exist_ok=True);results=[]
    def run(variant,goal,ripple,drift=0.,fault=None):
        dt=.02;gain=.55;delay=.24;deadzone=.30
        native=lib.p2_handoff_create();buffer=MotionBuffer(accel=.4,decel=.8,angular_accel=.5,ramp_enabled=False)
        mean=MeasuredMotionMean();handoff=GaitHandoff();clear=ClearanceRecovery();gate=ForwardGate(Limits(max_v=.8,max_w=1.,accel=.4,decel=.8));gate.armed=True
        queued=collections.deque([0.]*(round(delay/dt)+1));yaw=rate=0.;measured=(0.,0.,0.);receipt=(0.,0.,0.);receipt_at=-99.;last_nonzero=1.;last_was_zero=True
        trace=[];reached=False;peak=0.;fault_bad=0;quiet=0.;clear_restarts=0;was_clear=False;first_yaw=None
        try:
            for i in range(4000):
                t=1.+i*dt
                if i%5==0:
                    #10Hz original source values, with9.6mm zero-net body sway
                    # at2Hz for amplitude.12m/s. It is not sustained drift.
                    measured=(drift+ripple*math.sin(4*math.pi*(t-1.)),0.,rate);source=t;average=mean.step(measured,source)
                age=t-source;error=math.remainder(goal-yaw,2*math.pi)
                raw=(0.,0.,lib.p2_handoff_pivot(native,error,yaw,rate,t))
                blocked=fault=='obstacle'and 3.<=t-1.<5.
                stale=fault=='stale'and 3.<=t-1.<4.5
                sensor_age=.7 if stale else age
                bounded=buffer.step(raw,average,t,sensor_age,hard_stop=blocked,frame_verified=True)
                zero=bounded==(0.,0.,0.)
                if zero and not last_was_zero:last_nonzero=t
                if not zero:last_nonzero=t
                # This model is independently tested against the actual ROS
                # monitor. An absent request is never manufactured here.
                if variant!='baseline' or not zero or t-last_nonzero<=2.:
                    receipt=(0.,0.,0.)if blocked else bounded;receipt_at=t
                last_was_zero=zero
                ready=(not blocked and not stale and t-receipt_at<=.3)
                recovered=clear.step(t,ready)
                if was_clear and not recovered:clear_restarts+=1
                was_clear=recovered
                command=receipt
                if not recovered:handoff.reset();command=(0.,0.,0.)
                elif variant!='single_phase':command=handoff.step(command,raw,measured,t,0.,sensor_age)
                safe=gate.step(command,dt,t-receipt_at,sensor_age,sensor_age)
                input_yaw=lib.p2_handoff_input(native,safe[2],rate,dt,round(t*1e9));peak=max(peak,abs(input_yaw))
                if(blocked or stale)and(abs(safe[2])>1e-12 or abs(input_yaw)>1e-12):fault_bad+=1
                queued.append(input_yaw);applied=queued.popleft();target=0. if abs(applied)<deadzone else gain*applied
                rate+=(target-rate)*(1-math.exp(-dt/.15));yaw+=rate*dt
                if first_yaw is None and abs(yaw)>.01:first_yaw=t-1.
                quiet=quiet+dt if abs(error)<=.03 and abs(rate)<.03 else 0.
                trace.append([t-1.,raw[2],bounded[2],receipt[2],t-receipt_at,recovered,safe[2],input_yaw,yaw,rate,*measured])
                if quiet>=.3 and(fault is None or t-1.>5.):reached=True;break
            result=dict(variant=variant,goal=goal,ripple_mps=ripple,sustained_drift_mps=drift,fault=fault,goal_reached=reached,duration_s=t-1.,yaw_error_rad=abs(math.remainder(goal-yaw,2*math.pi)),peak_native_input=peak,first_yaw_s=first_yaw,clear_hold_restarts=clear_restarts,fault_nonzero_outputs=fault_bad,
                        parameters=dict(dt=dt,gain=gain,delay=delay,deadzone=deadzone,clear_hold=.75,source_hz=10,command_timeout=.3,sensor_timeout=.5,body_total_side=.55,commanded_y=0,seed=0))
            label=f'{variant}-{len(results):02d}';(root/(label+'.trace.json')).write_text(json.dumps(trace));results.append(result);return result
        finally:lib.p2_handoff_destroy(native)
    for variant in ['baseline','zero_receipts','single_phase']:
        for ripple in [0.,.12]:run(variant,-.14,ripple)
    for goal in [.14,-.5,1.57,-1.57]:run('single_phase',goal,.12)
    run('single_phase',-.5,.12,fault='obstacle');run('single_phase',-.5,.12,fault='stale')
    drift=run('single_phase',-.14,0.,drift=.12)
    candidate=[r for r in results if r['variant']=='single_phase'and r['sustained_drift_mps']==0]
    passed=all(x['goal_reached']and not x['fault_nonzero_outputs']for x in candidate)and not drift['goal_reached']and drift['peak_native_input']==0.
    report=dict(passed=passed,scope=__doc__,loaded_arithmetic_library=str(Path(a.library).resolve()),library_sha256=hashlib.sha256(Path(a.library).read_bytes()).hexdigest(),cases=results,hardware_commands=0)
    (root/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items()if k!='scope'},indent=2));return 0 if passed else 1
if __name__=='__main__':raise SystemExit(main())
