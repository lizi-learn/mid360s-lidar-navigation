"""Bounded stride/turn pipeline comparison; synthetic plant, no robot commands."""
import argparse,collections,ctypes,hashlib,json,math,sys
from pathlib import Path
NAV=Path(__file__).resolve().parents[2]/'navigation';sys.path.insert(0,str(NAV))
from motion_buffer import MotionBuffer,MeasuredMotionMean

def main():
 p=argparse.ArgumentParser();p.add_argument('--library',required=True);p.add_argument('--output',required=True);a=p.parse_args()
 lib=ctypes.CDLL(str(Path(a.library).resolve()));D=ctypes.c_double;P=ctypes.c_void_p
 lib.p2_handoff_create.restype=P;lib.p2_handoff_destroy.argtypes=[P]
 lib.p2_handoff_pivot.argtypes=[P,D,D,D,D];lib.p2_handoff_pivot.restype=D
 lib.p2_handoff_input.argtypes=[P,D,D,D,ctypes.c_ulonglong];lib.p2_handoff_input.restype=D
 out=Path(a.output);out.mkdir(parents=True,exist_ok=True);results=[]
 for window in [0.,.5]:
  for goal,gain,delay,dead in [(1.57,.55,.24,.3),(-1.57,.55,.24,.3),(.7,.8,.8,.55),(-.7,.8,.8,.55)]:
   native=lib.p2_handoff_create();mean=MeasuredMotionMean(translation_window=window);buffer=MotionBuffer(ramp_enabled=False)
   dt=.02;queue=collections.deque([0.]*(round(delay/dt)+1));yaw=rate=quiet=0.;zeros=0;trace=[];reached=False;raw_starts=0;prev=0.
   try:
    for i in range(5000):
     t=1.+i*dt;error=math.remainder(goal-yaw,2*math.pi)
     if i%5==0:source=t;measured=mean.step((0.,.2*math.sin(4*math.pi*t),rate),source)
     raw=lib.p2_handoff_pivot(native,error,yaw,rate,t)
     request=buffer.step((0.,0.,raw),measured,t,t-source,frame_verified=True)[2]
     actual=lib.p2_handoff_input(native,request,rate,dt,round(t*1e9));queue.append(actual);applied=queue.popleft()
     desired=0. if abs(applied)<dead else gain*applied
     rate+=(desired-rate)*(1-math.exp(-dt/.15));yaw+=rate*dt
     if request==0. and abs(error)>.15:zeros+=1
     if request!=0. and prev==0.:raw_starts+=1
     prev=request;trace.append([t-1.,error,raw,request,actual,yaw,rate,*measured])
     quiet=quiet+dt if abs(error)<=.15 and abs(rate)<.03 and raw==0. else 0.
     if quiet>=1.:reached=True;break
    row=dict(translation_window=window,goal=goal,gain=gain,delay=delay,deadzone=dead,goal_reached=reached,duration_s=t-1.,yaw_error_rad=abs(math.remainder(goal-yaw,2*math.pi)),zero_s_while_turn_needed=zeros*dt,turn_starts=raw_starts,seed=0,source_hz=10,ripple_mps=.2,ripple_hz=2)
    results.append(row);(out/(str(len(results))+'.trace.json')).write_text(json.dumps(trace))
   finally:lib.p2_handoff_destroy(native)
 paired=[{'baseline':results[i],'candidate':results[i+4]}for i in range(4)]
 passed=all(v['candidate']['goal_reached'] and v['candidate']['duration_s']<v['baseline']['duration_s'] and v['candidate']['zero_s_while_turn_needed']<v['baseline']['zero_s_while_turn_needed']for v in paired)
 report=dict(passed=passed,scope=__doc__,cases=paired,native_library=str(Path(a.library).resolve()),library_sha256=hashlib.sha256(Path(a.library).read_bytes()).hexdigest(),hardware_commands=0)
 (out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report));return 0 if passed else 1
if __name__=='__main__':raise SystemExit(main())
