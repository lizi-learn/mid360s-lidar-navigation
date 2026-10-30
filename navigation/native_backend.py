"""Optional isolated dynamics backend for the headless navigation simulator.

External executable supplies a floating-base model and policy. It must implement
the local lease and STATE-line contracts; it is never a physical gait executable.
"""
import math,os,signal,socket,struct,subprocess,tempfile,threading,time
from pathlib import Path
from collections import deque

def sensor_sample(line):
    values=tuple(float(v) for v in line.split()[1:])
    if not line.startswith('SENSOR ') or len(values)!=16 or not all(math.isfinite(v) for v in values) or values[0]<0:
        raise ValueError('Invalid 3D backend sample')
    for offset in (4,11):
        if not .8<math.sqrt(sum(v*v for v in values[offset:offset+4]))<1.2:raise ValueError('Invalid model quaternion')
    return values

PACKET=struct.Struct('<IHHQQQfffI')
class NativeBackend:
    def __init__(self):
        binary=Path(os.environ['P2_NAV_NATIVE_EXEC']).resolve()
        config=Path(os.environ['P2_NAV_NATIVE_CONFIG']).resolve();model=Path(os.environ['P2_NAV_NATIVE_MODEL']).resolve()
        if not all(p.is_file() for p in (binary,config,model)):raise ValueError('Dynamics backend assets missing')
        self.directory=tempfile.TemporaryDirectory(prefix='p2-nav-sim-');self.address=str(Path(self.directory.name)/'command.sock')
        self.sequence=0;self.instance=(time.monotonic_ns()^(os.getpid()<<20))&((1<<64)-1)
        self.socket=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);self.lock=threading.Lock();self.state=None;self.at=0.
        trace=os.environ.get('P2_NAV_NATIVE_TRACE',str(Path(self.directory.name)/'dynamics.csv'))
        duration=float(os.getenv('P2_NAV_NATIVE_SECONDS','400'))
        if not 30<=duration<=600:raise ValueError('Dynamics run must be bounded to 30–600 seconds')
        args=[str(binary),str(config),trace,str(duration),str(model),self.address]
        self.process=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1,start_new_session=True)
        self.tracker=None;self.command_at=time.monotonic()
        if os.getenv('P2_NAV_VELOCITY_TRACKING')=='1':
            from gait_velocity import GaitVelocityTracker
            self.tracker=GaitVelocityTracker()
        self.sensors=deque(maxlen=4096);self.collect_sensors=os.getenv('P2_NAV_COLLECT_3D')=='1';self.dropped_sensor_samples=0;self.protocol_errors=0
        self.lines=[];self.thread=threading.Thread(target=self.read,daemon=True);self.thread.start()
    def read(self):
        for line in self.process.stdout:
            try:
                if line.startswith('STATE '):
                    values=[float(v) for v in line.split()[1:]]
                    if len(values)!=8 or not all(math.isfinite(v) for v in values):raise ValueError('Invalid STATE sample')
                    with self.lock:self.state=values;self.at=time.monotonic()
                elif line.startswith('SENSOR '):
                    values=sensor_sample(line)
                    if self.collect_sensors:
                        with self.lock:
                            if len(self.sensors)==self.sensors.maxlen:self.dropped_sensor_samples+=1
                            self.sensors.append(values)
                else:self.lines.append(line.strip());self.lines=self.lines[-30:]
            except ValueError:self.protocol_errors+=1
    def drain_sensor_samples(self):
        with self.lock:
            samples=list(self.sensors);self.sensors.clear();return samples
    def send(self,v,w):
        now=time.monotonic();dt=now-self.command_at;self.command_at=now
        if self.tracker:
            state,at=self.latest()
            v,w=self.tracker.step((v,w),(state[4],state[6]) if state else (0.,0.),dt,now-at if state else 999.)
        self.sequence+=1
        packet=PACKET.pack(0x504e4156,1,1,self.instance,self.sequence,time.monotonic_ns(),float(v),0.,float(w),0)
        try:self.socket.sendto(packet,self.address)
        except (FileNotFoundError,ConnectionRefusedError):pass
    def latest(self):
        with self.lock:return self.state,self.at
    def close(self):
        self.send(0.,0.)
        if self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGTERM)
            try:self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(self.process.pid,signal.SIGKILL);self.process.wait()
        self.socket.close();self.directory.cleanup()
