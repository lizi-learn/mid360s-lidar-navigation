"""Ordinary thread for explicit session->physical-speed lease, no actuator I/O."""
import threading,time,secrets
from gait_socket_client import GaitClient,Operation,Authority

LOCAL_READY=1|2|4|8|16|128

def readiness_error(r):
    checks=((1,'未选择 D13 策略'),(2,'步态尚未进入可行走状态'),(4,'硬件状态未就绪'),
        (8,'驱动保护未核验'),(16,'实机标定未核验'),(128,'真实手柄数据已过期'))
    missing=['遥控急停已锁定，请在现场复位'] if getattr(r,'reason','')=='estop' else []
    missing.extend(name for bit,name in checks if not r.readiness&bit)
    if not r.operator_verified:missing.append('手柄身份未核验')
    if r.operator_mode!=1:missing.append('E 未在导航档')
    if r.manual_activity:missing.append('摇杆已介入')
    if r.cancel_pending:missing.append('等待旧任务取消回执')
    return '；'.join(missing)

class GaitTransport:
    def __init__(self,path):
        self.path=path;self.lock=threading.RLock();self.running=True;self.client=None
        self.feedback=None;self.error='步态接口尚未连接';self.health_stamp=0;self.health_flags=0
        self.command=(0.,0.,0.);self.command_stamp=0;self.start_request=None
        self.operator_observer=None
        self.phase='idle';self.generation=None;self.stop_serial=0;self.sent_stop=0
        self.ack_requested=None;self.started=threading.Event();self.attempt_started=0.
        self.thread=threading.Thread(target=self._run,name='gait_velocity_rpc',daemon=True);self.thread.start()
    def ready(self):
        with self.lock:
            r=self.feedback
            return bool(r and getattr(r,'reason','')!='estop' and 0<=time.monotonic_ns()-r.monotonic_ns<=100_000_000 and
                r.readiness&LOCAL_READY==LOCAL_READY and r.operator_verified and r.operator_mode==1
                and not r.manual_activity and not r.cancel_pending and self.phase not in ('failed','disconnected'))
    def update(self,gate,command_enabled):
        with self.lock:
            self.health_stamp=int(gate.get('evaluated_monotonic_ns',0))
            self.health_flags=(8 if gate.get('data_fresh') else 0)|(16 if gate.get('calibration_ready') else 0)|\
                (32 if gate.get('ground_ok') else 0)|(64 if gate.get('localized') else 0)
            self.command=tuple(gate.get('command',(0.,0.,0.))) if command_enabled else (0.,0.,0.)
            self.command_stamp=self.health_stamp
    def arm(self,generation,timeout=2.):
        with self.lock:
            if not self.ready():raise ValueError(self.error or '当前步态或物理开关未就绪')
            self.started.clear();self.start_request=generation;self.phase='requested';self.error=''
        if not self.started.wait(timeout):self.stop();raise ValueError('未收到真实步态零速度接管回执')
        with self.lock:
            if self.phase!='active':raise ValueError(self.error or '步态接管被撤销')
    def stop(self):
        with self.lock:
            self.stop_serial+=1;self.start_request=None;self.generation=None
            self.command=(0.,0.,0.);self.phase='stopped';self.started.set()
    def acknowledge_cancel(self):
        # Caller must have actual terminal receipts for the old Nav2 action.
        with self.lock:
            r=self.feedback
            self.ack_requested=(r.controller,r.generation,r.transport_epoch,r.cancel_generation) if r and r.cancel_pending else None
    def _fault(self,message):
        with self.lock:
            self.error=message;self.phase='disconnected';self.feedback=None
            self.start_request=None;self.generation=None;self.started.set()
        if self.client:self.client.close()
        self.client=None
    def _run(self):
        while self.running:
            start=time.monotonic()
            try:
                if self.client is None:self.client=GaitClient(self.path,secrets.randbits(63)+1)
                with self.lock:
                    r=self.feedback;flags=self.health_flags;stamp=self.health_stamp
                    authority=Authority(r.receiver_instance if r else 0,r.receiver_generation if r else 0,
                        r.native_instance if r else 0,r.native_stamp if r else 0,stamp,0,flags)
                r=self.client.exchange(Operation.AUTHORITY,authority)
                with self.lock:
                    self.feedback=r
                    if self.phase=='disconnected':self.phase='idle'
                    if self.phase in ('idle','stopped'):self.error=readiness_error(r)
                    authority=Authority(r.receiver_instance,r.receiver_generation,r.native_instance,r.native_stamp,
                        self.health_stamp,0,self.health_flags)
                    if self.stop_serial!=self.sent_stop:
                        self.client.exchange(Operation.STOP,authority);self.sent_stop=self.stop_serial
                    elif self.ack_requested:
                        expected=(r.controller,r.generation,r.transport_epoch,r.cancel_generation)
                        if r.cancel_pending and self.ack_requested==expected:
                            self.client.exchange(Operation.CANCEL_ACK,authority)
                        self.ack_requested=None
                    elif self.phase=='requested':
                        self.client.exchange(Operation.ARM,authority);self.phase='arming';self.attempt_started=time.monotonic()
                    elif self.phase=='arming' and r.armed:
                        self.client.exchange(Operation.VELOCITY,authority);self.phase='handshake'
                    elif self.phase=='handshake' and r.navigation_owned:
                        self.phase='active';self.generation=self.start_request;self.start_request=None;self.started.set()
                    elif self.phase=='active':
                        if not r.navigation_owned or r.cancel_pending:
                            self.phase='failed';self.error='真实步态已撤销导航接管';self.generation=None
                        else:self.client.exchange(Operation.VELOCITY,authority,self.command,self.command_stamp)
                    if self.phase in ('arming','handshake') and time.monotonic()-self.attempt_started>.3:
                        self.phase='failed';self.error='真实步态未确认接管';self.started.set();self.stop_serial+=1
            except (OSError,ValueError,KeyError,TypeError,OverflowError) as exc:self._fault(str(exc))
            observer=self.operator_observer
            if observer:observer()
            time.sleep(max(.001,.02-(time.monotonic()-start)))
    def operator_authority(self):
        with self.lock:
            r=self.feedback
            if not r:return None
            return {'mode':{1:'navigation_allowed',2:'paused',3:'manual'}.get(r.operator_mode,'unknown'),
                'verified':bool(r.operator_verified and r.readiness&128),
                'connected':bool(r.operator_verified and r.readiness&128),
                'activity':bool(r.manual_activity),'source':r.native_instance,'sequence':r.native_sequence,
                'stamp':r.native_stamp/1e9,'reply_stamp':r.monotonic_ns/1e9}
    def status(self):
        with self.lock:
            r=self.feedback
            return {'phase':self.phase,'execution_ready':self.ready(),'error':self.error,
                'cancel_pending':bool(r and r.cancel_pending),'cancel_generation':r.cancel_generation if r else 0,
                'controller_instance':r.controller if r else None,'policy_generation':r.generation if r else None,
                'navigation_owned':bool(r and r.navigation_owned),'manual_owned':bool(r and r.manual_owned),
                'tick':r.tick if r else 0,'velocity':[r.vx,r.vy,r.wz] if r else None,
                'armed':bool(r and r.armed),'force_zero':bool(r and r.force_zero),
                'receiver_instance':r.receiver_instance if r else 0,'receiver_generation':r.receiver_generation if r else 0,
                'native_source_sequence':r.native_sequence if r else 0,
                'native_source_age_ms':(time.monotonic_ns()-r.native_stamp)/1e6 if r and r.native_stamp else None,
                'readiness':r.readiness if r else 0,'native_reason':getattr(r,'reason','') if r else '',
                'blockers':readiness_error(r) if r else '步态接口尚未连接',
                'operator_mode':r.operator_mode if r else 0,'source_monotonic_ns':r.monotonic_ns if r else None}
    def close(self):
        self.stop();self.running=False;self.thread.join(timeout=.5)
        if self.client:self.client.close()
