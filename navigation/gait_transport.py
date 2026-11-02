"""Ordinary thread for explicit session->physical-speed lease, no actuator I/O."""
import threading,time,secrets,math
from gait_socket_client import GaitClient,Operation,Authority

LOCAL_READY=1|2|4|8|128|256

def readiness_error(r):
    checks=((1,'未选择 D13 策略'),(2,'步态尚未进入可行走状态'),(4,'硬件状态未就绪'),
        (8,'驱动保护未核验'),(256,'实时机身坐标数据未就绪'),(128,'真实手柄数据已过期'))
    missing=['遥控急停已锁定，请在现场复位'] if getattr(r,'reason','')=='estop' else []
    missing.extend(name for bit,name in checks if not r.readiness&bit)
    if not r.operator_verified:missing.append('手柄身份未核验')
    if r.operator_mode!=1:missing.append('E 未在导航档')
    if r.manual_activity:missing.append('摇杆已介入')
    if r.cancel_pending:missing.append('等待旧任务取消回执')
    return '；'.join(missing)

class GaitTransport:
    def __init__(self,path,trace_path=None):
        from native_operator_snapshot import NativeSnapshotReader
        self.native_snapshot=NativeSnapshotReader(path+'.operator.snapshot')
        self.path=path;self.lock=threading.RLock();self.running=True;self.client=None
        self.last_authority_sample=None;self.feedback=None;self.error='步态接口尚未连接';self.health_stamp=0;self.health_flags=0
        self.command=(0.,0.,0.);self.command_stamp=0;self.start_request=None
        self.measured=None;self.measured_source_ns=0;self.speed_tracking=None
        self.stage_commands={};self.trace=None
        if trace_path:
            from velocity_trace import VelocityTrace
            self.trace=VelocityTrace(trace_path)
        self.operator_observer=None
        self.phase='idle';self.generation=None;self.stop_serial=0;self.sent_stop=0
        self.ack_requested=None;self.started=threading.Event();self.attempt_started=0.;self.arm_fence_ns=0
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
            self.stage_commands=gate.get('velocity_stages',{})
            measured=gate.get('body_velocity_feedback')
            if (isinstance(measured,(tuple,list)) and len(measured)==3 and
                    all(isinstance(v,(int,float)) and math.isfinite(v) for v in measured)):
                self.measured=tuple(measured);self.measured_source_ns=int(gate.get('velocity_source_monotonic_ns',0))
            else:self.measured=None;self.measured_source_ns=0
    def arm(self,generation,timeout=2.):
        with self.lock:
            if not self.ready():raise ValueError(self.error or '当前步态或物理开关未就绪')
            self.started.clear();self.start_request=generation;self.phase='requested';self.error='';self.arm_fence_ns=0
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
            self.error=message;self.phase='disconnected';self.feedback=None;self.speed_tracking=None
            self.start_request=None;self.generation=None;self.started.set()
        if self.client:self.client.close()
        self.client=None
    def _authority(self,r):
        # Every reply carries an original native sample. Prefer the latest
        # actual received reply, including velocity/measurement replies.
        latest=getattr(self.client,'reply',None)
        r=latest if latest is not None else r
        with self.lock:
            sample=self.native_snapshot.sample(r.native_instance)if r else None
            stamp=sample['stamp_ns']if sample and sample['stamp_ns']>=r.native_stamp else(r.native_stamp if r else 0)
            self.operator_echo_origin='native_packet_snapshot'if sample and stamp==sample['stamp_ns']else'loop_reply'
            return Authority(r.receiver_instance if r else 0,r.receiver_generation if r else 0,
                r.native_instance if r else 0,stamp,self.health_stamp,0,self.health_flags)
    def _exchange_authority(self,r):
        authority=self._authority(r)
        reply=self.client.exchange(Operation.AUTHORITY,authority)
        with self.lock:
            self.feedback=reply
            self.last_authority_sample={'health_source_ns':authority.health_stamp_ns,'health_flags':authority.flags,
                'operator_source_ns':authority.operator_stamp_ns,'reply_source_ns':reply.monotonic_ns,
                'health_age_at_reply_ms':(reply.monotonic_ns-authority.health_stamp_ns)/1e6 if authority.health_stamp_ns else None,
                'operator_echo_age_at_reply_ms':(reply.monotonic_ns-authority.operator_stamp_ns)/1e6 if authority.operator_stamp_ns else None,
                'reply_operator_source_ns':reply.native_stamp}
            self.last_authority_sample['operator_echo_origin']=self.operator_echo_origin
        return reply
    def _run(self):
        while self.running:
            start=time.monotonic()
            try:
                if self.client is None:self.client=GaitClient(self.path,secrets.randbits(63)+1)
                with self.lock:r=self.feedback
                r=self._exchange_authority(r)
                with self.lock:
                    if self.phase=='disconnected':self.phase='idle'
                    if self.phase in ('idle','stopped'):self.error=readiness_error(r)
                    measured=self.measured;measured_stamp=self.measured_source_ns
                    pending_stop=self.stop_serial!=self.sent_stop
                operation=None;velocity=(0.,0.,0.);command_stamp=None;stop_serial=None;ack=None
                with self.lock:
                    if self.stop_serial!=self.sent_stop:
                        operation=Operation.STOP;stop_serial=self.stop_serial
                    elif self.ack_requested:
                        expected=(r.controller,r.generation,r.transport_epoch,r.cancel_generation)
                        ack=self.ack_requested
                        if r.cancel_pending and ack==expected:operation=Operation.CANCEL_ACK
                        else:self.ack_requested=None
                    elif self.phase=='requested':
                        operation=Operation.ARM;self.phase='arming';self.attempt_started=time.monotonic()
                    elif self.phase=='arming' and r.armed:
                        if not self.arm_fence_ns:self.arm_fence_ns=r.monotonic_ns
                        if self.command_stamp>=self.arm_fence_ns:
                            operation=Operation.VELOCITY;command_stamp=self.command_stamp;self.phase='handshake'
                    elif self.phase=='handshake' and r.navigation_owned:
                        self.phase='active';self.generation=self.start_request;self.start_request=None;self.started.set()
                    elif self.phase=='active':
                        if not r.navigation_owned or r.cancel_pending:
                            self.phase='failed';self.error='真实步态已撤销导航接管';self.generation=None
                        else:operation=Operation.VELOCITY;velocity=self.command;command_stamp=self.command_stamp
                    if self.phase in ('arming','handshake') and time.monotonic()-self.attempt_started>.3:
                        self.phase='failed';self.error='真实步态未确认接管';self.started.set();self.stop_serial+=1
                        operation=Operation.STOP;stop_serial=self.stop_serial
                if operation is not None:
                    reply=self.client.exchange(operation,self._authority(r),velocity,command_stamp)
                    with self.lock:
                        self.feedback=reply
                        if stop_serial is not None:self.sent_stop=stop_serial
                        if ack is not None and self.ack_requested==ack:self.ack_requested=None
                    # Do not let a slow control reply age the receiver echo
                    # across the following measured-feedback exchange.
                    r=self._exchange_authority(reply)
                with self.lock:
                    send_measured=(measured is not None and measured_stamp and
                        self.stop_serial==self.sent_stop and self.phase not in ('requested','arming','handshake'))
                if send_measured:
                    reply=self.client.exchange(Operation.MEASURED_VELOCITY,self._authority(r),measured,measured_stamp)
                    with self.lock:
                        self.feedback=reply;self.speed_tracking=self.client.tracking
                    # Original health/receiver sample times stay unchanged.
                    # Refresh the transport echo from THIS received reply,
                    # rather than the one from before the measured exchange.
                    r=self._exchange_authority(reply)
            except (OSError,ValueError,KeyError,TypeError,OverflowError) as exc:self._fault(str(exc))
            observer=self.operator_observer
            if observer:observer()
            if self.trace:
                with self.lock:
                    self.trace.submit({'phase':self.phase,'error':self.error,'generation':self.generation,
                        'target':self.command,'target_source_ns':self.command_stamp,
                        'velocity_stages':self.stage_commands,'speed_tracking':self.speed_tracking,
                        'health_source_ns':self.health_stamp,'health_flags':self.health_flags,
                        'last_authority_sample':self.last_authority_sample,
                        'measured_source_ns':self.measured_source_ns,
                        'readiness':self.feedback.readiness if self.feedback else None,
                        'policy_generation':self.feedback.generation if self.feedback else None,
                        'controller_instance':self.feedback.controller if self.feedback else None,
                        'armed':bool(self.feedback and self.feedback.armed),
                        'navigation_owned':bool(self.feedback and self.feedback.navigation_owned),
                        'controller_tick':self.feedback.tick if self.feedback else None,
                        'native_reason':self.feedback.reason if self.feedback else None})
            time.sleep(max(.001,.02-(time.monotonic()-start)))
    def operator_authority(self):
        with self.lock:
            r=self.feedback
            if not r:return None
            sample=self.native_snapshot.sample(r.native_instance)
            if sample and sample['stamp_ns']>=r.native_stamp:
                from native_operator_snapshot import operator_authority
                return operator_authority(sample)
            return {'mode':{1:'navigation_allowed',2:'paused',3:'manual'}.get(r.operator_mode,'unknown'),
                'verified':bool(r.operator_verified and r.readiness&128),
                'connected':bool(r.operator_verified and r.readiness&128),
                'activity':bool(r.manual_activity),'source':r.native_instance,'sequence':r.native_sequence,
                'stamp':r.native_stamp/1e9,'reply_stamp':r.monotonic_ns/1e9,'source_transport':'gait_reply'}
    def status(self):
        with self.lock:
            r=self.feedback
            return {'phase':self.phase,'execution_ready':self.ready(),'error':self.error,
                'cancel_pending':bool(r and r.cancel_pending),'cancel_generation':r.cancel_generation if r else 0,
                'last_authority_sample':self.last_authority_sample,
                'controller_instance':r.controller if r else None,'policy_generation':r.generation if r else None,
                'navigation_owned':bool(r and r.navigation_owned),'manual_owned':bool(r and r.manual_owned),
                'tick':r.tick if r else 0,'velocity':[r.vx,r.vy,r.wz] if r else None,
                'velocity_semantics':'D_policy_manual_equivalent_input_not_measured_speed',
                'speed_tracking':self.speed_tracking,
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
        if self.trace:self.trace.close()
        self.native_snapshot.close()
