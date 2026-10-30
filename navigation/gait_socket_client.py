"""Explicit local gait RPC client. No ROS/HTTP fallback or automatic rearm.

The caller supplies original operator/health sample times. Refreshing a transport
packet never refreshes these samples. Physical deployment requires an explicit, separately reviewed SDK adapter.
"""
import dataclasses,enum,math,socket,struct,time

MAGIC=0x504E4731
HEADER=struct.Struct('<4I12Q2I')
VELOCITY=struct.Struct('<IHHQQQfffI')
REPLY=struct.Struct('<2I12Q8I3fI96s')
assert HEADER.size+VELOCITY.size==168 and REPLY.size==248

class Operation(enum.IntEnum):
    AUTHORITY=1
    ARM=2
    VELOCITY=3
    STOP=4
    CANCEL_ACK=5

@dataclasses.dataclass(frozen=True)
class Authority:
    receiver_instance:int
    receiver_generation:int
    operator_instance:int
    operator_stamp_ns:int
    health_stamp_ns:int
    mode:int
    flags:int

@dataclasses.dataclass(frozen=True)
class Reply:
    tick:int;monotonic_ns:int;controller:int;generation:int
    receiver_instance:int;receiver_generation:int;seen_command:int
    cancel_generation:int;transport_epoch:int
    native_instance:int;native_stamp:int;native_sequence:int
    armed:int;navigation_owned:int;force_zero:int;cancel_pending:int
    manual_owned:int;operator_mode:int;operator_verified:int;manual_activity:int
    vx:float;vy:float;wz:float;readiness:int;reason:str

def decode_reply(data,now_ns):
    if len(data)!=REPLY.size:raise ValueError('gait_reply_size')
    magic,version,*v=REPLY.unpack(data)
    if magic!=MAGIC or version!=1:raise ValueError('gait_reply_version')
    v[-1]=v[-1].split(b'\0',1)[0].decode('ascii',errors='strict');r=Reply(*v)
    if not r.monotonic_ns or not -5_000_000<=now_ns-r.monotonic_ns<=100_000_000:
        raise ValueError('gait_reply_expired')
    if any(x not in (0,1) for x in (r.armed,r.navigation_owned,r.force_zero,r.cancel_pending)):
        raise ValueError('gait_reply_flags')
    if not all(math.isfinite(x) for x in (r.vx,r.vy,r.wz)) or r.readiness&~255:
        raise ValueError('gait_reply_nonfinite')
    return r

def encode_frame(operation,client,sequence,now_ns,authority,reply=None,velocity=(0.,0.,0.),command_stamp_ns=None):
    if operation==Operation.VELOCITY:
        vx,vy,wz=velocity
        if not all(math.isfinite(v) for v in velocity) or not(0<=vx<=.15 and vy==0 and abs(wz)<=.24):
            raise ValueError('physical_forward_speed_contract')
    elif velocity!=(0.,0.,0.):raise ValueError('velocity_only_in_velocity_operation')
    if not client or not sequence:raise ValueError('missing_client_identity')
    if authority.mode not in (0,1,2,3) or authority.flags&~127:raise ValueError('authority_flags')
    get=lambda field:getattr(reply,field) if reply else 0
    frame=HEADER.pack(MAGIC,1,int(operation),authority.flags,
        client,sequence,now_ns,get('controller'),get('generation'),get('cancel_generation'),get('transport_epoch'),
        authority.receiver_instance,authority.receiver_generation,authority.operator_instance,
        authority.operator_stamp_ns,authority.health_stamp_ns,authority.mode,0)
    return frame+VELOCITY.pack(0x504E4156,1,1,client,sequence,now_ns if command_stamp_ns is None else command_stamp_ns,*velocity,0)

class GaitClient:
    def __init__(self,path,client_instance):
        if not client_instance:raise ValueError('client_instance_required')
        self.client=client_instance;self.sequence=0;self.reply=None
        self.socket=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET)
        self.socket.settimeout(.1)
        try:self.socket.connect(path)
        except BaseException:self.socket.close();raise
    def exchange(self,operation,authority,velocity=(0.,0.,0.),command_stamp_ns=None):
        # ARM is a separate explicit call. Caller must wait for its gait-loop
        # acknowledgement and complete a zero handshake before nonzero input.
        if operation!=Operation.AUTHORITY and self.reply is None:
            raise ValueError('no_actual_controller_feedback')
        self.sequence+=1
        frame=encode_frame(operation,self.client,self.sequence,time.monotonic_ns(),authority,self.reply,velocity,command_stamp_ns)
        try:
            if self.socket.send(frame)!=len(frame):raise ValueError('short_gait_request')
            r=decode_reply(self.socket.recv(REPLY.size+1),time.monotonic_ns())
            if self.reply and (r.controller,r.generation,r.transport_epoch)!=(self.reply.controller,self.reply.generation,self.reply.transport_epoch):
                raise ValueError('controller_or_connection_changed_explicit_reconnect_required')
            self.reply=r;return r
        except BaseException:
            self.close();raise
    def close(self):self.socket.close()
