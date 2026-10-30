"""Existing G12 raw telemetry -> fixed-size local read-only source socket.
Does not open serial, publish joystick data, change a button or write a motor.
"""
import socket,struct,time
NATIVE=struct.Struct('<II4QIIfffI')
assert NATIVE.size==64

def encode(result,values):
    mode={'navigation_allowed':1,'paused':2,'manual':3}.get(result.get('switch_mode'),0)
    flags=(int(result.get('receiver_connected') is True)|
           int(result.get('switch_mapping_verified') is True)<<1|
           int(result.get('manual_activity') is True)<<2)
    if len(values)==37 and values[32]==1:flags|=8
    def axis(i):
        v=max(-1.,min(1.,(values[i]-1002)/720.)) if len(values)==37 else 0.
        return 0. if abs(v)<.04 else v
    # Exact existing receiver/bridge signs, with raw axes retained in E-left.
    axes=(axis(3),-axis(4),-axis(1))
    return NATIVE.pack(0x4e4f5031,1,result['source_instance'],result['source_sequence'],
        max(0,int(result.get('source_monotonic_ns',0))),max(0,int(result.get('source_unix_us',0))),mode,flags,*axes,0)

class NativePublisher:
    def __init__(self,path):self.path=path;self.socket=None;self.retry_at=0.
    def send(self,result,values):
        if self.socket is None:
            if time.monotonic()<self.retry_at:return
            self.retry_at=time.monotonic()+.2
            s=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);s.settimeout(.001)
            try:s.connect(self.path)
            except OSError:s.close();return
            self.socket=s
        try:
            packet=encode(result,values)
            if self.socket.send(packet)!=len(packet):raise OSError('short_native_packet')
        except (OSError,ValueError,OverflowError,struct.error):self.close()
    def close(self):
        if self.socket is not None:self.socket.close()
        self.socket=None
