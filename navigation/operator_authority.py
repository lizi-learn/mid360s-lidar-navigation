"""Read-only decoding of a commissioned 37-value receiver telemetry message.

The timestamp belongs to the original host receiver, not this observer. This
module cannot write joystick input, change policy, or command an actuator.
"""
import math

class ReceiverAuthority:
    def __init__(self, verified=False, timeout=.1):
        self.verified=verified is True;self.timeout=timeout
        self.publisher=None;self.last_stamp=0;self.sequence=0;self.clock_fault=False
        self.offset=None

    def decode(self, values, publisher, wall, mono, instance):
        if self.offset is None:self.offset=mono-wall
        if abs((mono-wall)-self.offset)>.02:self.clock_fault=True
        out=dict(switch_mode='unknown',switch_mapping_verified=False,
            receiver_connected=False,manual_activity=False,source_instance=instance,
            source_sequence=self.sequence+1,source_monotonic_ns=0,
            provenance='receiver_host_unix_us',fault='invalid_receiver_message')
        self.sequence+=1
        if (len(values)!=37 or not all(math.isfinite(v) for v in values)
                or any(v!=int(v) for v in values[:19])):return out
        stamp=int(values[0]);age=wall-stamp/1e6
        if self.clock_fault:out['fault']='host_clock_changed';return out
        if not publisher or instance<=0:out['fault']='publisher_unknown';return out
        if publisher==self.publisher and stamp<=self.last_stamp:
            out['fault']='replayed_or_reversed_receiver_time';return out
        # A changed ROS publisher starts a new identity in the bridge. Never
        # convert an old receiver sample into a fresh observation timestamp.
        self.publisher,self.last_stamp=publisher,stamp
        out['source_monotonic_ns']=int((stamp/1e6+self.offset)*1e9)
        out['source_unix_us']=stamp;out['source_age_s']=age
        if not -.005<=age<=self.timeout:out['fault']='stale_receiver_sample';return out
        if any(not 0<=v<=2047 for v in values[1:17]):return out
        if any(values[i] not in (0.,1.) for i in (17,18,32,36)):return out
        raw=int(values[5]);out['raw_e']=raw
        mode=('navigation_allowed' if 262<=raw<=302 else
              'paused' if 982<=raw<=1022 else 'manual' if 1700<=raw<=1745 else 'unknown')
        # Raw axes remain observable when the receiver masks mapped velocities
        # in E-left. A non-neutral raw stick must still cancel navigation.
        out['manual_activity']=any(abs(values[i]-1002)>20 for i in (1,3,4))
        connected=all(values[i]==0 for i in (17,18,32,36))
        out.update(switch_mode=mode,receiver_connected=connected,
            switch_mapping_verified=self.verified and mode!='unknown',
            fault='' if connected and mode!='unknown' else 'receiver_fault_or_unknown_switch')
        return out
