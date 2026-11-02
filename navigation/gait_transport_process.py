"""Keep native control I/O and original ROS health outside the UI Python GIL.

Only this owned worker opens the existing native gait socket. No robot policy,
actuator or health contract is changed. IPC keys remain in inherited pipes.
"""
import os,secrets,select,subprocess,sys,tempfile,threading,time
from pathlib import Path
from multiprocessing.managers import BaseManager

EXPOSED=('ready','arm','stop','acknowledge_cancel','status','operator_authority','shutdown')
class ControlManager(BaseManager):pass
ControlManager.register('transport',exposed=EXPOSED)

class GaitTransportProcess:
    def __init__(self,path,trace_path=None):
        from native_operator_snapshot import NativeSnapshotReader
        self.native_snapshot=NativeSnapshotReader(path+'.operator.snapshot');self.native_instance=0
        self.directory=tempfile.TemporaryDirectory(prefix='p2-gait-control-')
        ipc=str(Path(self.directory.name)/'control.sock');key=secrets.token_bytes(32)
        kr,kw=os.pipe();rr,rw=os.pipe();lr,self.lifeline=os.pipe()
        self.running=True;self.operator_observer=None;self.observer_error='';self.fault=''
        command=[sys.executable,'-S',str(Path(__file__).resolve()),'--worker','--gait-socket',path,'--ipc',ipc,'--key-fd',str(kr),'--ready-fd',str(rw),'--parent-fd',str(lr)]
        if trace_path:command+=['--trace',str(trace_path)]
        self.process=subprocess.Popen(command,pass_fds=(kr,rw,lr))
        os.close(kr);os.close(rw);os.close(lr);os.write(kw,key);os.close(kw)
        try:
            assert select.select([rr],[],[],8.)[0],'Control worker startup timeout'
            if os.read(rr,1)!=b'1':raise RuntimeError('Control worker failed to start')
            self.manager=ControlManager(address=ipc,authkey=key);self.manager.connect();self.proxy=self.manager.transport()
        except BaseException:
            self.process.terminate();self.process.wait(timeout=3);os.close(self.lifeline);self.directory.cleanup();raise
        finally:os.close(rr)
        self.thread=threading.Thread(target=self._observe,daemon=True,name='native_authority_observer');self.thread.start()
    def _call(self,name,*args):
        if self.fault:raise OSError(self.fault)
        try:return getattr(self.proxy,name)(*args)
        except (EOFError,OSError,BrokenPipeError)as error:
            self.fault=type(error).__name__+': '+str(error)
            # An inaccessible control worker must not keep an autonomous lease.
            if self.process.poll()is None:self.process.terminate()
            raise OSError(self.fault)from error
    def _observe(self):
        while self.running:
            observer=self.operator_observer
            if observer:
                try:observer();self.observer_error=''
                except (OSError,ValueError,RuntimeError)as error:self.observer_error=str(error)
            time.sleep(.02)
    def update(self,gate,command_enabled):
        # The worker independently receives the SAME original ROS evaluations.
        # UI processing cannot renew or delay their source timestamps.
        pass
    def ready(self):
        try:return self._call('ready')
        except OSError:return False
    def arm(self,generation,timeout=2.):return self._call('arm',generation,timeout)
    def stop(self):
        try:return self._call('stop')
        except OSError:return None
    def acknowledge_cancel(self):
        try:return self._call('acknowledge_cancel')
        except OSError:return None
    def operator_authority(self):
        for _ in range(2):
            if self.native_instance:
                sample=self.native_snapshot.sample(self.native_instance)
                if sample:
                    from native_operator_snapshot import operator_authority
                    return operator_authority(sample)
            try:result=self._call('operator_authority')
            except OSError:return None
            if not result:return None
            if result.get('verified')and result.get('source'):
                self.native_instance=result['source'];sample=self.native_snapshot.sample(self.native_instance)
                if sample:
                    from native_operator_snapshot import operator_authority
                    return operator_authority(sample)
            now=time.monotonic()
            stamp=result['stamp']if result.get('source_transport')=='native_packet_snapshot'else result['reply_stamp']
            if -.005<=now-stamp<=.08:return result
            # Query a new actual snapshot after UI GIL starvation. Never relabel
            # the old reply or repeat ARM/velocity to recover a stale read.
        return result
    @property
    def phase(self):return self.status()['phase']
    def status(self):
        try:
            d=self._call('status');d['control_worker_pid']=self.process.pid;d['control_process_isolated']=True;d['observer_error']=self.observer_error;return d
        except OSError:
            return dict(phase='disconnected',execution_ready=False,error=self.fault,cancel_pending=False,navigation_owned=False,manual_owned=False,velocity=None,armed=False,readiness=0,operator_mode=0,native_reason='control_worker_unavailable',control_process_isolated=True)
    def close(self):
        self.running=False;self.operator_observer=None;self.stop()
        try:self._call('shutdown')
        except OSError:pass
        if self.lifeline is not None:os.close(self.lifeline);self.lifeline=None
        self.thread.join(timeout=.5)
        try:self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:self.process.terminate();self.process.wait(timeout=3)
        self.directory.cleanup()
        self.native_snapshot.close()


def worker(args):
    import json
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from gait_transport import GaitTransport
    key=os.read(args.key_fd,32);os.close(args.key_fd);assert len(key)==32
    stopping=threading.Event();transport=GaitTransport(args.gait_socket,trace_path=args.trace)
    class Service:
        def ready(self):return transport.ready()
        def arm(self,*a):return transport.arm(*a)
        def stop(self):return transport.stop()
        def acknowledge_cancel(self):return transport.acknowledge_cancel()
        def status(self):return transport.status()
        def operator_authority(self):return transport.operator_authority()
        def shutdown(self):stopping.set()
    service=Service();ControlManager.register('transport',callable=lambda:service,exposed=EXPOSED)
    server=ControlManager(address=args.ipc,authkey=key).get_server();server_thread=threading.Thread(target=server.serve_forever,daemon=True);server_thread.start()
    rclpy.init();node=Node('navigation_native_control_io')
    def health(msg):
        try:gate=json.loads(msg.data)
        except (ValueError,TypeError):return
        transport.update(gate,transport.phase=='active'and gate.get('armed')is True)
    node.create_subscription(String,'/p2_nav/gate_status',health,1)
    os.write(args.ready_fd,b'1');os.close(args.ready_fd)
    try:
        while not stopping.is_set()and rclpy.ok():
            # Kernel EOF follows parent exit, even when no orderly close runs.
            if select.select([args.parent_fd],[],[],0)[0] and not os.read(args.parent_fd,1):break
            rclpy.spin_once(node,timeout_sec=.02)
    finally:
        transport.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        server.stop_event.set();os.close(args.parent_fd)

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--worker',action='store_true');p.add_argument('--gait-socket',required=True);p.add_argument('--ipc',required=True);p.add_argument('--key-fd',type=int,required=True);p.add_argument('--ready-fd',type=int,required=True);p.add_argument('--parent-fd',type=int,required=True);p.add_argument('--trace');worker(p.parse_args())
