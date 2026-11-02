"""Read-only mirror of the exact native packet already sent to the gait loop.

The snapshot lives beside the owned Unix socket in tmpfs. POSIX record locks
prevent torn reads across processes. No timestamp, identity or mode is created
by a reader, and a missing/stale/different source cannot refresh authority.
"""
import fcntl,math,os,stat,time
from native_operator_source import NATIVE

class NativeSnapshotWriter:
    def __init__(self,path):
        from pathlib import Path
        p=Path(path);parent=p.parent.stat();uid=parent.st_uid
        if not stat.S_ISDIR(parent.st_mode)or parent.st_mode&0o077 or os.geteuid()not in(0,uid):raise OSError('Untrusted native snapshot directory')
        self.fd=os.open(p,os.O_CREAT|os.O_WRONLY|os.O_CLOEXEC|os.O_NOFOLLOW,0o600)
        try:
            if os.geteuid()==0:os.fchown(self.fd,uid,-1)
            meta=os.fstat(self.fd)
            if not stat.S_ISREG(meta.st_mode)or meta.st_uid!=uid or meta.st_mode&0o077 or meta.st_size not in(0,NATIVE.size):raise OSError('Untrusted native snapshot file')
        except BaseException:os.close(self.fd);raise
    def publish(self,packet):
        if len(packet)!=NATIVE.size:raise ValueError('Invalid native packet size')
        try:fcntl.lockf(self.fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:return False
        try:return os.pwrite(self.fd,packet,0)==NATIVE.size
        finally:fcntl.lockf(self.fd,fcntl.LOCK_UN)
    def close(self):os.close(self.fd)

class NativeSnapshotReader:
    def __init__(self,path):self.path=path;self.fd=None
    def sample(self,instance,now_ns=None):
        if not instance:return None
        now=time.monotonic_ns()if now_ns is None else now_ns
        try:
            if self.fd is None:
                fd=os.open(self.path,os.O_RDONLY|os.O_CLOEXEC|os.O_NOFOLLOW)
                meta=os.fstat(fd)
                if not stat.S_ISREG(meta.st_mode)or meta.st_uid not in(0,os.geteuid())or meta.st_mode&0o077 or meta.st_size!=NATIVE.size:
                    os.close(fd);return None
                self.fd=fd
            fcntl.lockf(self.fd,fcntl.LOCK_SH|fcntl.LOCK_NB)
            try:data=os.pread(self.fd,NATIVE.size,0)
            finally:fcntl.lockf(self.fd,fcntl.LOCK_UN)
            if len(data)!=NATIVE.size:return None
            magic,version,source,sequence,stamp,unix_us,mode,flags,x,y,yaw,reserved=NATIVE.unpack(data)
            if (magic!=0x4e4f5031 or version!=1 or source!=instance or not sequence or not stamp or
                    not -5_000_000<=now-stamp<=100_000_000 or mode not in(1,2,3)or
                    flags&3!=3 or flags&~15 or reserved or not all(math.isfinite(v)and abs(v)<=1. for v in(x,y,yaw))):return None
            return dict(source=source,sequence=sequence,stamp_ns=stamp)
        except OSError:return None
    def close(self):
        if self.fd is not None:os.close(self.fd);self.fd=None
