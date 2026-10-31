"""Bounded private command receipts; disk I/O never runs on a control/RPC tick."""
import json, os, queue, threading, time
from pathlib import Path

class VelocityTrace:
    def __init__(self,path,maximum_bytes=8*1024*1024):
        self.path=Path(path);self.maximum_bytes=maximum_bytes
        self.queue=queue.Queue(maxsize=128);self.dropped=0;self.error=''
        self.closed=False;self.last=0.
        self.thread=threading.Thread(target=self._run,name='velocity_receipt_writer',daemon=True);self.thread.start()
    def submit(self,value):
        now=time.monotonic()
        if now-self.last<.1:return
        self.last=now
        try:self.queue.put_nowait({**value,'trace_monotonic_ns':time.monotonic_ns(),'trace_dropped':self.dropped})
        except queue.Full:self.dropped+=1
    def _run(self):
        while not self.closed or not self.queue.empty():
            try:value=self.queue.get(timeout=.1)
            except queue.Empty:continue
            try:
                self.path.parent.mkdir(parents=True,exist_ok=True)
                if self.path.exists() and self.path.stat().st_size>=self.maximum_bytes:
                    self.path.replace(str(self.path)+'.1')
                fd=os.open(self.path,os.O_WRONLY|os.O_CREAT|os.O_APPEND|os.O_NOFOLLOW,0o600)
                with os.fdopen(fd,'a') as f:f.write(json.dumps(value,allow_nan=False,separators=(',',':'))+'\n')
            except (OSError,TypeError,ValueError) as e:self.error=str(e)
            finally:self.queue.task_done()
    def close(self):
        self.closed=True;self.thread.join(timeout=1.)
