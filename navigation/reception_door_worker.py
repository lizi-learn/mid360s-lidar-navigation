"""One latest-scan slot. Projected-ray processing never blocks the control timer."""
import math
import os
import threading
import time
import numpy as np
from reception_doors import DoorRays

def accept_rows(decisions,document,rows,now):
    accepted=0
    for row in rows:
        # Partial visibility is not contradictory evidence. It cannot refresh
        # the original full-opening evidence's lifetime, either.
        if row['state']=='unknown':continue
        accepted+=bool(decisions.observe({k:row[k]for k in ('door_id','state','source_monotonic','source_sequence')},now,document['policy']['clear_hold']))
    return accepted

class DoorWorker:
    def __init__(self,document,consume,refresh):
        self.document=document;self.consume=consume;self.refresh=refresh
        self.lock=threading.Lock();self.packet=None;self.event=threading.Event();self.closed=threading.Event()
        self.rays=DoorRays();self.sequence=0;self.last_source_ns=0;self.last_processed=-math.inf
        self.summary={'source':'projected_scan','state':'waiting','frames':0,'source_ns':0,'processing_ms':0.,'doors':[]}
        self.thread=threading.Thread(target=self.run,daemon=True,name='reception-door-rays');self.thread.start()
    def submit(self,packet):
        with self.lock:self.packet=packet
        self.event.set()
    def run(self):
        try:os.setpriority(os.PRIO_PROCESS,threading.get_native_id(),10)
        except (AttributeError,OSError):pass
        while not self.closed.is_set():
            self.event.wait(.2);self.event.clear()
            with self.lock:packet=self.packet;self.packet=None
            now=time.monotonic()
            try:
                document=self.document()
                if packet and packet['source_ns']>self.last_source_ns and now-self.last_processed>=.15 and document['doors']:
                    self.last_source_ns=packet['source_ns'];self.last_processed=now;start=time.perf_counter()
                    values=np.asarray(packet['ranges'],dtype=float);angles=packet['angle_min']+np.arange(len(values))*packet['angle_increment']+packet['yaw']
                    valid=np.isfinite(values)&(values>=packet['range_min'])&(values<=packet['range_max'])
                    # An infinity/empty 3D projection bin cannot establish a ray.
                    points=np.column_stack((packet['x']+values[valid]*np.cos(angles[valid]),packet['y']+values[valid]*np.sin(angles[valid])))
                    if len(points)<=4096:
                        rows=self.rays.observe(document,[packet['x'],packet['y']],points,packet['source_monotonic'],now)
                        self.sequence+=1
                        for row in rows:row['source_sequence']=self.sequence;row['source_ros_ns']=packet['source_ns']
                        self.consume(document,rows)
                        self.summary={'source':'projected_scan','state':'observing','frames':self.summary['frames']+1,
                                      'source_ns':packet['source_ns'],'processing_ms':(time.perf_counter()-start)*1000,'doors':rows}
                self.refresh(document)
            except Exception as error:
                self.summary={**self.summary,'state':'error','error':type(error).__name__}
    def close(self):self.closed.set();self.event.set();self.thread.join(timeout=2)
