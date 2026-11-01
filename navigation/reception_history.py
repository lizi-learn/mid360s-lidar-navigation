"""Private bounded event journal, written outside the ROS/control threads."""
import json
from pathlib import Path
import queue
import os
import sqlite3
import threading
import time

def wire_clocks(value,key=''):
    # Browser Numbers cannot exactly represent ROS nanoseconds (~1e18).
    # Store integer source stamps in SQLite; serialize them losslessly as text.
    if isinstance(value,dict):return {k:wire_clocks(v,k)for k,v in value.items()}
    if isinstance(value,list):return [wire_clocks(v)for v in value]
    return str(value)if key.endswith('_ns')and type(value)is int else value


class History:
    def __init__(self, path):
        self.path = Path(path); self.path.parent.mkdir(parents=True, exist_ok=True)
        self.pending = queue.Queue(maxsize=512); self.dropped = 0; self.error = ''
        self.closed = threading.Event(); self.ready = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True, name='reception-history')
        self.thread.start()

    def append(self, event):
        try: self.pending.put_nowait(dict(event))
        except queue.Full: self.dropped += 1

    def run(self):
        connection = None
        try:
            connection = sqlite3.connect(str(self.path), timeout=2)
            os.chmod(self.path,0o600)
            connection.execute('PRAGMA journal_mode=WAL')
            connection.execute('CREATE TABLE IF NOT EXISTS events (id INTEGER PRIMARY KEY, run TEXT NOT NULL, map_id TEXT NOT NULL, wall REAL NOT NULL, data TEXT NOT NULL)')
            connection.execute('CREATE INDEX IF NOT EXISTS event_map ON events(map_id,id)')
            connection.execute('CREATE TABLE IF NOT EXISTS samples (id INTEGER PRIMARY KEY, run TEXT NOT NULL, map_id TEXT NOT NULL, wall REAL NOT NULL, data TEXT NOT NULL)')
            connection.execute('CREATE INDEX IF NOT EXISTS sample_run ON samples(map_id,run,id)')
            connection.commit(); self.ready.set()
            while not self.closed.is_set() or not self.pending.empty():
                try: event = self.pending.get(timeout=.1)
                except queue.Empty: continue
                try:
                    table='samples'if event.get('kind')=='telemetry'else 'events'
                    cursor=connection.execute('INSERT INTO '+table+'(run,map_id,wall,data) VALUES(?,?,?,?)',
                        (event['run_id'], event['map_id'], event['wall_time'], json.dumps(event, ensure_ascii=False, allow_nan=False)))
                    maximum=25000 if table=='samples'else 20000
                    connection.execute('DELETE FROM '+table+' WHERE id <= ?',(cursor.lastrowid-maximum,))
                    connection.commit(); self.error = ''
                except (ValueError, sqlite3.Error) as error: self.error = type(error).__name__
                finally: self.pending.task_done()
        except sqlite3.Error as error:
            self.error = type(error).__name__; self.ready.set()
        finally:
            if connection: connection.close()

    def read(self, map_id, before=None, limit=100):
        if type(limit) is not int or not 1 <= limit <= 200: raise ValueError('历史条数须在1到200之间')
        if before is not None and (type(before) is not int or before < 1): raise ValueError('历史游标无效')
        if not self.ready.wait(timeout=2) or self.error: raise ValueError('任务历史暂不可读：'+(self.error or 'initializing'))
        connection = sqlite3.connect(str(self.path), timeout=2)
        try:
            rows = connection.execute('SELECT id,data FROM events WHERE map_id=? AND id<? ORDER BY id DESC LIMIT ?',
                (map_id, before or 9223372036854775807, limit)).fetchall()
        finally: connection.close()
        return {'events': [dict(json.loads(data), cursor=i) for i, data in rows],
                'before': rows[-1][0] if len(rows) == limit else None,
                'pending': self.pending.qsize(), 'dropped': self.dropped, 'error': self.error,
                'retention_events': 20000}

    def close(self):
        self.closed.set(); self.thread.join(timeout=3)

    def runs(self,map_id):
        if not self.ready.wait(2):raise ValueError('记录正在初始化')
        with sqlite3.connect(str(self.path),timeout=2)as c:
            rows=c.execute('SELECT run,MIN(wall),MAX(wall),COUNT(*) FROM samples WHERE map_id=? GROUP BY run ORDER BY MAX(wall) DESC LIMIT 50',(map_id,)).fetchall()
        return {'runs':[dict(run_id=r,start=a,end=b,samples=n)for r,a,b,n in rows],
                'sample_rate_hz':5,'retention_samples':25000,'dropped':self.dropped,'error':self.error}

    def trace(self,map_id,run_id,after=0,limit=200):
        if not isinstance(run_id,str)or not 1<=len(run_id)<=80 or type(after)is not int or after<0 or type(limit)is not int or not 1<=limit<=500:
            raise ValueError('轨迹查询参数无效')
        if not self.ready.wait(2):raise ValueError('记录正在初始化')
        with sqlite3.connect(str(self.path),timeout=2)as c:
            rows=c.execute('SELECT id,data FROM samples WHERE map_id=? AND run=? AND id>? ORDER BY id LIMIT ?',(map_id,run_id,after,limit)).fetchall()
            events=c.execute('SELECT data FROM events WHERE map_id=? AND run=? ORDER BY id',(map_id,run_id)).fetchall()
        return {'samples':[dict(wire_clocks(json.loads(d)),cursor=i)for i,d in rows],
                'after':rows[-1][0]if len(rows)==limit else None,'events':[json.loads(d)for d, in events],
                'clock':'wall_time+capture_monotonic_ns; original source clocks retained separately',
                'scope':'5Hz sampled actual feedback; not every control packet or physical ground truth'}
