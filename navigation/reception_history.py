"""Private bounded event journal, written outside the ROS/control threads."""
import json
from pathlib import Path
import queue
import os
import sqlite3
import threading
import time


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
            connection.commit(); self.ready.set()
            while not self.closed.is_set() or not self.pending.empty():
                try: event = self.pending.get(timeout=.1)
                except queue.Empty: continue
                try:
                    connection.execute('INSERT INTO events(run,map_id,wall,data) VALUES(?,?,?,?)',
                        (event['run_id'], event['map_id'], event['wall_time'], json.dumps(event, ensure_ascii=False, allow_nan=False)))
                    connection.execute('DELETE FROM events WHERE id <= COALESCE((SELECT id FROM events ORDER BY id DESC LIMIT 1 OFFSET 20000),-1)')
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
