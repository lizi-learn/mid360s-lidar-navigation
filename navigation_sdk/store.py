"""Durable dispatch journal. A lost reply never causes an automatic replay."""
import hashlib
import json
import sqlite3
import threading
import time
import uuid
from .errors import NavigationError
from .validation import task_request, text

TERMINAL = {'succeeded','failed','cancelled','interrupted','unknown'}
class TaskStore:
    def __init__(self, backend, path=':memory:'):
        self.backend=backend;self.lock=threading.RLock()
        self.db=sqlite3.connect(str(path),check_same_thread=False)
        self.db.execute('CREATE TABLE IF NOT EXISTS tasks (id TEXT PRIMARY KEY, key TEXT UNIQUE, fingerprint TEXT, payload TEXT NOT NULL)')
        # Do not resurrect or replay navigation after a gateway process restart.
        for ident,raw in self.db.execute('SELECT id,payload FROM tasks').fetchall():
            task=json.loads(raw)
            if task['state'] not in TERMINAL:
                task.update(state='unknown',reason='gateway_restarted_reconcile_native_state_before_new_dispatch',updated_at=time.time())
                self._write(task)
        self.db.commit()
        from .sequence import Sequences
        self.sequences=Sequences(self)
    def close(self):self.sequences.close();self.db.close()
    def _write(self, task):
        self.db.execute('UPDATE tasks SET payload=? WHERE id=?',(json.dumps(task,ensure_ascii=False,allow_nan=False),task['id']))
        self.db.commit()
    def _load(self, ident):
        row=self.db.execute('SELECT payload FROM tasks WHERE id=?',(ident,)).fetchone()
        if row is None:raise NavigationError('TASK_NOT_FOUND','Unknown task',status=404)
        return json.loads(row[0])
    def submit(self, body, key=None):
        body=task_request(body)
        if key is not None:key=text(key,'Idempotency-Key',128)
        fingerprint=hashlib.sha256(json.dumps(body,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
        with self.lock:
            if key is not None:
                row=self.db.execute('SELECT fingerprint,payload FROM tasks WHERE key=?',(key,)).fetchone()
                if row:
                    if row[0]!=fingerprint:raise NavigationError('IDEMPOTENCY_CONFLICT','Same key used for a different request')
                    return json.loads(row[1])
            # Providers expose one current native run. Capture its terminal
            # receipt before the next dispatch replaces that run's snapshot.
            # This observes old tasks only; it never replays any request.
            for ident,raw in self.db.execute('SELECT id,payload FROM tasks').fetchall():
                previous=json.loads(raw)
                if previous['state']not in TERMINAL and previous.get('native_ref'):self.get(ident)
            active=self.sequences.active()
            if active:raise NavigationError('TASK_BUSY','An ordered motion task is active',details={'task_id':active['id']})
            if body['kind']=='sequence'and not self.backend.capabilities()['features'].get('motion_sequence'):
                raise NavigationError('UNSUPPORTED_CAPABILITY','Backend has no conditional ordered-motion execution')
            now=time.time();task={'id':uuid.uuid4().hex,'state':'dispatching','request':body,
                                  'created_at':now,'updated_at':now,'backend':self.backend.name,'native_ref':None}
            self.db.execute('INSERT INTO tasks VALUES(?,?,?,?)',(task['id'],key,fingerprint,json.dumps(task)));self.db.commit()
            try:
                task['native_ref']=self.backend.submit(body['steps'][0]if body['kind']=='sequence'else body)
                task['state']='accepted'
            except NavigationError as exc:
                task.update(state='unknown' if exc.code in ('UPSTREAM_UNCERTAIN','INVALID_RESPONSE','BACKEND_PROTOCOL') else 'failed',error=exc.to_dict()['error'])
                task['updated_at']=time.time();self._write(task)
                exc.details={**exc.details,'task_id':task['id']};raise
            except Exception:
                task.update(state='unknown',reason='dispatch_outcome_unknown');self._write(task)
                raise NavigationError('UPSTREAM_UNCERTAIN','Dispatch outcome unknown; request was not replayed',status=503,details={'task_id':task['id']}) from None
            if body['kind']=='sequence':task['sequence']=dict(index=0,total=len(body['steps']),completed=0,receipts=[],revision=0)
            task['updated_at']=time.time();self._write(task)
            if body['kind']=='sequence':self.sequences.start(task)
            return task
    def get(self, ident):
        with self.lock:
            task=self._load(ident)
            if task['request']['kind']=='sequence':return task
            if task['state'] not in TERMINAL and task['native_ref']:
                try:
                    observation=self.backend.observe(task['native_ref'])
                    if observation.get('state') not in {'accepted','running','blocked','paused','presenting','waiting','dwelling',*TERMINAL}:
                        raise NavigationError('BACKEND_PROTOCOL','Unknown backend task state',status=502)
                    task.update(state=observation['state'],observation=observation,observation_stale=False,observed_at=time.time())
                    task.pop('last_observation_error',None)
                except NavigationError as exc:
                    task.update(observation_stale=True,last_observation_error=exc.to_dict()['error'])
                task['updated_at']=time.time();self._write(task)
            return task
    def control(self, ident, operation):
        with self.lock:sequence=self._load(ident)['request']['kind']=='sequence'
        if sequence:return self.sequences.control(ident,operation)
        with self.lock:
            task=self.get(ident)
            if task['state'] in TERMINAL:
                if operation=='cancel' and task['state']=='cancelled':return task
                raise NavigationError('TASK_TERMINAL','Task is terminal; a new task is never cancelled in its place')
            task['native_ref']=self.backend.control(task['native_ref'],operation)
            task['last_control']=operation;task['updated_at']=time.time();self._write(task)
            return self.get(ident)
    def stop(self):
        task=self.sequences.stop_intents()
        try:receipt=self.backend.stop()
        except Exception:
            self.sequences.finish_stop(task,'unknown');raise
        self.sequences.finish_stop(task);return receipt
    def status(self):
        status=self.backend.status()
        with self.lock:task=self.sequences.active()
        if task:status.update(task_active=True,navigation_state=task['state'],current_task=dict(type='sequence',task_id=task['id'],**task['sequence']))
        return status
