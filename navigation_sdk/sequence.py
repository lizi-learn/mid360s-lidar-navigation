"""Ordered task execution independent of browser/model polling. Never replay on restart."""
import copy
import threading
import time
from .errors import NavigationError

TERMINAL={'succeeded','failed','cancelled','interrupted','unknown'}

class Sequences:
    def __init__(self,store):
        self.store=store;self.stop_event=threading.Event();self.events={};self.workers=[]
    def active(self):
        for ident,raw in self.store.db.execute('SELECT id,payload FROM tasks').fetchall():
            import json
            t=json.loads(raw)
            if t['request']['kind']=='sequence'and t['state']not in TERMINAL:return t
        return None
    def start(self,task):
        event=threading.Event();self.events[task['id']]=event
        worker=threading.Thread(target=self.run,args=(task['id'],event),daemon=True,name='navigation-sequence')
        self.workers.append(worker);worker.start()
    def cancel_intent(self,task):
        self.events.setdefault(task['id'],threading.Event()).set()
        task['sequence']['cancel_requested']=True;self.store._write(task)
    def stop_intents(self):
        with self.store.lock:
            task=self.active()
            if task:self.cancel_intent(task)
            return task
    def finish_stop(self,task,state='cancelled'):
        if task:
            with self.store.lock:
                current=self.store._load(task['id'])
                if current['state']not in TERMINAL:current.update(state=state,reason='sequence_stop_requested');self.store._write(current)
    def close(self):
        self.stop_event.set()
        for worker in self.workers:worker.join(timeout=25)
    def run(self,ident,event):
        while not self.stop_event.wait(.1):
            with self.store.lock:
                t=self.store._load(ident)
                if t['state']in TERMINAL or event.is_set():return
                if t['sequence'].get('explicit_pause'):continue
                ref=copy.deepcopy(t['native_ref']);index=t['sequence']['index'];revision=t['sequence'].get('revision',0)
            try:observed=self.store.backend.observe(ref)
            except Exception as exc:
                with self.store.lock:
                    t=self.store._load(ident)
                    if t['state']not in TERMINAL:t.update(observation_stale=True,last_observation_error=type(exc).__name__);self.store._write(t)
                continue
            next_request=None
            with self.store.lock:
                t=self.store._load(ident)
                if t['state']in TERMINAL or event.is_set():return
                if t['native_ref']!=ref or t['sequence'].get('revision',0)!=revision:continue
                state=observed.get('state')
                if t['sequence'].get('resume_completed')and observed.get('native_terminal_succeeded')and state=='paused':state='succeeded'
                if state not in {'accepted','running','blocked','paused','presenting','waiting','dwelling',*TERMINAL}:
                    state='unknown'
                t.update(observation=observed,observation_stale=False,observed_at=time.time(),updated_at=time.time())
                if state=='succeeded':
                    t['sequence']['completed']=index+1
                    if len(t['sequence']['receipts'])<=index:t['sequence']['receipts'].append(dict(index=index,native_ref=ref,observation=observed))
                    if index+1==len(t['request']['steps']):t['state']='succeeded';self.store._write(t);return
                    # Persist a pending transition before any next native side effect.
                    next_request=t['request']['steps'][index+1];t['state']='dispatching'
                    t['sequence']['transition_pending']=index+1
                else:t['state']=state
                self.store._write(t)
                if state in TERMINAL and state!='succeeded':return
            if next_request is None:continue
            if event.is_set()or self.stop_event.is_set():return
            if t['sequence'].get('resume_completed'):ref=dict(ref,resume_completed_phase=observed['native_phase'])
            try:
                new_ref=self.store.backend.submit(next_request,after=ref,cancelled=lambda:event.is_set()or self.stop_event.is_set())
                error=None
            except NavigationError as exc:new_ref=None;error=exc
            except Exception:new_ref=None;error=NavigationError('UPSTREAM_UNCERTAIN','Sequence transition outcome unknown; not replayed',status=503)
            if error and error.details.get('native_ref'):new_ref=error.details['native_ref']
            paused_error=error and error.code=='OPERATOR_PAUSED'
            if error and error.code not in('UPSTREAM_UNCERTAIN','INVALID_RESPONSE','BACKEND_PROTOCOL'):
                try:paused_error=self.store.backend.observe(new_ref or ref)['state']=='paused'
                except Exception:pass
            with self.store.lock:
                t=self.store._load(ident)
                if new_ref is not None:
                    t['native_ref']=new_ref;t['sequence']['index']=index+1
                    t['sequence'].pop('transition_pending',None)
                if event.is_set()or self.stop_event.is_set():
                    if t['state']not in TERMINAL:t['state']='cancelled'
                elif error:
                    t.update(state='paused'if paused_error else'unknown'if error.code in('UPSTREAM_UNCERTAIN','INVALID_RESPONSE','BACKEND_PROTOCOL')else'cancelled'if error.code=='SEQUENCE_CANCELLED'else'interrupted'if error.code in('TASK_REPLACED','MANUAL_PRIORITY')else'failed',error=error.to_dict()['error'])
                else:t['state']='accepted'
                if new_ref is not None:t['sequence'].pop('resume_completed',None)
                self.store._write(t)
            if event.is_set()and new_ref is not None:
                # Cancel only our late accepted child, never a newer APP task.
                try:self.store.backend.control(new_ref,'cancel')
                except NavigationError:pass
                return
            if error and not paused_error:return

    def control(self,ident,operation):
        with self.store.lock:
            task=self.store._load(ident)
            if task['state']in TERMINAL:raise NavigationError('TASK_TERMINAL','Sequence is terminal')
            if task['state']=='dispatching':
                if operation!='cancel':raise NavigationError('TRANSITION_PENDING','A step is being dispatched; cancel remains available')
            if operation=='cancel':self.cancel_intent(task)
            elif operation=='pause':task['sequence']['explicit_pause']=True
            elif operation not in('resume','continue'):raise NavigationError('INVALID_ARGUMENT','Unknown sequence control')
            task['sequence']['revision']=task['sequence'].get('revision',0)+1;self.store._write(task)
            ref=task['native_ref'];request=task['request']['steps'][task['sequence']['index']]
        if operation in('cancel','pause'):
            observed=self.store.backend.observe(ref)
            if observed['state']not in TERMINAL:
                try:self.store.backend.control(ref,'cancel')
                except NavigationError as exc:
                    if exc.code!='TASK_TERMINAL':raise
            new_ref=ref
        else:
            if task['state']!='paused':raise NavigationError('INVALID_ARGUMENT','Sequence is not paused')
            event=self.events[ident]
            observed=self.store.backend.observe(ref)
            if observed['state']=='succeeded':new_ref=ref
            elif observed['state']=='paused'and observed.get('native_terminal_succeeded'):
                new_ref=ref;task['sequence']['resume_completed']=True
            else:new_ref=self.store.backend.resume_relative(ref,request,cancelled=event.is_set)
        with self.store.lock:
            task=self.store._load(ident)
            if task['sequence'].get('cancel_requested')and operation!='cancel':return task
            task.update(native_ref=new_ref,state='cancelled'if operation=='cancel'else'paused'if operation=='pause'else'accepted',last_control=operation,updated_at=time.time())
            if operation in('resume','continue'):task['sequence'].pop('explicit_pause',None)
            if operation in('resume','continue')and observed.get('native_terminal_succeeded'):task['sequence']['resume_completed']=True
            self.store._write(task);return task
