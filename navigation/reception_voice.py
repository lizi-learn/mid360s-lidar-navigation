"""Optional background TTS cache and local body-speaker presentation playback.

TTS adapter is an administrator-configured argv, never a shell expression. It
receives JSON on stdin with text/output. Arrival playback uses only local WAV.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time
import wave
from audio_feedback import atomic_json
from reception_catalog import content_key, introduction


def checked_clip(path):
    with wave.open(str(path), 'rb') as stream:
        seconds = stream.getnframes()/stream.getframerate()
        if stream.getnchannels() not in (1, 2) or stream.getsampwidth() != 2 or not .1 <= seconds <= 180:
            raise ValueError('讲解音频须为0.1到180秒、16位单声道或双声道WAV')
        return seconds

class VoiceCache:
    def __init__(self, root, command=None):
        self.root = Path(root); self.command = command; self.lock = threading.Lock()
        self.jobs = {}; self.busy = False
    def status(self, document):
        scripts = {content_key(t): t for p in document['points'] if (t := introduction(document, p))}
        with self.lock:
            return {'configured': bool(self.command), 'busy': self.busy,
                    'clips': {k: {'available': (self.root/(k+'.wav')).is_file(), **self.jobs.get(k, {})} for k in scripts}}
    def prepare(self, document):
        if not self.command: raise ValueError('尚未配置讲解语音合成适配器；文字与路线可正常编辑')
        scripts = {content_key(t): t for p in document['points'] if (t := introduction(document, p))}
        with self.lock:
            if self.busy: raise ValueError('上一批讲解音频仍在生成')
            self.busy = True
        def run():
            try:
                self.root.mkdir(parents=True, exist_ok=True)
                for key, script in scripts.items():
                    clip = self.root/(key+'.wav'); temporary = self.root/(key+'.pending.wav')
                    if clip.is_file(): continue
                    try:
                        result = subprocess.run(self.command, input=json.dumps({'text': script, 'output': str(temporary)}).encode(),
                                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=45, check=False)
                        if result.returncode: raise ValueError('合成适配器返回错误')
                        duration = checked_clip(temporary); temporary.replace(clip)
                        with self.lock: self.jobs[key] = {'result': 'ready', 'seconds': duration}
                    except Exception as error:
                        with self.lock: self.jobs[key] = {'result': 'failed', 'error': type(error).__name__}
                        temporary.unlink(missing_ok=True)
            finally:
                with self.lock: self.busy = False
        threading.Thread(target=run, daemon=True, name='reception-voice-cache').start()
        return {'queued': len(scripts)}

class PresentationPlayer:
    def __init__(self, root, result_path, sink=''):
        self.root = Path(root); self.result_path = Path(result_path); self.sink = sink
        self.process = None; self.identity = None; self.handled = None; self.started = 0.
        self.duration = 0.
    def acknowledge(self, identity, result):
        atomic_json(self.result_path, {'epoch': identity[0], 'key': identity[1], 'result': result, 'at': time.time()})
        self.handled = identity
    def stop(self, interrupted=True):
        previous=self.identity
        if self.process:
            self.process.terminate()
            try: self.process.wait(timeout=.3)
            except subprocess.TimeoutExpired: self.process.kill(); self.process.wait(timeout=.3)
        self.process = None; self.identity = None
        if interrupted and previous:self.acknowledge(previous,'failed')
    def observe(self, mission, *, enabled, volume, now, cue_busy=False):
        identity = (mission.get('epoch'), mission.get('speech_key')) if mission.get('phase') == 'presenting' else None
        if identity and (type(identity[0]) is not int or not isinstance(identity[1], str) or len(identity[1]) != 64 or any(c not in '0123456789abcdef' for c in identity[1])):
            identity = None
        if self.process:
            if self.identity != identity or not enabled or cue_busy:
                self.stop(); return
            rc = self.process.poll()
            if rc is not None or now-self.started > self.duration+3:
                previous = self.identity
                result = 'completed' if rc == 0 else 'failed'
                self.stop(interrupted=False); self.acknowledge(previous, result)
        if not identity or identity == self.handled or self.process or cue_busy: return
        if not enabled or not volume:
            self.acknowledge(identity, 'missing'); return
        # Restart never replays a completed introduction for the same mission.
        try:
            previous = json.loads(self.result_path.read_text())
            if (previous.get('epoch'), previous.get('key')) == identity:
                self.handled = identity
                if previous.get('result')=='started':self.acknowledge(identity,'failed')
                return
        except (OSError, ValueError): pass
        clip = self.root/(identity[1]+'.wav')
        try: self.duration = checked_clip(clip)
        except (OSError, ValueError, wave.Error): self.acknowledge(identity, 'missing'); return
        args = ['paplay', '--client-name=P2 reception', '--stream-name=product-introduction', '--volume='+str(round(volume*65536/100))]
        if self.sink: args += ['--device='+self.sink]
        try:
            self.process = subprocess.Popen(args+[str(clip)], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.identity = identity; self.started = now
            atomic_json(self.result_path,{'epoch':identity[0],'key':identity[1],'result':'started','at':time.time()})
        except OSError: self.acknowledge(identity, 'failed')
