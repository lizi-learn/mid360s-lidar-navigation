#!/usr/bin/env python3
"""Observe local navigation status and play local cues on the BODY speaker.

The only navigation HTTP operation is GET /api/status?summary=1. This service
cannot arm, start, stop, replan, reset or command the robot.
"""
import argparse
import fcntl
import json
import logging
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import urllib.parse
import urllib.request
import wave
from audio_feedback import AudioSettings, CuePolicy, Cue, PHRASES, atomic_json


def status_url(value):
    url = urllib.parse.urlsplit(value)
    if url.scheme != 'http' or url.hostname not in {'127.0.0.1', 'localhost', '::1'} or url.username or url.password or url.path not in {'', '/'} or url.query or url.fragment:
        raise ValueError('Audio observer requires a local navigation base URL')
    return value.rstrip('/') + '/api/status?summary=1'


def make_tone(path, kind):
    """Own quiet sine cues with smooth edges, no siren or repeating alarm."""
    if path.exists():
        return path
    frequencies = {'started': (523., 659.), 'arrived': (523., 659., 784.),
                   'obstacle': (440., 392.), 'failed': (392., 349.), 'no_path': (392., 349.),
                   'continued': (523., 659.)}.get(kind, (440.,))
    rate, data = 24000, []
    for freq in frequencies:
        count = int(.16 * rate)
        for i in range(count):
            fade = min(1., i / (.03 * rate), (count - 1 - i) / (.04 * rate))
            data.append(struct.pack('<h', round(.075 * 32767 * math.sin(2 * math.pi * freq * i / rate) * fade)))
        data.extend([b'\0\0'] * int(.05 * rate))
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), 'wb') as output:
        output.setparams((1, 2, rate, 0, 'NONE', 'not compressed'))
        output.writeframes(b''.join(data))
    return path


class LocalPlayer:
    def __init__(self, voice_dir, cache, sink=''):
        self.voice_dir, self.cache, self.sink = Path(voice_dir), Path(cache), sink
        self.process, self.cue, self.started, self.last_result = None, None, 0., None

    def path(self, kind):
        if kind not in PHRASES:
            raise ValueError('Unknown navigation cue')
        clip = self.voice_dir / (kind + '.wav')
        return clip if clip.is_file() else make_tone(self.cache / (kind + '.wav'), kind)

    def play(self, cue, volume, now):
        args = ['paplay', '--client-name=P2 navigation cues', '--stream-name=' + cue.kind,
                '--volume=' + str(round(volume * 65536 / 100))]
        if self.sink:
            args += ['--device=' + self.sink]
        self.process = subprocess.Popen(args + [str(self.path(cue.kind))], stdin=subprocess.DEVNULL,
                                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.cue, self.started = cue, now

    def poll(self, now):
        if not self.process:
            return None
        rc = self.process.poll()
        if rc is None and now - self.started > 8.:
            self.stop('playback_timeout')
            return self.last_result
        if rc is not None:
            self.last_result = {'event': self.cue.kind, 'process_returncode': rc,
                                'result': 'playback_process_completed' if rc == 0 else 'playback_process_failed'}
            self.process, self.cue = None, None
            return self.last_result
        return None

    def stop(self, reason):
        if self.process:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=.3)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=.3)
            self.last_result = {'event': self.cue.kind, 'result': reason}
            self.process, self.cue = None, None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8930')
    parser.add_argument('--settings', required=True)
    parser.add_argument('--voice-dir', required=True)
    parser.add_argument('--sink', default='')
    args = parser.parse_args()
    url = status_url(args.url)
    settings_path = Path(args.settings)
    settings_path.parent.mkdir(parents=True, exist_ok=True)
    lock = settings_path.with_name('audio-feedback.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    settings = AudioSettings(settings_path)
    player = LocalPlayer(args.voice_dir, settings_path.parent / 'audio-tone-cache', args.sink)
    policy, pending, test_generation, stopped = CuePolicy(), None, None, False
    last_event, error, last_report = None, '', 0.
    def stop_signal(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGTERM, stop_signal)
    signal.signal(signal.SIGINT, stop_signal)
    try:
        while not stopped:
            now = time.monotonic()
            player.poll(now)
            try:
                with urllib.request.urlopen(url, timeout=1.) as response:
                    data = json.load(response)
                cfg = settings.read()
                cue = policy.observe(data, now)
                if test_generation is not None and cfg['test_generation'] != test_generation:
                    cue = Cue('test', policy.generation, policy.epoch, now)
                test_generation = cfg['test_generation']
                if not cfg['enabled'] or cfg['volume'] == 0:
                    pending = None
                    player.stop('muted')
                else:
                    if cue:
                        pending = cue
                    # Cancel stale speech as soon as navigation state changes.
                    if player.cue and not policy.valid(player.cue, data, now, max_age=None):
                        player.stop('superseded')
                    if pending and not policy.valid(pending, data, now):
                        pending = None
                    if pending and player.cue and pending.priority > player.cue.priority:
                        player.stop('higher_priority_event')
                    if pending and not player.process:
                        player.play(pending, cfg['volume'], now)
                        last_event = {'kind': pending.kind, 'text': PHRASES[pending.kind],
                                      'generation': pending.generation, 'at': time.time()}
                        pending = None
                error = ''
            except Exception as exc:
                # Audio/transport failure never writes a navigation setting or
                # controls motion. Do not flush old speech on reconnection.
                policy.disconnected()
                pending = None
                player.stop('observer_unavailable')
                error = type(exc).__name__
            if now - last_report >= .5:
                atomic_json(settings_path.with_name('audio-feedback-state.json'),
                            {'pid': os.getpid(), 'updated_at': time.time(), 'last_event': last_event,
                             'playing': player.cue.kind if player.cue else None,
                             'last_result': player.last_result, 'error': error or ('playback_failed' if (player.last_result or {}).get('result') == 'playback_process_failed' else ''),
                             'voice_available': all((Path(args.voice_dir) / (k + '.wav')).is_file() for k in PHRASES),
                             'hardware_control_requests': 0})
                last_report = now
            time.sleep(.2)
    finally:
        player.stop('service_stopped')


if __name__ == '__main__':
    main()
