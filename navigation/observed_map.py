"""Persist measured changes to old occupied cells, separately from transient people.

Only finite, original-time projected scan rays establish free-space evidence.
Unknown cells and authored restrictions are never cleared. The base raster is
immutable; this small, map-fingerprinted layer participates in navigation,
display, map versions and site backup. It is not full-height 3D clearance.
"""
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import numpy as np
from audio_feedback import atomic_json

FILENAME = 'observed-map.json'


def signature(grid):
    geometry = [int(grid['width']), int(grid['height']),
                round(float(grid['resolution']), 7),
                *[round(float(v), 6) for v in grid['origin'][:2]]]
    values = np.asarray(grid['cells'], dtype=np.int8)
    if (not 0 < geometry[0]*geometry[1] <= 16_000_000 or
            len(values) != geometry[0]*geometry[1] or
            not all(math.isfinite(v) for v in geometry) or geometry[2] <= 0):
        raise ValueError('Invalid observation map geometry')
    return hashlib.sha256(json.dumps(geometry).encode()+values.tobytes()).hexdigest()


def read_layer(path, grid):
    """A changed/reloaded base cannot inherit observations from another map."""
    path = Path(path)
    if not path.exists():
        return {'enabled': True, 'entries': []}
    if path.stat().st_size > 4*1024*1024:
        raise ValueError('Observation layer exceeds 4MiB')
    data = json.loads(path.read_text())
    if not isinstance(data, dict) or data.get('schema') != 1 or type(data.get('enabled')) is not bool:
        raise ValueError('Invalid observation layer schema')
    if data.get('base_sha256') != signature(grid):
        return {'enabled': data['enabled'], 'entries': []}
    entries = data.get('entries')
    if not isinstance(entries, list) or len(entries) > len(grid['cells']):
        raise ValueError('Invalid observation layer entries')
    seen = set()
    for item in entries:
        if not isinstance(item, dict):
            raise ValueError('Invalid measured map cell')
        i = item.get('index')
        if (type(i) is not int or not 0 <= i < len(grid['cells']) or i in seen or
                grid['cells'][i] < 65 or
                any(type(item.get(k)) is not int or item[k] <= 0
                    for k in ('first_source_ns', 'last_source_ns', 'observations')) or
                item['last_source_ns'] < item['first_source_ns']):
            raise ValueError('Invalid measured map cell')
        seen.add(i)
    return data


def saved_cells(path, grid):
    data = read_layer(path, grid)
    cells = list(grid['cells'])
    for item in data['entries']:
        cells[item['index']] = 0
    return cells, {'cleared_cells': len(data['entries']), 'enabled': data['enabled'],
                   'base_raster_modified': False}


class ObservedMap:
    # These are evidence accumulation parameters, not motion permission gates.
    def __init__(self, minimum_frames=8, minimum_span=2., maximum_gap=1.):
        self.minimum_frames = minimum_frames
        self.minimum_span = minimum_span
        self.maximum_gap = maximum_gap
        self.lock = threading.RLock()
        self.key = None
        self.revision = 0
        self.entries = {}
        self.enabled = True
        self.dirty = False
        self.last_flush = 0.
        self.frames = 0
        self.last_source_ns = 0
        self.processing_ms = 0.
        self.error = None
        self.path = None

    def bind(self, grid, directory, force=False):
        key = signature(grid)
        with self.lock:
            if self.key == key and self.path == Path(directory)/FILENAME and not force:
                return
            self.flush(force=True)
            self.grid = dict(grid, cells=np.asarray(grid['cells'], dtype=np.int8).copy())
            self.key = key
            self.path = Path(directory)/FILENAME
            self.indices = np.flatnonzero(self.grid['cells'] >= 65)
            w = grid['width']; r = grid['resolution']; ox, oy = grid['origin'][:2]
            self.centers = np.column_stack((ox+(self.indices % w+.5)*r,
                                           oy+(self.indices // w+.5)*r))
            self.first = np.zeros(len(self.indices))
            self.last = np.zeros(len(self.indices))
            self.counts = np.zeros(len(self.indices), dtype=np.int32)
            self.first_ns = np.zeros(len(self.indices), dtype=np.int64)
            self.last_source_ns = 0
            self.frames = 0
            self.error = None
            try:
                data = read_layer(self.path, self.grid)
                self.enabled = data['enabled']
                self.entries = {v['index']: dict(v) for v in data['entries']}
            except (ValueError, OSError, KeyError, TypeError) as error:
                # Corrupt learned data cannot silently become free space.
                self.enabled = False
                self.entries = {}
                self.error = type(error).__name__
            self.dirty = False
            self.revision += 1

    def apply(self, grid):
        with self.lock:
            cells = list(grid['cells'])
            if (self.key is not None and grid.get('revision') == self.grid.get('revision') and
                    grid['width'] == self.grid['width'] and grid['height'] == self.grid['height']):
                for i in self.entries:
                    if i < len(cells) and cells[i] >= 65:
                        cells[i] = 0
            return cells

    def observe(self, packet, now, localized=True):
        started = time.perf_counter()
        with self.lock:
            stamp = packet.get('source_monotonic', math.nan)
            source_ns = packet.get('source_ns', 0)
            if (self.key is None or not self.enabled or not localized or
                    packet.get('map_revision', self.grid.get('revision')) != self.grid.get('revision') or
                    not math.isfinite(stamp) or not -.05 <= now-stamp <= .5 or
                    type(source_ns) is not int or source_ns <= self.last_source_ns):
                if not localized:
                    self.counts[:] = 0
                return False
            ranges = np.asarray(packet['ranges'], dtype=float)
            inc = packet['angle_increment']
            if (ranges.ndim != 1 or not 0 < len(ranges) <= 4096 or
                    not math.isfinite(inc) or inc <= 0 or
                    not all(math.isfinite(packet[k]) for k in ('x', 'y', 'yaw', 'angle_min', 'range_min', 'range_max'))):
                return False
            self.last_source_ns = source_ns
            self.frames += 1
            delta = self.centers-[packet['x'], packet['y']]
            distance = np.linalg.norm(delta, axis=1)
            angle = (np.arctan2(delta[:, 1], delta[:, 0])-packet['yaw']-packet['angle_min']) % (2*math.pi)
            nearest = np.rint(angle/inc).astype(int)
            if abs(len(ranges)*inc-2*math.pi) < 1e-5:
                nearest %= len(ranges)
            covered = (nearest >= 0) & (nearest < len(ranges))
            bins = np.clip(nearest, 0, len(ranges)-1)
            values = ranges[bins]
            valid = (covered & np.isfinite(values) & (values >= packet['range_min']) &
                     (values <= min(packet['range_max'], 12.)))
            beam_angle = packet['yaw']+packet['angle_min']+bins*inc
            along = delta[:, 0]*np.cos(beam_angle)+delta[:, 1]*np.sin(beam_angle)
            across = np.abs(-delta[:, 0]*np.sin(beam_angle)+delta[:, 1]*np.cos(beam_angle))
            # A thin finite ray must cross the cell's interior, with a return
            # measured at least25cm beyond it. Infinity/empty bins do nothing.
            free = valid & (along > .35) & (across < self.grid['resolution']*.4) & (values > along+.25)

            # All finite endpoints veto nearby clearing, including endpoints in
            # neighboring angular bins. A returning person restores old static
            # occupancy immediately; new people on base-free cells stay dynamic.
            finite = np.isfinite(ranges) & (ranges >= packet['range_min']) & (ranges <= packet['range_max'])
            aa = packet['yaw']+packet['angle_min']+np.flatnonzero(finite)*inc
            ends = np.column_stack((packet['x']+ranges[finite]*np.cos(aa), packet['y']+ranges[finite]*np.sin(aa)))
            w, h, res = self.grid['width'], self.grid['height'], self.grid['resolution']
            ec = np.floor((ends-np.asarray(self.grid['origin'][:2]))/res).astype(int)
            hit_grid = np.zeros(w*h, dtype=bool)
            margin = math.ceil(.1/res)
            for dx in range(-margin, margin+1):
                for dy in range(-margin, margin+1):
                    xx, yy = ec[:, 0]+dx, ec[:, 1]+dy
                    ok = (xx >= 0) & (xx < w) & (yy >= 0) & (yy < h)
                    hit_grid[yy[ok]*w+xx[ok]] = True
            hit = hit_grid[self.indices]
            free &= ~hit
            changed = False
            for i in self.indices[hit]:
                if self.entries.pop(int(i), None) is not None:
                    changed = True
            reset = hit | (stamp-self.last > self.maximum_gap)
            self.counts[reset] = 0
            beginning = free & (self.counts == 0)
            self.first[beginning] = stamp
            self.first_ns[beginning] = source_ns
            self.counts[free] += 1
            self.last[free] = stamp
            proven = free & (self.counts >= self.minimum_frames) & (stamp-self.first >= self.minimum_span)
            for k in np.flatnonzero(proven):
                i = int(self.indices[k])
                if i not in self.entries:
                    self.entries[i] = dict(index=i, first_source_ns=int(self.first_ns[k]),
                                           last_source_ns=source_ns, observations=int(self.counts[k]))
                    changed = True
            if changed:
                self.revision += 1
                self.dirty = True
            self.processing_ms = (time.perf_counter()-started)*1000
            return changed

    def set_enabled(self, value):
        if type(value) is not bool:
            raise ValueError('自动更新开关必须是布尔值')
        with self.lock:
            self.enabled = value
            self.counts[:] = 0
            self.dirty = True
            self.flush(force=True)

    def unbind(self):
        with self.lock:
            self.flush(force=True)
            self.key = None
            self.path = None
            self.entries = {}
            self.dirty = False
            self.revision += 1

    def reset(self):
        with self.lock:
            self.entries = {}
            self.counts[:] = 0
            self.revision += 1
            self.dirty = True
            self.flush(force=True)

    def flush(self, force=False):
        with self.lock:
            if self.path is None or not self.dirty or (not force and time.monotonic()-self.last_flush < 5.):
                return
            atomic_json(self.path, dict(schema=1, base_sha256=self.key, enabled=self.enabled,
                                       entries=sorted(self.entries.values(), key=lambda v: v['index'])))
            self.last_flush = time.monotonic()
            self.dirty = False

    def status(self):
        with self.lock:
            return dict(enabled=self.enabled, cleared_cells=len(self.entries), revision=self.revision,
                        frames=self.frames, source_ns=self.last_source_ns, processing_ms=self.processing_ms,
                        minimum_frames=self.minimum_frames, minimum_span_s=self.minimum_span,
                        error=self.error, base_raster_modified=False,
                        scope='repeated finite projected rays; transient returns are not saved as static obstacles')


class ObservationWorker:
    """One latest slot; bounded background processing, no control-loop disk I/O."""
    def __init__(self, model, healthy, refresh):
        self.model, self.healthy, self.refresh = model, healthy, refresh
        self.lock = threading.Lock()
        self.packet = None
        self.event = threading.Event()
        self.closed = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True, name='observed-map-rays')
        self.thread.start()

    def submit(self, packet):
        with self.lock:
            self.packet = packet
        self.event.set()

    def run(self):
        last = -math.inf
        while not self.closed.is_set():
            self.event.wait(.3)
            self.event.clear()
            if time.monotonic()-last < .25:
                self.closed.wait(.25-(time.monotonic()-last))
            with self.lock:
                packet, self.packet = self.packet, None
            try:
                last = time.monotonic()
                if packet and self.model.observe(packet, last, self.healthy()):
                    self.refresh()
                self.model.flush()
            except Exception as error:
                self.model.error = type(error).__name__

    def close(self):
        self.closed.set()
        self.event.set()
        self.thread.join(timeout=2)
        try:
            self.model.flush(force=True)
        except (OSError, ValueError) as error:
            # A map disk error must not prevent Console's remaining shutdown
            # sequence from cancelling navigation and stopping its processes.
            self.model.error = type(error).__name__
