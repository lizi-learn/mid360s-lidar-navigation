"""Private, map-scoped reception content. No ROS, network, or actuator imports."""
import copy
import hashlib
import json
import math
import re
import threading
from pathlib import Path
from audio_feedback import atomic_json

ID = re.compile(r'[A-Za-z0-9_-]{1,64}\Z')
MAX_BYTES = 512 * 1024

def ident(value):
    if not isinstance(value, str) or not ID.fullmatch(value):
        raise ValueError('标识只能包含字母、数字、短横线和下划线')
    return value

def number(value, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'数值范围应为 {low} 到 {high}')
    return float(value)

def text(value, limit=120):
    if not isinstance(value, str) or len(value) > limit or '\x00' in value:
        raise ValueError('文字内容无效或过长')
    return value.strip()

def blank(map_id):
    return {'schema': 1, 'map_id': ident(map_id), 'revision': 0, 'points': [],
            'products': [], 'routes': [], 'doors': [], 'edges': [],
            'policy': {'brief_wait': 4., 'max_wait': 25., 'clear_hold': 1.5,
                       'replan_interval': 3., 'shortcut_gain': .2}}

def validate(raw):
    if not isinstance(raw, dict) or set(raw) - set(blank('map')):
        raise ValueError('内容类型或字段无效')
    d = copy.deepcopy(raw)
    if d.get('schema') != 1 or type(d.get('revision')) is not int or d['revision'] < 0:
        raise ValueError('内容版本无效')
    ident(d.get('map_id'))
    collections = {'points': 1000, 'products': 1000, 'routes': 100, 'doors': 200, 'edges': 2000}
    ids = {}
    for key, maximum in collections.items():
        rows = d.get(key)
        if not isinstance(rows, list) or len(rows) > maximum:
            raise ValueError('内容数量超过范围')
        ids[key] = set()
        for row in rows:
            if not isinstance(row, dict): raise ValueError('内容项无效')
            i = ident(row.get('id'))
            if i in ids[key]: raise ValueError('同类标识重复')
            ids[key].add(i)
    def fields(row, allowed):
        if set(row) - set(allowed.split()): raise ValueError('存在未支持的内容字段')
    for p in d['products']:
        fields(p, 'id name script tags')
        p['name'] = text(p.get('name', ''), 80)
        p['script'] = text(p.get('script', ''), 3000)
        p['tags'] = text(p.get('tags', ''), 200)
    for p in d['points']:
        fields(p, 'id name x y yaw product_id script dwell guest_wait kind')
        p['name'] = text(p.get('name', ''), 80)
        if not p['name']: raise ValueError('点位名称不能为空')
        for k in ('x', 'y'): p[k] = number(p.get(k), -100000., 100000.)
        yaw = p.get('yaw')
        p['yaw'] = None if yaw is None else number(yaw, -math.pi, math.pi)
        p['product_id'] = p.get('product_id') or ''
        if p['product_id'] and p['product_id'] not in ids['products']: raise ValueError('点位引用的产品不存在')
        p['script'] = text(p.get('script', ''), 3000)
        p['dwell'] = number(p.get('dwell', 0), 0, 600)
        if type(p.get('guest_wait', False)) is not bool: raise ValueError('等候访客设置无效')
        p['guest_wait'] = p.get('guest_wait', False)
        p['kind'] = p.get('kind', 'visit')
        if p['kind'] not in ('visit', 'transit', 'home'): raise ValueError('点位类型无效')
    for r in d['routes']:
        fields(r, 'id name mode steps loops description')
        r['name'] = text(r.get('name', ''), 80)
        r['description'] = text(r.get('description', ''), 300)
        r['mode'] = r.get('mode', 'showcase')
        if r['mode'] not in ('showcase', 'adaptive'): raise ValueError('路线模式无效')
        if type(r.get('loops', 1)) is not int or not 1 <= r.get('loops', 1) <= 10: raise ValueError('循环次数应为1到10')
        r['loops'] = r.get('loops', 1)
        if not isinstance(r.get('steps'), list) or not 1 <= len(r['steps']) <= 100: raise ValueError('路线需包含1到100个点位')
        for s in r['steps']:
            fields(s, 'point_id speak')
            if s.get('point_id') not in ids['points']: raise ValueError('路线引用的点位不存在')
            if type(s.get('speak', True)) is not bool: raise ValueError('讲解开关无效')
            s['speak'] = s.get('speak', True)
    for door in d['doors']:
        fields(door, 'id name x y width yaw')
        door['name'] = text(door.get('name', ''), 80)
        for k in ('x', 'y'): door[k] = number(door.get(k), -100000., 100000.)
        door['width'] = number(door.get('width', 1.), .1, 10)
        door['yaw'] = number(door.get('yaw', 0), -math.pi, math.pi)
    for e in d['edges']:
        fields(e, 'id a b bidirectional door_id penalty')
        if e.get('a') not in ids['points'] or e.get('b') not in ids['points'] or e['a'] == e['b']: raise ValueError('通道两端点位无效')
        e['door_id'] = e.get('door_id') or ''
        if e['door_id'] and e['door_id'] not in ids['doors']: raise ValueError('通道引用的门不存在')
        if type(e.get('bidirectional', True)) is not bool: raise ValueError('通道方向无效')
        e['bidirectional'] = e.get('bidirectional', True)
        e['penalty'] = number(e.get('penalty', 0), 0, 10000)
    policy = d.get('policy')
    if not isinstance(policy, dict) or set(policy) != set(blank('map')['policy']): raise ValueError('等待与绕行参数字段无效')
    for k in ('brief_wait', 'max_wait', 'clear_hold', 'replan_interval'):
        policy[k] = number(policy[k], .1, 600)
    if policy['brief_wait'] > policy['max_wait']: raise ValueError('短暂等候不能超过最长等候')
    policy['shortcut_gain'] = number(policy['shortcut_gain'], .01, .9)
    if len(json.dumps(d, ensure_ascii=False).encode()) > MAX_BYTES: raise ValueError('内容包过大')
    return d

class Catalog:
    def __init__(self, root):
        self.root = Path(root); self.lock = threading.RLock(); self.cache = {}
    def path(self, map_id): return self.root / (ident(map_id) + '.json')
    def _load(self,map_id):
        path=self.path(map_id)
        try:stat=path.stat()
        except FileNotFoundError:return blank(map_id)
        identity=(stat.st_mtime_ns,stat.st_size)
        if stat.st_size>MAX_BYTES:raise ValueError('内容包过大')
        cached=self.cache.get(map_id)
        if cached and cached[0]==identity:return cached[1]
        document=validate(json.loads(path.read_text()))
        if document['map_id']!=map_id:raise ValueError('内容包地图标识与所在文件不一致')
        self.cache[map_id]=(identity,document)
        return document
    def read(self, map_id):
        with self.lock:return copy.deepcopy(self._load(map_id))
    def summary(self,map_id):
        with self.lock:
            document=self._load(map_id)
            return {'map_id':map_id,'revision':document['revision'],'doors':copy.deepcopy(document['doors'])}
    def save(self, document, expected_revision):
        d = validate(document)
        if type(expected_revision) is not int: raise ValueError('修订号无效')
        with self.lock:
            old = self.read(d['map_id'])
            if old['revision'] != expected_revision: raise ValueError('内容已被另一窗口修改，请刷新后合并')
            d['revision'] = old['revision'] + 1
            if old['revision']:
                atomic_json(self.root/'history'/d['map_id']/f"{old['revision']}.json", old)
            atomic_json(self.path(d['map_id']), d)
            # Bound private revision storage; retain the most recent 30.
            versions = sorted((self.root/'history'/d['map_id']).glob('*.json'), key=lambda p: int(p.stem))
            for p in versions[:-30]: p.unlink()
            return d

def introduction(document, point):
    product = next((p for p in document['products'] if p['id'] == point['product_id']), None)
    return point['script'] or (product['script'] if product else '')

def content_key(script):
    return hashlib.sha256(script.encode()).hexdigest()
