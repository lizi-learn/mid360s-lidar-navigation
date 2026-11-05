"""WATER JSON stream decoder, independent of recv boundaries and line breaks."""
import codecs
import json
import math
import urllib.parse
from .errors import NavigationError

class JSONStream:
    def __init__(self, maximum_bytes=2000000):
        self.decoder=codecs.getincrementaldecoder('utf-8')('strict');self.buffer=''
        self.maximum=maximum_bytes;self.json=json.JSONDecoder()
    def feed(self, data, *, final=False):
        try:self.buffer+=self.decoder.decode(data,final=final)
        except UnicodeError:raise NavigationError('INVALID_RESPONSE','Invalid UTF-8 stream',status=502) from None
        if len(self.buffer.encode())>self.maximum:
            raise NavigationError('RESPONSE_TOO_LARGE','JSON stream bound exceeded',status=502)
        objects=[]
        while self.buffer.strip():
            self.buffer=self.buffer.lstrip()
            if not self.buffer.startswith('{'):
                raise NavigationError('INVALID_RESPONSE','Expected a JSON object',status=502)
            try:obj,end=self.json.raw_decode(self.buffer)
            except ValueError:
                if final:raise NavigationError('INVALID_RESPONSE','Truncated JSON at EOF',status=502) from None
                break
            if not isinstance(obj,dict):raise NavigationError('INVALID_RESPONSE','Expected object',status=502)
            objects.append(obj);self.buffer=self.buffer[end:]
        return objects

def command(path, params=None):
    if not path.startswith('/api/') or '?' in path:raise ValueError('Expected an API path')
    return path+('?' + urllib.parse.urlencode(params, safe=',',quote_via=urllib.parse.quote) if params else '')

def parse_command(line):
    if not line.startswith('/api/') or any(ord(c)<32 for c in line):
        raise NavigationError('INVALID_ARGUMENT','Expected a bare /api/ URI',status=400)
    parsed=urllib.parse.urlsplit(line)
    if parsed.scheme or parsed.netloc or parsed.fragment:raise NavigationError('INVALID_ARGUMENT','Invalid command URI',status=400)
    pairs=urllib.parse.parse_qsl(parsed.query,keep_blank_values=True,max_num_fields=32,encoding='utf-8',errors='strict')
    if len(dict(pairs))!=len(pairs):raise NavigationError('INVALID_ARGUMENT','Repeated parameters are ambiguous',status=400)
    return parsed.path,dict(pairs)

def quaternion(theta):return {'x':0.,'y':0.,'z':math.sin(theta/2),'w':math.cos(theta/2)}
def yaw(q):return math.remainder(2*math.atan2(float(q['z']),float(q['w'])),2*math.pi)
