"""Run an isolated HTTP SDK companion and optional WATER LF bridge."""
import argparse
import hmac
import ipaddress
import json
import os
from pathlib import Path
import signal
import threading
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from .errors import NavigationError
from .store import TaskStore

class GatewayServer(ThreadingHTTPServer):
    daemon_threads=True
    def __init__(self,address,store,*,api_key=None):
        self.store=store;self.api_key=api_key;super().__init__(address,Handler)

class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass  # Never log tokens, request parameters or scripts.
    def send_json(self, body, status=200):
        raw=json.dumps(body,ensure_ascii=False,allow_nan=False).encode()
        self.send_response(status);self.send_header('Content-Type','application/json; charset=utf-8')
        self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff')
        self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
    def body(self):
        if self.headers.get_content_type()!='application/json':
            raise NavigationError('INVALID_ARGUMENT','Content-Type application/json required',status=415)
        try:length=int(self.headers.get('Content-Length','0'))
        except ValueError:raise NavigationError('INVALID_ARGUMENT','Invalid Content-Length',status=400) from None
        if not 0<length<=65536:raise NavigationError('INVALID_ARGUMENT','JSON body size must be 1..65536 bytes',status=413)
        try:return json.loads(self.rfile.read(length),parse_constant=lambda _: (_ for _ in ()).throw(ValueError()))
        except (ValueError,UnicodeError):raise NavigationError('INVALID_ARGUMENT','Invalid finite JSON',status=400) from None
    def dispatch(self):
        try:
            if self.server.api_key and not hmac.compare_digest(self.headers.get('Authorization',''),'Bearer '+self.server.api_key):
                raise NavigationError('UNAUTHORIZED','Bearer credential required',status=401)
            if self.headers.get('Origin') and urllib.parse.urlsplit(self.headers['Origin']).netloc!=self.headers.get('Host'):
                raise NavigationError('ORIGIN_REJECTED','Use the server-side SDK or a same-origin application',status=403)
            path=urllib.parse.urlsplit(self.path).path;parts=[urllib.parse.unquote(x)for x in path.split('/')if x]
            store=self.server.store;backend=store.backend;method=self.command
            gets={'/v1/capabilities':backend.capabilities,'/v1/status':store.status,'/v1/maps':backend.maps,
                  '/v1/markers':backend.markers,'/v1/routes':backend.routes}
            if method=='GET' and path in gets:return self.send_json(gets[path]())
            if method=='POST' and path=='/v1/tasks':return self.send_json(store.submit(self.body(),self.headers.get('Idempotency-Key')),202)
            if method=='POST' and path=='/v1/stop':
                if self.body()!={}:raise NavigationError('INVALID_ARGUMENT','stop expects {}',status=400)
                return self.send_json(store.stop())
            if method=='POST' and path=='/v1/markers':return self.send_json(backend.save_marker(self.body()))
            if method=='DELETE' and len(parts)==3 and parts[:2]==['v1','markers']:
                return self.send_json(backend.delete_marker(parts[2]))
            if len(parts)in(3,4) and parts[:2]==['v1','tasks']:
                if method=='GET' and len(parts)==3:return self.send_json(store.get(parts[2]))
                if method=='POST' and len(parts)==4 and parts[3]in('cancel','pause','resume','continue'):
                    if self.body()!={}:raise NavigationError('INVALID_ARGUMENT','Control expects {}',status=400)
                    return self.send_json(store.control(parts[2],parts[3]))
            if method=='POST' and path=='/mock/advance' and backend.name=='mock':
                data=self.body()
                if not isinstance(data,dict)or set(data)!={'state'}:raise NavigationError('INVALID_ARGUMENT','Expected state',status=400)
                return self.send_json(backend.advance(data['state']))
            raise NavigationError('NOT_FOUND','Unknown API path',status=404)
        except NavigationError as exc:self.send_json(exc.to_dict(),exc.status)
        except (KeyError,TypeError,ValueError):self.send_json(NavigationError('INVALID_ARGUMENT','Invalid request or backend payload',status=400).to_dict(),400)
        except (BrokenPipeError,ConnectionResetError):pass
        except Exception:self.send_json(NavigationError('INTERNAL_ERROR','Unexpected gateway error; no automatic replay',status=500).to_dict(),500)
    do_GET=dispatch;do_POST=dispatch;do_DELETE=dispatch

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend',choices=('mock','p2','water'),default='mock')
    parser.add_argument('--bind',default='127.0.0.1');parser.add_argument('--port',type=int,default=18940)
    parser.add_argument('--p2-url',default='http://127.0.0.1:8930')
    parser.add_argument('--water-host');parser.add_argument('--water-port',type=int,default=31001)
    parser.add_argument('--water-terminator',choices=('bare','lf','crlf'),default='bare')
    parser.add_argument('--water-bridge-port',type=int,help='Enable explicit LF/CRLF request profile on loopback')
    parser.add_argument('--state-dir',type=Path,default=Path('.sdk-state'));args=parser.parse_args()
    key=os.environ.get('NAVBRIDGE_API_KEY')
    if args.bind not in ('localhost',) and not ipaddress.ip_address(args.bind).is_loopback and not key:
        parser.error('Remote HTTP bind needs NAVBRIDGE_API_KEY; use a private tunnel for transport encryption')
    if args.backend=='mock':
        from .mock import MockBackend
        backend=MockBackend()
    elif args.backend=='p2':
        from .p2 import P2Backend
        backend=P2Backend(args.p2_url)
    else:
        if not args.water_host:parser.error('--water-host is required')
        from .water import WaterBackend
        backend=WaterBackend(args.water_host,args.water_port,terminator={'bare':'','lf':'\n','crlf':'\r\n'}[args.water_terminator])
    args.state_dir.mkdir(mode=0o700,parents=True,exist_ok=True)
    import fcntl
    lock=(args.state_dir/'gateway.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:parser.error('Another gateway owns this state directory')
    store=TaskStore(backend,args.state_dir/'tasks.sqlite3');server=GatewayServer((args.bind,args.port),store,api_key=key)
    bridge=None
    if args.water_bridge_port:
        if args.backend=='water':parser.error('WATER-to-WATER proxy is not a supported topology')
        from .compat import WaterServer
        bridge=WaterServer(('127.0.0.1',args.water_bridge_port),store)
        threading.Thread(target=bridge.serve_forever,daemon=True).start()
    stopping=threading.Event();server.timeout=.5
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stopping.set())
    print(json.dumps({'http':f'http://{args.bind}:{server.server_port}','backend':backend.name,
                      'water_profile':'navbridge-water-lf/1' if bridge else None,'physical_execution':args.backend!='mock'}),flush=True)
    try:
        while not stopping.is_set():server.handle_request()
    finally:
        server.server_close()
        if bridge:bridge.shutdown();bridge.server_close()
        store.close();lock.close()
if __name__=='__main__':main()
