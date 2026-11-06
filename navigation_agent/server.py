"""Loopback-only RAI companion; may be served through the navigation Console."""
import argparse
import hmac
import importlib.metadata
import json
import os
from pathlib import Path
import secrets
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.parse import urlsplit,parse_qs
from navigation_sdk import NavigationClient
from .app import NavigationAgent,deepseek_model

class AgentServer(ThreadingHTTPServer):
    daemon_threads=True
    def __init__(self,address,agent,model):
        self.agent=agent;self.model=model;self.token=secrets.token_urlsafe(32)
        super().__init__(address,Handler)

class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def send(self,body,status=200,ctype='application/json; charset=utf-8'):
        raw=body if isinstance(body,bytes)else json.dumps(body,ensure_ascii=False,allow_nan=False).encode()
        self.send_response(status);self.send_header('Content-Type',ctype);self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff');self.send_header('Content-Length',str(len(raw)))
        self.end_headers();self.wfile.write(raw)
    def dispatch(self):
        try:
            path=urlsplit(self.path).path
            if self.headers.get('Origin')and urlsplit(self.headers['Origin']).netloc!=self.headers.get('Host'):
                return self.send({'error':'请从同一导航页面操作'},403)
            if self.command=='GET':
                if path in('/','/agent'):return self.send(Path(__file__).with_name('ui.html').read_bytes(),ctype='text/html; charset=utf-8')
                if path=='/api/agent/meta':return self.send(dict(model=self.server.model,framework='RAI',rai_version=importlib.metadata.version('rai_core'),request_token=self.server.token))
                if path=='/api/agent/status':return self.send(self.server.agent.status(parse_qs(urlsplit(self.path).query).get('conversation',[''])[0]))
                if path.startswith('/api/agent/jobs/'):return self.send(self.server.agent.journal.job(path.rsplit('/',1)[-1]))
                return self.send({'error':'未找到接口'},404)
            if not hmac.compare_digest(self.headers.get('X-Agent-Token',''),self.server.token):return self.send({'error':'请刷新AI助手页面'},403)
            if self.headers.get_content_type()!='application/json':return self.send({'error':'需要JSON请求'},415)
            length=int(self.headers.get('Content-Length','0'))
            if not 0<length<=16384:raise ValueError('请求大小无效')
            data=json.loads(self.rfile.read(length))
            if path=='/api/agent/stop':
                if data!={}:raise ValueError('停止请求不应包含其他参数')
                return self.send(self.server.agent.stop())
            if path=='/api/agent/chat':
                if set(data)!={'id','conversation','text'}:raise ValueError('聊天字段无效')
                return self.send(self.server.agent.submit(data['id'],data['conversation'],data['text']),202)
            return self.send({'error':'未找到接口'},404)
        except (ValueError,KeyError,TypeError):self.send({'error':'请求内容无效'},400)
        except (BrokenPipeError,ConnectionResetError):pass
        except Exception:self.send({'error':'服务暂时无法取得回执，请稍后查询；未自动重试动作'},503)
    do_GET=dispatch;do_POST=dispatch

def main():
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8950)
    p.add_argument('--sdk-url',default='http://127.0.0.1:8940');p.add_argument('--state-dir',type=Path,required=True);args=p.parse_args()
    os.umask(0o077);args.state_dir.mkdir(mode=0o700,parents=True,exist_ok=True)
    import fcntl
    lock=(args.state_dir/'agent.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    agent=NavigationAgent(NavigationClient(args.sdk_url,timeout=8),args.state_dir/'agent.sqlite3',deepseek_model())
    server=AgentServer(('127.0.0.1',args.port),agent,os.getenv('P2_AGENT_MODEL','deepseek-v4-pro'))
    server.serve_forever()
if __name__=='__main__':main()
