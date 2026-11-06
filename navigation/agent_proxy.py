"""Serve the independent Agent through the existing loopback Console origin."""
import json
from urllib.parse import urlsplit
import urllib.request
import urllib.error

def serve(handler):
    if handler.headers.get('Origin')and urlsplit(handler.headers['Origin']).netloc!=handler.headers.get('Host'):
        handler.send({'error':'请从同一导航页面操作'},403);return
    headers={};data=None
    if handler.command=='POST':
        if handler.headers.get_content_type()!='application/json':handler.send({'error':'需要JSON'},415);return
        try:length=int(handler.headers.get('Content-Length','0'))
        except ValueError:handler.send({'error':'无效请求'},400);return
        if not 0<length<=16384:handler.send({'error':'请求过大'},413);return
        data=handler.rfile.read(length)
        headers={'Content-Type':'application/json','X-Agent-Token':handler.headers.get('X-Agent-Token','')}
    request=urllib.request.Request('http://127.0.0.1:8950'+handler.path,data=data,headers=headers)
    try:
        with urllib.request.urlopen(request,timeout=12)as response:
            handler.send(response.read(),response.status,response.headers.get('Content-Type','application/json'))
    except urllib.error.HTTPError as error:handler.send(error.read(),error.code,'application/json; charset=utf-8')
    except OSError:handler.send({'error':'AI助手服务暂未连接，导航原功能仍可使用'},503)
