"""Narrow same-origin gateway to the existing localhost navigation workbench.
No robot input writer, arbitrary destination, battery, process or motor controls.
"""
import json
from urllib.request import Request as HttpRequest,urlopen
from urllib.error import HTTPError,URLError
from urllib.parse import urlsplit
from fastapi import APIRouter,Request,HTTPException
from fastapi.responses import Response,JSONResponse
from starlette.concurrency import run_in_threadpool

api_router=APIRouter(prefix='/api/v1/modules/navigation')
BASE='http://127.0.0.1:8930'
GETS={'/api/status','/api/map'}
POSTS={'/api/goal','/api/confirm','/api/stop','/api/speed','/api/initialpose','/api/start','/api/save'}
def local(path,body=None,token=None):
    headers={'Content-Type':'application/json'}
    if token:headers['X-Nav-Token']=token
    req=HttpRequest(BASE+path,data=body,headers=headers)
    try:
        with urlopen(req,timeout=25 if body else 5) as r:return r.status,r.read(4*1024*1024),r.headers.get('Content-Type','application/json')
    except HTTPError as e:return e.code,e.read(16384),'application/json'
    except (URLError,TimeoutError) as e:raise HTTPException(503,'导航服务暂未连接') from e
@api_router.get('/snapshot')
def snapshot():
    status,body,_=local('/api/status?summary=1')
    if status!=200:raise HTTPException(503,'导航状态不可用')
    data=json.loads(body);data.pop('request_token',None)
    return {'ok':True,'data':data}
@api_router.post('/actions/cancel')
def cancel(request:Request):
    if not request.headers.get('X-YMbot-Client','').startswith('g12-console/'):
        raise HTTPException(403,'缺少操作台请求标记')
    code,body,_=local('/api/status?summary=1')
    if code!=200:raise HTTPException(503,'导航状态不可用')
    token=json.loads(body)['request_token'];code,body,_=local('/api/stop',b'{}',token)
    return JSONResponse({'ok':code==200,'data':json.loads(body)},status_code=code)
@api_router.get('/workbench')
def workbench():
    code,body,content_type=local('/')
    return Response(body,status_code=code,media_type='text/html',headers={'Cache-Control':'no-store'})
@api_router.api_route('/proxy/{path:path}',methods=['GET','POST'])
async def proxy(path:str,request:Request):
    path='/'+path
    if request.method=='GET':
        if path not in GETS:raise HTTPException(404,'未开放此导航接口')
        if path=='/api/status' and request.query_params.get('summary')=='1':path+='?summary=1'
        code,body,content_type=await run_in_threadpool(local,path)
    else:
        if path not in POSTS:raise HTTPException(404,'未开放此导航接口')
        origin=request.headers.get('Origin')
        if origin and urlsplit(origin).netloc!=request.url.netloc:raise HTTPException(403,'拒绝跨站导航操作')
        if request.headers.get('Content-Type','').split(';')[0]!='application/json':raise HTTPException(415,'需要JSON请求')
        length=request.headers.get('Content-Length','')
        if not length.isdigit() or not 0<int(length)<=8192:raise HTTPException(413,'请求大小无效')
        data=await request.body()
        if len(data)>8192:raise HTTPException(413,'请求大小无效')
        code,body,content_type=await run_in_threadpool(local,path,data,request.headers.get('X-Nav-Token',''))
    return Response(body,status_code=code,media_type=content_type.split(';')[0],headers={'Cache-Control':'no-store'})
