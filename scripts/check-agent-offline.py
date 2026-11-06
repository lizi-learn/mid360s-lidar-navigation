"""Run Agent tests with no provider keys and only in-process test-server sockets."""
from pathlib import Path
import argparse,hashlib,ipaddress,json,os,socket,sys,time,unittest,weakref
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root))
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
for key in list(os.environ):
 if key.endswith(('API_KEY','SECRET_KEY'))or key in ('LANGFUSE_PUBLIC_KEY','LANGSMITH_ENDPOINT'):os.environ.pop(key,None)
os.environ.update(LANGSMITH_TRACING='false',LANGCHAIN_TRACING_V2='false',LANGFUSE_TRACING_ENABLED='false',OTEL_SDK_DISABLED='true')
for name in ('HTTP_PROXY','HTTPS_PROXY','ALL_PROXY','http_proxy','https_proxy','all_proxy'):os.environ.pop(name,None)
os.environ['NO_PROXY']='localhost,127.0.0.1,::1'
listeners=weakref.WeakSet();blocked=[];connections=0

def local(host):
 try:return ipaddress.ip_address(host).is_loopback
 except ValueError:return host=='localhost'
def audit(event,values):
 global connections
 if event=='socket.bind':
  sock,address=values
  if isinstance(address,tuple)and local(address[0]):listeners.add(sock)
 elif event=='socket.getaddrinfo':
  if values[0]is not None and not local(values[0]):
   blocked.append({'kind':'DNS','host':str(values[0])});raise PermissionError('Offline tests do not resolve external hosts')
 elif event=='socket.connect':
  sock,address=values
  if not isinstance(address,tuple):return
  ports=set()
  for listener in tuple(listeners):
   try:ports.add(listener.getsockname()[1])
   except OSError:pass
  if not local(address[0])or address[1]not in ports:
   blocked.append({'kind':'connect','host':str(address[0]),'port':address[1]});raise PermissionError('Offline tests connect only to their own bound loopback servers')
  connections+=1
sys.addaudithook(audit)
# Check the guard before importing the application; this issues no DNS/network call.
try:socket.getaddrinfo('api.deepseek.com',443)
except PermissionError:guard_verified=True
else:raise RuntimeError('Offline network guard did not work')
blocked.clear()
start=time.monotonic();suite=unittest.defaultTestLoader.discover(str(root/'tests_agent'))
result=unittest.TextTestRunner(verbosity=2).run(suite)
report={'tests':result.testsRun,'failures':len(result.failures),'errors':len(result.errors),'skipped':len(result.skipped),
 'seconds':round(time.monotonic()-start,3),'provider_requests':0,'network_guard_verified':guard_verified,
 'application_blocked_network_attempts':blocked,'test_loopback_connections':connections,
 'isolation':'API keys removed in child; only servers bound by this test process may receive connects',
 'passed':result.wasSuccessful()and not blocked,
 'failure_cases':[str(t)for t,_ in result.failures+result.errors]}
report['native_engine_evidence']=getattr(sys.modules.get('test_mission_integration'),'EVIDENCE',[])
report['source_sha256']={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()for directory in ('navigation_agent','navigation_sdk','tests_agent')for p in sorted((root/directory).glob('*'))if p.is_file()}
args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False))
raise SystemExit(0 if report['passed']else 1)
