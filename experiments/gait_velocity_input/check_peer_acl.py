import os,sys,pathlib,tempfile,subprocess,time,json,shutil
import gait_socket_client
from gait_socket_client import GaitClient,Operation,Authority
if len(sys.argv)==3 and sys.argv[1]=='--client':
 try:
  c=GaitClient(sys.argv[2],1);r=c.exchange(Operation.AUTHORITY,Authority(0,0,0,0,0,0,0));c.close();assert r.controller==1
  print('received_actual_loop_reply')
 except (OSError,ValueError) as e:print(type(e).__name__);sys.exit(4)
 sys.exit(0)
binary=sys.argv[1];results={}
with tempfile.TemporaryDirectory(prefix='p2-acl-') as root:
 os.chmod(root,0o755)
 # CI checkout ancestors may be private to the runner. Drop privileges only
 # after copying the exact client module/probe to this public temporary root.
 shutil.copyfile(gait_socket_client.__file__,pathlib.Path(root)/'gait_socket_client.py')
 probe=pathlib.Path(root)/'probe.py'
 probe.write_text(pathlib.Path(__file__).read_text().split('binary=sys.argv[1]')[0])
 os.chmod(probe,0o644);os.chmod(pathlib.Path(root)/'gait_socket_client.py',0o644)
 d=pathlib.Path(root)/'cat';d.mkdir(mode=0o700);os.chown(d,1000,1000);path=str(d/'gait.sock')
 bad=subprocess.run([binary,path,'0'],capture_output=True,text=True);assert bad.returncode==3
 results['mismatched_directory_owner_rejected']=True
 p=subprocess.Popen([binary,path,'1000'])
 try:
  for _ in range(100):
   if pathlib.Path(path).exists():break
   time.sleep(.01)
  for suffix in ['', '.operator']:
   st=os.stat(path+suffix);assert st.st_uid==1000 and st.st_mode&0o777==0o600
  results['socket_permissions_0600_configured_uid']=True
  def drop(uid):
   def apply():os.setgroups([]);os.setgid(uid);os.setuid(uid)
   return apply
  ok=subprocess.run([sys.executable,str(probe),'--client',path],preexec_fn=drop(1000),capture_output=True,text=True)
  assert ok.returncode==0,(ok.stdout,ok.stderr);results['root_server_configured_cat_client_receives_loop_reply']=True
  no=subprocess.run([sys.executable,str(probe),'--client',path],preexec_fn=drop(65534),capture_output=True,text=True)
  assert no.returncode==4 and 'PermissionError' in no.stdout,(no.stdout,no.stderr)
  results['other_uid_rejected']=True
  # Even if filesystem access is deliberately opened in this synthetic test,
  # SO_PEERCRED must independently reject another UID.
  os.chmod(d,0o711);os.chmod(path,0o666);time.sleep(.05)
  no=subprocess.run([sys.executable,str(probe),'--client',path],preexec_fn=drop(65534),capture_output=True,text=True)
  assert no.returncode==4,(no.stdout,no.stderr)
  results['peer_credentials_reject_other_uid']=True
  os.chmod(path,0o600);os.chmod(d,0o700)
 finally:p.wait(timeout=8)
 results['sockets_removed_on_exit']=not pathlib.Path(path).exists() and not pathlib.Path(path+'.operator').exists()
print(json.dumps({'checks':results,'hardware_io':False}))
