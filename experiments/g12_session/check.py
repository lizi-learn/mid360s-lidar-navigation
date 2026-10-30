#!/usr/bin/env python3
"""Bounded synthetic protocol acceptance. Requires an isolated ROS domain/runtime.
Not humanoid dynamics, live policy takeover, or physical stopping acceptance.
"""
import os,sys,time,json,subprocess,signal,threading,hashlib,traceback,argparse
from pathlib import Path
from urllib.request import urlopen,Request
from urllib.error import HTTPError
import rclpy
from std_msgs.msg import String
from nav2_msgs.msg import CollisionMonitorState
from lifecycle_msgs.srv import GetState
repository=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(repository/'navigation'))
from sim_world import save_truth
parser=argparse.ArgumentParser(description='Synthetic G12 session lifecycle + real Nav2 STOP witness; never real robot motion')
parser.add_argument('--directory',required=True);args=parser.parse_args()
root=Path(args.directory).resolve()
sys.path.insert(0,str(repository));from robot_profile import check_simulation_runtime
check_simulation_runtime(root)
root.mkdir(parents=True,exist_ok=True);runtime=root/'runtime';m=runtime/'maps'/'synthetic';save_truth(m)
(m/'truth.pgm').rename(m/'mapped.pgm');(m/'truth.yaml').rename(m/'mapped.yaml');(m/'mapped.yaml').write_text((m/'mapped.yaml').read_text().replace('truth.pgm','mapped.pgm'))
records=[];events=[];token='';p=None
rclpy.init();node=rclpy.create_node('g12_session_test_witness');publisher=node.create_publisher(String,'/p2_nav/sim_control',10)
node.create_subscription(CollisionMonitorState,'/p2_nav/collision_state',lambda x:events.append({'time':time.monotonic(),'action':int(x.action_type),'polygon':x.polygon_name}),10)
thread=threading.Thread(target=rclpy.spin,args=(node,),daemon=True);thread.start()
def get():
 global token
 with urlopen('http://127.0.0.1:8956/api/status?summary=1',timeout=3) as f:d=json.load(f)
 token=d.pop('request_token');return d

def post(action,data):
 b=json.dumps(data).encode();req=Request('http://127.0.0.1:8956/api/'+action,data=b,headers={'Content-Type':'application/json','X-Nav-Token':token})
 try:
  with urlopen(req,timeout=20) as f:r=json.load(f)
 except HTTPError as e:r={'http_status':e.code,**json.load(e)}
 records.append({'at':time.monotonic(),'action':action,'input':data,'reply':r});return r

def wait(fn,seconds=20):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  d=get()
  if fn(d):return d
  time.sleep(.1)
 raise RuntimeError('condition timeout: '+json.dumps(d,ensure_ascii=False))
def mode(x,activity=False):post('sim_operator',{'mode':x,'manual_activity':activity});time.sleep(.15)
def prepare(x):
 d=get();pose=d['pose'];r=post('goal',dict(x=pose[0]+x,y=pose[1],yaw=0.));assert r.get('planned'),r;return r['generation']
def start(x=.8):
 mode('navigation_allowed');g=prepare(x);r=post('confirm',{'generation':g});assert r.get('accepted'),r
 return wait(lambda d:abs(d.get('gate',{}).get('command',[0])[0])>.01,8)
def zero(d):return all(abs(v)<1e-6 for v in d.get('gate',{}).get('command',[999]))
checks={}
try:
 p=subprocess.Popen(['bash',str(repository/'navigation/console.sh'),'--simulation','--port','8956','--runtime',str(runtime),'--gait-profile','p2-flat-model'],stdout=open(root/'console.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
 time.sleep(2);get();post('start',{'mode':'navigation','name':'synthetic','controller':'rpp'})
 wait(lambda d:d['gate'].get('localized') and d['gate'].get('ground_ok'),40)
 for name in ('planner_server','controller_server','bt_navigator','collision_monitor'):
  client=node.create_client(GetState,'/'+name+'/get_state');assert client.wait_for_service(timeout_sec=20),name
  end=time.monotonic()+25
  while time.monotonic()<end:
   future=client.call_async(GetState.Request())
   while not future.done() and time.monotonic()<end:time.sleep(.03)
   if future.done() and future.result().current_state.id==3:break
   time.sleep(.2)
  else:raise RuntimeError(name+' was never Active')
  node.destroy_client(client)
 checks['actual_nav2_lifecycle_active']=True
 mode('navigation_allowed');g=prepare(.8);time.sleep(.5);assert zero(get());post('stop',{});d=wait(lambda d:not d['session']['cancel_pending'],10);assert zero(d);checks['preview_then_cancel_without_start']=True
 d=start();mode('manual');d=wait(lambda d:zero(d) and not d['session']['cancel_pending'],10);assert d['session']['phase']=='manual';mode('navigation_allowed');time.sleep(.6);assert zero(get());checks['right_takeover_and_no_auto_resume']=True
 d=start();mode('paused');d=wait(lambda d:zero(d) and not d['session']['cancel_pending'],10);assert d['session']['phase']=='paused';checks['middle_pauses']=True
 d=start();mode('navigation_allowed',True);d=wait(lambda d:zero(d) and not d['session']['cancel_pending'],10);assert d['session']['phase']=='manual';mode('navigation_allowed');time.sleep(.3);assert zero(get());checks['joystick_takeover_latches']=True
 d=start(.65);d=wait(lambda d:d['session']['phase'] in ('arrived','blocked'),30);assert d['session']['phase']=='arrived',d['session'];assert zero(wait(zero,3));checks['confirmed_goal_completed']=True
 d=start(1.1);pose=d['truth']['pose'];control={'obstacle':[pose[0]+.85,pose[1],.20],'control_id':'g12-new-session-obstacle-001'};msg=String();msg.data=json.dumps(control)
 for _ in range(3):publisher.publish(msg);time.sleep(.06)
 d=wait(lambda d:d['truth'].get('controls',{}).get('control_id')==control['control_id'],3);checks['obstacle_control_ack']=True
 d=wait(lambda d:any(e['action']==int(CollisionMonitorState.STOP) for e in events) and zero(d),5);checks['actual_collision_monitor_stop']=True
 d=wait(lambda d:d['session']['phase'] in ('blocked','arrived'),12);checks['persistent_obstacle_bounded_zero_and_notice']=d['session']['phase']=='blocked' and zero(d)
 checks['zero_envelope_intrusions']=d['truth']['collision_attempts']==0
 post('stop',{});final=get();checks['joint_protocol_pass']=all(checks.values())
 result={'checks':checks,'final':final,'events':events,'requests':records,'model':'synthetic raycast planar kinematics; NOT P2 dynamics or physical acceptance','seed':0,'error':None}
except Exception as e:
 result={'checks':checks,'events':events,'requests':records,'error':str(e),'traceback':traceback.format_exc(),'model':'synthetic raycast planar kinematics; NOT P2 dynamics or physical acceptance','seed':0}
finally:
 if p is not None and p.poll() is None:
  os.killpg(p.pid,signal.SIGTERM)
  try:p.wait(timeout=25)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 result['source_sha256']={str(x.relative_to(repository)):hashlib.sha256(x.read_bytes()).hexdigest() for x in (repository/'navigation').rglob('*') if x.is_file() and x.suffix in ('.py','.xml','.html')}
 (root/'result.private.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));rclpy.shutdown();thread.join(timeout=3);node.destroy_node()
 print(json.dumps({'checks':checks,'error':result['error']},ensure_ascii=False),flush=True)
