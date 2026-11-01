#!/usr/bin/env python3
"""Actual ROS2/Nav2 reception integration, synthetic rays and planar body only.

No physical profile, gait socket, battery, driver or vendor model is used.
"""
import argparse,copy,hashlib,json,math,os,shlex,signal,subprocess,sys,time,urllib.request,urllib.error
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];NAV=ROOT/'navigation';sys.path.insert(0,str(NAV))

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--setup',required=True);ap.add_argument('--port',type=int,default=8947);ap.add_argument('--domain',type=int,default=208);ap.add_argument('--stop-profile',choices=['baseline-directional','legacy-rectangle'],default='baseline-directional');ap.add_argument('--scenario',choices=['transit','all','obstacle'],default='all');args=ap.parse_args()
    out=Path(args.output).resolve().parent
    if out.exists()and any(out.iterdir()):raise ValueError('Use a new output directory; old failures are retained')
    out.mkdir(parents=True,exist_ok=True);runtime=out/'runtime';runtime.mkdir()
    setup=Path(args.setup).resolve();assert setup.is_file()
    env={k:v for k,v in os.environ.items()if not k.startswith('P2_NAV_')and k!='P2_RECEPTION_TTS_COMMAND'}
    env.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',P2_NAV_CONTROLLER_SETUP=str(setup))
    os.environ.update({k:env[k]for k in ['ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY']})
    if args.stop_profile=='baseline-directional':env['P2_NAV_DIRECTIONAL_STOP']='true'
    # Explicit optional loopback DDS configuration supplied by the caller.
    from sim_world import save_truth
    from reception_catalog import blank
    door={'id':'front_door','x':5.9,'y':2.5,'yaw':math.pi/2,'width':2.4,'thickness':.15,'closed':True}
    doors=[door]if args.scenario=='all'else []
    mapdir=runtime/'maps/native_showroom';save_truth(mapdir,doors);(mapdir/'mapped.yaml').write_text((mapdir/'truth.yaml').read_text())
    shape=[[.275,.275],[.275,-.275],[-.275,-.275],[-.275,.275]]
    foot=out/'footprint.json';foot.write_text(json.dumps({'schema_version':1,'vertices':shape}))
    (runtime/'planning_geometry.json').write_text(json.dumps({'footprint':shape,'footprint_padding':0,'global_obstacle_combination':0}))
    if doors:env['P2_NAV_SIM_DOORS']=json.dumps(doors)
    shell='source /opt/ros/humble/setup.bash\nsource '+shlex.quote(str(setup))+'\nexport PYTHONPATH=/usr/lib/python3/dist-packages:'+shlex.quote(str(NAV))+':${PYTHONPATH:-}\nexec /usr/bin/python3 -S '+shlex.quote(str(NAV/'console.py'))+' --simulation --gait-profile p2-flat-model --footprint-profile '+shlex.quote(str(foot))+' --runtime '+shlex.quote(str(runtime))+' --port '+str(args.port)
    log=(out/'console.log').open('w');process=subprocess.Popen(['bash','-c',shell],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    url=f'http://127.0.0.1:{args.port}';token='';latest={};trace=[];events=[];report={};started=time.monotonic();ros=None
    def call(path,data=None):
        raw=None if data is None else json.dumps(data).encode();request=urllib.request.Request(url+path,data=raw,headers={'Content-Type':'application/json','X-Nav-Token':token})
        try:
            with urllib.request.urlopen(request,timeout=20)as response:return json.load(response)
        except urllib.error.HTTPError as e:raise ValueError(path+': '+e.read().decode())
    def status():
        nonlocal latest
        latest=call('/api/status?summary=1')
        if ros:ros.spin()
        if latest.get('truth'):trace.append({'truth':latest['truth'],'feedback':latest['feedback'],'session':latest['session'],'path_source':latest.get('path_source',{}),'mission':{k:latest['reception']['mission'][k]for k in ['phase','index','epoch']}})
        return latest
    def wait(predicate,seconds=50):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if process.poll()is not None:raise RuntimeError('console exited')
            try:d=status()
            except urllib.error.URLError:
                time.sleep(.1);continue
            if predicate(d):return d
            if d.get('reception',{}).get('mission',{}).get('phase')=='failed':raise RuntimeError(d['reception']['mission']['error'])
            time.sleep(.1)
        raise TimeoutError('native reception stage timeout')
    def point(i,x,y,kind='transit'):
        return {'id':i,'name':i,'x':x,'y':y,'yaw':None,'product_id':'','kind':kind,'dwell':0,'guest_wait':False,'script':''}
    def save(document):return call('/api/save',{'reception':{'operation':'save','document':document,'expected_revision':document['revision']}})
    def start(document,route='tour'):
        return call('/api/confirm',{'reception_command':{'command':'start','route_id':route,'revision':document['revision']}})
    def complete():
        d=wait(lambda d:d['reception']['mission']['phase']=='completed',90)
        assert d['truth']['collision_attempts']==0,d['truth']
        return d
    class ScenarioROS:
        def __init__(self):
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
            from sensor_msgs.msg import LaserScan
            from rclpy.qos import qos_profile_sensor_data
            rclpy.init();self.rclpy=rclpy;self.String=String;self.node=Node('reception_synthetic_scenario');self.scan=None
            self.pub=self.node.create_publisher(String,'/p2_nav/sim_control',10)
            self.sub=self.node.create_subscription(LaserScan,'/scan',lambda m:setattr(self,'scan',m),qos_profile_sensor_data)
        def spin(self):self.rclpy.spin_once(self.node,timeout_sec=0.)
        def control(self,data):
            message=self.String();message.data=json.dumps(data);self.pub.publish(message);self.spin()
        def obstacle_rays(self,d,circle):
            if not self.scan:return 0
            pose=d['truth']['pose'];count=0
            for i,r in enumerate(self.scan.ranges):
                if not math.isfinite(r):continue
                a=self.scan.angle_min+i*self.scan.angle_increment+pose[2]
                x=pose[0]+r*math.cos(a);y=pose[1]+r*math.sin(a)
                if abs(math.hypot(x-circle[0],y-circle[1])-circle[2])<.12:count+=1
            return count
        def close(self):self.node.destroy_node();self.rclpy.shutdown()
    try:
        token=wait(lambda d:'request_token'in d,15)['request_token']
        call('/api/start',{'mode':'navigation','name':'native_showroom','controller':'rpp'})
        wait(lambda d:d.get('gate',{}).get('localized')and d['gate'].get('collision_source_valid')is True and d['gate'].get('data_fresh')is True and d.get('pose'),50)
        call('/api/sim_operator',{'mode':'navigation_allowed'});wait(lambda d:d['session']['navigation_permitted'],5)
        call('/api/speed',{'speed':.75,'velocity_scale':.5})
        d=status();doc=blank('native_showroom')
        if args.scenario!='obstacle':
            doc=blank('native_showroom');doc['points']=[point('a',2.8,2.5),point('b',3.6,2.5),point('c',4.4,2.5,'visit')]
            doc['routes']=[{'id':'tour','name':'continuous transit','mode':'showcase','loops':1,'description':'synthetic','steps':[{'point_id':i,'speak':False}for i in ['a','b','c']]}]
            doc=save(doc);start(doc);d=complete();mission=d['reception']['mission']
            assert sum(e['kind']=='arrived'for e in mission['events'])==1
            assert sum(e['kind']=='transit_segment_completed'for e in mission['events'])==2
            assert sum(e['kind']=='leg_started'for e in mission['events'])==1
            windows={}
            for x in [2.8,3.6]:
                samples=[t['truth']for t in trace if abs(t['truth']['pose'][0]-x)<.12]
                assert samples and min(t['command'][0]for t in samples)>.05,('transit stopped or unobserved',x)
                windows[str(x)]={'samples':len(samples),'minimum_forward':min(t['command'][0]for t in samples)}
            events.append({'case':'continuous_native_transit','passed':True,'windows':windows})
        if args.scenario=='all':
            ros=ScenarioROS()
            # Annotate a door that really exists in the ray-cast plant and was
            # closed in the original synthetic occupancy map.
            doc['doors']=[{k:v for k,v in door.items()if k not in ('closed','thickness')}|{'name':'synthetic door'}]
            x,y=d['pose'][:2]
            doc['points']=[point('start',x,y,'home'),point('north_a',4.9,8.2),point('north_b',7.,8.2),point('door',5.9,2.5),point('goal',8.4,2.5,'visit')]
            doc['edges']=[{'id':str(i),'a':a,'b':b,'door_id':'front_door'if a=='start'and b=='door'else '', 'bidirectional':True,'penalty':0}for i,(a,b)in enumerate([('start','north_a'),('north_a','north_b'),('north_b','goal'),('start','door'),('door','goal')])]
            doc['routes'][0].update(name='adaptive door opening',mode='adaptive',steps=[{'point_id':'start','speak':False},{'point_id':'goal','speak':False}])
            doc=save(doc);closed=wait(lambda d:d['reception']['doors'].get('front_door')=='closed',8)
            evidence=closed['reception']['door_evidence']['doors'];assert any(e['blocking_returns']>=2 for e in evidence)
            start(doc);wait(lambda d:d['reception']['mission']['phase']=='navigating'and d['reception']['mission']['point']['id']=='goal',30)
            open_door=dict(door,closed=False);control={'doors':[open_door],'control_id':'reception-door-open'}
            for _ in range(8):ros.control(control);time.sleep(.08)
            wait(lambda d:d['truth']['controls'].get('control_id')=='reception-door-open',5)
            opened=wait(lambda d:d['reception']['doors'].get('front_door')=='open',15)
            assert opened['reception']['door_overlay']['changed_cells']>0
            assert opened['reception']['door_overlay']['unknown_cells_promoted']==0
            rerouted=wait(lambda d:any(e.get('kind')=='live_update'for e in d['reception']['route_report']),15)
            d=complete();assert any(e['kind']=='route_replanning'for e in d['reception']['mission']['events'])
            events.append({'case':'observed_closed_door_reopens_and_live_shortcut','passed':True,'evidence':opened['reception']['door_evidence'],'overlay':opened['reception']['door_overlay'],'decisions':rerouted['reception']['route_report']})
        if args.scenario in ('all','obstacle'):
            # Actual ray-cast obstacle affects Nav2; ground truth is used only
            # by this independent witness and is never sent as localization.
            if ros is None:ros=ScenarioROS()
            x,y=d['pose'][:2]
            if not doc['routes']:doc['routes']=[dict(id='tour',name='dynamic obstacle',mode='showcase',loops=1,description='',steps=[])]
            circle=[x+1.5,y,.28]
            doc['points']=[point('finish',min(10.95,x+3.),y,'visit')];doc['edges']=[];doc['routes'][0].update(name='dynamic obstacle',mode='showcase',steps=[{'point_id':'finish','speak':False}]);doc=save(doc)
            start(doc);wait(lambda d:d['session']['phase']=='executing',10)
            control={'obstacle':circle,'control_id':'reception-person-block'}
            for _ in range(8):ros.control(control);time.sleep(.08)
            observed=wait(lambda d:d['truth']['controls']['control_id']=='reception-person-block'and ros.obstacle_rays(d,circle)>=2,5)
            rays=ros.obstacle_rays(observed,circle);d=complete()
            assert d['truth']['controls']['obstacle']==circle and rays>=2
            witnessed=[t['truth']['pose']for t in trace if t['truth']['controls'].get('control_id')=='reception-person-block']
            assert max(abs(p[1]-circle[1])for p in witnessed)>.45,'no lateral detour observed'
            events.append({'case':'actual_dynamic_obstacle_and_complete','passed':True,'ray_returns':rays,'circle':circle})
        if args.scenario=='all':
            doc['points']=[point('finish',10.5,4.25,'visit')];doc=save(doc)
            start(doc);wait(lambda d:d['reception']['mission']['phase']=='navigating',10)
            call('/api/sim_operator',{'mode':'paused'})
            paused=wait(lambda d:d['reception']['mission']['phase']=='paused'and not d['session']['cancel_pending']and max(map(abs,d['truth']['command']))<1e-6,8)
            call('/api/sim_operator',{'mode':'navigation_allowed'})
            d=wait(lambda d:d['session']['navigation_permitted'],3)
            assert d['reception']['mission']['phase']=='paused','left switch auto-resumed'
            call('/api/confirm',{'reception_command':{'command':'resume','epoch':d['reception']['mission']['epoch']}})
            d=complete();events.append({'case':'middle_pause_then_explicit_resume_native','passed':True})
            doc['points']=[point('finish',10.5,2.5,'visit')];doc=save(doc);start(doc)
            wait(lambda d:d['reception']['mission']['phase']=='navigating',8)
            call('/api/sim_operator',{'mode':'manual'})
            d=wait(lambda d:d['reception']['mission']['phase']=='manual'and not d['session']['cancel_pending']and max(map(abs,d['truth']['command']))<1e-6,8)
            call('/api/sim_operator',{'mode':'navigation_allowed'});d=wait(lambda d:d['session']['navigation_permitted'],3)
            assert d['reception']['mission']['phase']=='manual','manual takeover auto-resumed'
            history=call('/api/save',{'reception':{'operation':'history','map_id':doc['map_id']}})
            assert any(e['kind']=='resumed'for e in history['events'])and any(e['kind']=='manual'for e in history['events'])
            events.append({'case':'manual_cancel_no_auto_resume_and_durable_receipts','passed':True,'history_events':len(history['events'])})
        report={'passed':True,'cases':events,'final_pose':d['pose'],'truth':d['truth'],'elapsed_s':time.monotonic()-started,
                'stop_profile':args.stop_profile,'fixture_seed':0,'amcl_seed':'Nav2 default; stochastic localization is not forced to match one successful run','hardware_commands':0,'interfaces':os.listdir('/sys/class/net'),'source_sha':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in [NAV/'console.py',NAV/'reception_mission.py',NAV/'reception_doors.py',NAV/'sim_world.py']},
                'scope':'Actual Console, ROS2/Nav2 actions and projected-ray plant, unchanged55cm full-body model. No RL or physical robot acceptance.'}
    except Exception as error:
        latest=copy.deepcopy(latest);latest.pop('request_token',None)
        report={'passed':False,'source_sha':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in [NAV/'console.py',NAV/'reception_mission.py',NAV/'reception_doors.py',NAV/'sim_world.py']},'error':str(error),'cases':events,'latest':latest,'elapsed_s':time.monotonic()-started,'stop_profile':args.stop_profile,'hardware_commands':0,'scope':'Actual isolated ROS2/Nav2 + synthetic planar plant; failure retained.'}
    finally:
        (out/'trace.json').write_text(json.dumps(trace,ensure_ascii=False))
        if process.poll()is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=25)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        if ros:ros.close()
        log.close();Path(args.output).write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in report.items()if k not in ('latest','source_sha','truth')},ensure_ascii=False))
    return 0 if report.get('passed')else 1
if __name__=='__main__':sys.exit(main())
