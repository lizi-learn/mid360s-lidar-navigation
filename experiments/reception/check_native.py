#!/usr/bin/env python3
"""Actual ROS2/Nav2 reception integration, synthetic rays and planar body only.

No physical profile, gait socket, battery, driver or vendor model is used.
"""
import argparse,copy,hashlib,json,math,os,shlex,signal,subprocess,sys,time,urllib.request,urllib.error
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];NAV=ROOT/'navigation';sys.path.insert(0,str(NAV))

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--setup',required=True);ap.add_argument('--port',type=int,default=8947);ap.add_argument('--domain',type=int,default=208);ap.add_argument('--stop-profile',choices=['baseline-directional','legacy-rectangle'],default='baseline-directional');ap.add_argument('--scenario',choices=['transit','all','obstacle','tools','maps','blockage','followthrough'],default='all');ap.add_argument('--block-case',choices=['persistent','transient','takeover'],default='persistent');ap.add_argument('--obstacle-offset',type=float,default=0.);args=ap.parse_args()
    if not math.isfinite(args.obstacle_offset):raise ValueError('Invalid synthetic obstacle offset')
    out=Path(args.output).resolve().parent
    if out.exists()and any(out.iterdir()):raise ValueError('Use a new output directory; old failures are retained')
    out.mkdir(parents=True,exist_ok=True);runtime=out/'runtime';runtime.mkdir()
    setup=Path(args.setup).resolve();assert setup.is_file()
    env={k:v for k,v in os.environ.items()if not k.startswith('P2_NAV_')and k!='P2_RECEPTION_TTS_COMMAND'}
    env.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',P2_NAV_CONTROLLER_SETUP=str(setup))
    os.environ.update({k:env[k]for k in ['ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY']})
    if args.stop_profile=='baseline-directional':env['P2_NAV_DIRECTIONAL_STOP']='true'
    # Explicit optional loopback DDS configuration supplied by the caller.
    if args.scenario=='blockage':
        env['P2_NAV_SIM_CORRIDOR']='1';os.environ['P2_NAV_SIM_CORRIDOR']='1'
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
    url=f'http://127.0.0.1:{args.port}';token='';latest={};trace=[];events=[];report={};started=time.monotonic();ros=None;presentation=None;presentation_trace=None
    def call(path,data=None):
        raw=None if data is None else json.dumps(data).encode();request=urllib.request.Request(url+path,data=raw,headers={'Content-Type':'application/json','X-Nav-Token':token})
        try:
            with urllib.request.urlopen(request,timeout=20)as response:return json.load(response)
        except urllib.error.HTTPError as e:raise ValueError(path+': '+e.read().decode())
    def status():
        nonlocal latest
        latest=call('/api/status?summary=1')
        if presentation:
            from audio_feedback import atomic_json
            atomic_json(runtime/'audio-feedback-state.json',dict(updated_at=time.time()))
            presentation.observe(latest['reception']['mission'],enabled=True,volume=55,now=time.monotonic())
        if ros:ros.spin();ros.snapshot(latest)
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
            from nav2_msgs.msg import Costmap
            from nav_msgs.msg import Path as ROSPath
            from tf2_ros import Buffer,TransformListener
            from rclpy.qos import qos_profile_sensor_data
            rclpy.init();self.rclpy=rclpy;self.String=String;self.node=Node('reception_synthetic_scenario');self.scan=None;self.local_map=None;self.global_map=None;self.plan=None;self.snapshot_at=0.;self.snapshot_index=0
            self.tf=Buffer();self.tf_listener=TransformListener(self.tf,self.node)
            self.plan_sub=self.node.create_subscription(ROSPath,'/plan',lambda m:setattr(self,'plan',m),10)
            self.pub=self.node.create_publisher(String,'/p2_nav/sim_control',10)
            self.local_sub=self.node.create_subscription(Costmap,'/local_costmap/costmap_raw',lambda m:setattr(self,'local_map',m),10)
            self.global_sub=self.node.create_subscription(Costmap,'/global_costmap/costmap_raw',lambda m:setattr(self,'global_map',m),10)
            self.sub=self.node.create_subscription(LaserScan,'/scan',lambda m:setattr(self,'scan',m),qos_profile_sensor_data)
        def spin(self):
            for _ in range(12):self.rclpy.spin_once(self.node,timeout_sec=0.)
        def snapshot(self,d):
            if d.get('truth',{}).get('controls',{}).get('control_id')!='reception-person-block':return
            if time.monotonic()-self.snapshot_at<.5 or self.snapshot_index>=60:return
            self.snapshot_at=time.monotonic();self.snapshot_index+=1
            folder=out/'obstacle-snapshots';folder.mkdir(exist_ok=True)
            record={'truth':d['truth'],'estimate_pose':d.get('pose'),'session':d['session'],'maps':{}}
            for name,g in [('local',self.local_map),('global',self.global_map)]:
                if g is None:continue
                m=g.metadata;stem=f'{self.snapshot_index:03d}-{name}'
                (folder/(stem+'.map.txt')).write_text(f'{m.size_x} {m.size_y} {m.resolution:.17g} {m.origin.position.x:.17g} {m.origin.position.y:.17g}\n'+' '.join(map(str,g.data)))
                item={'frame':g.header.frame_id,'source_ros_ns':str(g.header.stamp.sec*1000000000+g.header.stamp.nanosec),'file':stem+'.map.txt'}
                try:
                    t=self.tf.lookup_transform(g.header.frame_id,'base_link',self.rclpy.time.Time());q=t.transform.rotation
                    item['robot_pose']=[t.transform.translation.x,t.transform.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
                    item['pose_source_ros_ns']=str(t.header.stamp.sec*1000000000+t.header.stamp.nanosec)
                except Exception as e:item['transform_error']=str(e)
                if self.plan:
                    try:
                        t=self.tf.lookup_transform(g.header.frame_id,self.plan.header.frame_id,self.rclpy.time.Time());q=t.transform.rotation
                        item['plan_transform']=[t.transform.translation.x,t.transform.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
                        item['plan_transform_source_ros_ns']=str(t.header.stamp.sec*1000000000+t.header.stamp.nanosec)
                    except Exception as e:item['plan_transform_error']=str(e)
                record['maps'][name]=item
            if self.plan:
                record['plan_frame']=self.plan.header.frame_id
                record['plan_source_ros_ns']=str(self.plan.header.stamp.sec*1000000000+self.plan.header.stamp.nanosec)
                record['path']=[[p.pose.position.x,p.pose.position.y,math.atan2(2*(p.pose.orientation.w*p.pose.orientation.z+p.pose.orientation.x*p.pose.orientation.y),1-2*(p.pose.orientation.y**2+p.pose.orientation.z**2))]for p in self.plan.poses]
            (folder/(f'{self.snapshot_index:03d}.json')).write_text(json.dumps(record))
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
        if args.scenario not in ('obstacle','tools','maps','blockage','followthrough'):
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
        if args.scenario=='followthrough':
            # Real native navigation results, real playback subprocess lifecycle;
            # only sound output is a short synthetic WAV and fake paplay.
            import wave
            from reception_voice import PresentationPlayer
            from reception_catalog import content_key
            voice=runtime/'reception-voice-cache';voice.mkdir(exist_ok=True)
            text='Synthetic repeated product introduction.';key=content_key(text)
            with wave.open(str(voice/(key+'.wav')),'wb')as wav:
                wav.setparams((1,2,24000,0,'NONE','none'));wav.writeframes(b'\0\0'*12000)
            fake=out/'fake-audio';fake.mkdir();presentation_trace=out/'presentation-processes.txt'
            player=fake/'paplay'
            player.write_text('\n'.join(['#!/usr/bin/python3', 'import sys,time',
                'with open('+repr(str(presentation_trace))+',"a")as f:',
                '    f.write(" ".join(sys.argv[1:])+"\\n")', 'time.sleep(.25)', '']))
            player.chmod(0o755);os.environ['PATH']=str(fake)+os.pathsep+os.environ['PATH']
            presentation=PresentationPlayer(voice,runtime/'reception-speech-result.json')
            a=point('product_a',2.8,2.5,'visit');a.update(yaw=.6,script=text,guest_wait=True)
            b=point('product_b',3.8,2.5,'visit');b.update(yaw=0.,script=text,dwell=1.)
            home=point('home',2.,2.5,'home')
            doc['points']=[point('via',2.4,2.5),a,b,home];doc['routes']=[dict(id='tour',name='Heading, product, guest wait, dwell, return',mode='showcase',loops=1,description='',steps=[dict(point_id=p['id'],speak=True)for p in doc['points']])]
            doc=save(doc);start(doc)
            parked=wait(lambda d:d['reception']['mission']['phase']=='guest_wait',45)
            assert parked['reception']['mission']['index']==1
            error=abs(math.remainder(parked['pose'][2]-a['yaw'],2*math.pi));assert error<.15,error
            epoch=parked['reception']['mission']['epoch'];pose=parked['truth']['pose']
            time.sleep(.6);parked=status()
            assert parked['reception']['mission']['phase']=='guest_wait'
            assert max(map(abs,parked['truth']['command']))<1e-6
            assert math.dist(parked['truth']['pose'][:2],pose[:2])<.01
            call('/api/confirm',{'reception_command':dict(command='continue',epoch=epoch)})
            dwelling=wait(lambda d:d['reception']['mission']['phase']=='dwell',45)
            assert dwelling['reception']['mission']['index']==2
            assert abs(math.remainder(dwelling['pose'][2]-b['yaw'],2*math.pi))<.15
            assert dwelling['reception']['mission']['epoch']!=epoch
            assert max(map(abs,dwelling['truth']['command']))<1e-6
            d=complete()
            assert len(presentation_trace.read_text().splitlines())==2
            assert d['truth']['collision_attempts']==0
            events.append(dict(case='native_heading_then_same_product_twice_guest_continue_dwell_return',passed=True,
                first_yaw_error=error,actual_playback_processes=2,guest_wait_held=True,dwell_observed=True,
                audio_scope='Synthetic silent WAV and fake paplay; not a physical speaker test'))
        if args.scenario=='blockage':
            ros=ScenarioROS()
            doc=blank('native_showroom');doc['points']=[point('finish',5.0,2.5,'visit')]
            doc['routes']=[dict(id='tour',name='complete obstruction test',mode='showcase',loops=1,description='',steps=[dict(point_id='finish',speak=False)])]
            doc=save(doc);start(doc)
            moving=wait(lambda d:d['session']['phase']=='executing' and d['truth']['command'][0]>.08,12)
            wall=dict(id='blocked_corridor',x=moving['truth']['pose'][0]+1.05,y=5.,yaw=math.pi/2,width=10.,thickness=.15,closed=True)
            wall_start=time.monotonic();control={'doors':[wall],'control_id':'full-width-block'}
            for _ in range(8):ros.control(control);time.sleep(.08)
            observed=wait(lambda d:d['truth']['controls'].get('control_id')=='full-width-block',4)
            def returns(d):
                if ros.scan is None:return 0
                pose=d['truth']['pose'];n=0
                for i,value in enumerate(ros.scan.ranges):
                    if not math.isfinite(value):continue
                    a=pose[2]+ros.scan.angle_min+i*ros.scan.angle_increment
                    x=pose[0]+value*math.cos(a);y=pose[1]+value*math.sin(a)
                    if abs(x-wall['x'])<.14 and 0<y<10:n+=1
                return n
            def section(grid):
                if grid is None:return False
                m=grid.metadata
                for sample in range(43):
                    y=1.45+sample*.05;iy=math.floor((y-m.origin.position.y)/m.resolution);found=False
                    for dx in [-.15,-.10,-.05,0.,.05,.10,.15]:
                        ix=math.floor((wall['x']+dx-m.origin.position.x)/m.resolution)
                        if 0<=ix<m.size_x and 0<=iy<m.size_y and grid.data[iy*m.size_x+ix]>=254:found=True;break
                    if not found:return False
                return True
            observed=wait(lambda d:returns(d)>=2 and section(ros.global_map)and section(ros.local_map),8);rays=returns(observed)
            cross_section={'global_lethal_span':section(ros.global_map),'local_lethal_span':section(ros.local_map),'y_interval':[1.45,3.55]}

            stopped=[];terminal=None;stop_start=None
            while time.monotonic()-wall_start<29.:
                d=status();elapsed=time.monotonic()-wall_start
                if max(map(abs,d['truth']['command']))<1e-5:
                    if stop_start is None:stop_start=time.monotonic()
                    stopped.append([elapsed,d['truth']['pose'],d['session']['phase']])
                elif stop_start is not None and elapsed>3.:raise AssertionError('renewed motion while wall fully blocks corridor')
                if d['reception']['mission']['phase']in ('failed','blocked','completed','cancelled','manual'):
                    terminal={'elapsed_s':elapsed,'mission_phase':d['reception']['mission']['phase'],'session':d['session']};break
                if args.block_case in ('transient','takeover')and stop_start is not None and time.monotonic()-stop_start>2.:break
                time.sleep(.1)
            witness={'cross_section':cross_section,'wall':wall,'ray_returns':rays,'stopped_samples':len(stopped),'terminal':terminal,'collision_attempts':d['truth']['collision_attempts']}
            events.append(dict(case='full_width_obstruction_witness',passed=bool(stopped)and rays>=2,witness=witness))
            assert stopped and rays>=2 and d['truth']['collision_attempts']==0,witness
            if args.block_case=='persistent':
                assert terminal is not None,'did not terminate persistent obstruction'
                assert terminal['elapsed_s']>=doc['policy']['max_wait']-.5,('premature terminal before configured wait',witness)
                assert d['session']['phase']=='blocked',d['session']
                for _ in range(8):ros.control({'doors':[dict(wall,closed=False)],'control_id':'wall-open-after-timeout'});time.sleep(.08)
                until=time.monotonic()+3.
                while time.monotonic()<until:
                    d=status();assert d['session']['phase']=='blocked'and max(map(abs,d['truth']['command']))<1e-6;time.sleep(.1)
                witness['no_auto_restart_after_timeout_and_reopen']=True
                events.append(dict(case='persistent_obstruction_wait_timeout_zero',passed=True,witness=witness))
            else:
                assert terminal is None,('premature navigation termination',witness)
                if args.block_case=='takeover':
                    call('/api/sim_operator',{'mode':'manual'})
                    d=wait(lambda d:d['session']['phase']=='manual'and not d['session']['cancel_pending'],6)
                opened=dict(wall,closed=False)
                for _ in range(8):ros.control({'doors':[opened],'control_id':'wall-reopened'});time.sleep(.08)
                if args.block_case=='transient':
                    d=complete();events.append(dict(case='full_block_then_clear_replan_complete',passed=True,witness=witness))
                else:
                    call('/api/sim_operator',{'mode':'navigation_allowed'})
                    until=time.monotonic()+3.
                    while time.monotonic()<until:
                        d=status();assert d['session']['phase']=='manual'and max(map(abs,d['truth']['command']))<1e-6;time.sleep(.1)
                    events.append(dict(case='takeover_while_blocked_no_auto_restart_after_reopen',passed=True,witness=witness))
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
            circle=[x+1.5,y+args.obstacle_offset,.28]
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
            # The actual action must finish with the requested yaw before the
            # receptionist marks arrival; this is not just a direct plugin call.
            before=status()['pose'];pose=before[2]
            orient=point('oriented',before[0],before[1],'visit');orient['yaw']=math.remainder(pose+1.,2*math.pi)
            doc['points']=[orient];doc['routes']=[dict(id='tour',name='directed arrival',mode='showcase',loops=1,description='',steps=[dict(point_id='oriented',speak=False)])];doc=save(doc);start(doc);d=complete()
            actual_yaw=d['pose'][2];yaw_error=abs(math.remainder(actual_yaw-orient['yaw'],2*math.pi))
            assert yaw_error<.15,(actual_yaw,orient['yaw'])
            events.append({'case':'actual_directed_goal_settled_action_completion','passed':True,'requested_yaw':orient['yaw'],'actual_yaw':actual_yaw,'error_rad':yaw_error})
            doc['points']=[point('finish',10.5,4.25,'visit')];doc['routes'][0]['steps']=[dict(point_id='finish',speak=False)];doc=save(doc)
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
        if args.scenario=='tools':
            ros=ScenarioROS()
            doc=blank('native_showroom');doc['points']=[point('end',5.2,2.5,'visit')]
            doc['routes']=[dict(id='tour',name='annotated detour',mode='showcase',loops=1,description='',steps=[dict(point_id='end',speak=False)])]
            doc['annotations']=[dict(id='line',name='authored line',kind='keepout',vertices=[[3.5,2.0],[3.5,3.0]],width=.10),dict(id='slow',name='slow zone',kind='speed',vertices=[[1.,1.],[3.,1.],[3.,4.],[1.,4.]],factor=.5)]
            doc=save(doc)
            def lethal(grid,x,y):
                if grid is None:return False
                m=grid.metadata;ix=math.floor((x-m.origin.position.x)/m.resolution);iy=math.floor((y-m.origin.position.y)/m.resolution)
                return 0<=ix<m.size_x and 0<=iy<m.size_y and grid.data[iy*m.size_x+ix]>=254
            d=wait(lambda d:lethal(ros.local_map,3.5,2.5)and lethal(ros.global_map,3.5,2.5)and d.get('speed_zones',{}).get('factor')==.5,15)
            assert d['reception']['door_overlay']['keepout_cells']>0
            witness={'local_lethal':True,'global_lethal':True,'actual_speed_factor':d['speed_zones'],'source_scan_ns':d['scan_overlay']['source_ns']}
            start(doc);d=complete()
            cross=[t['truth']['pose']for t in trace if abs(t['truth']['pose'][0]-3.5)<.08]
            assert cross and all(p[1]<1.7 or p[1]>3.3 for p in cross),cross
            wait(lambda d:d['speed_zones']['factor']==1.,5)
            run=d['reception']['mission']['run_id'];history=call('/api/save',{'reception':dict(operation='trace_read',map_id=doc['map_id'],run_id=run,limit=200)})
            assert len(history['samples'])>5 and all(s.get('pose_source_ros_ns')and s['gate'].get('evaluated_monotonic_ns')for s in history['samples'])
            assert any(s['speed_zones']['factor']==.5 for s in history['samples'])
            doc['annotations']=[];doc=save(doc);wait(lambda d:d['reception']['door_overlay']['keepout_cells']==0,5)
            events.append(dict(case='keepout_both_costmaps_speed_entry_exit_and_aligned_records',passed=True,witness=witness,recorded_samples=len(history['samples']),crossings=cross))
        if args.scenario in ('tools','maps'):
            call('/api/start',dict(mode='mapping',name='resumable_map',controller='rpp'))
            wait(lambda d:d['mode']=='mapping'and d.get('pose')and d.get('scan_overlay')and d['gate'].get('data_fresh'),35)
            time.sleep(2.)
            saved=call('/api/save',{'reception':dict(operation='map_save',map_id='resumable_map')})
            assert all((runtime/'maps/resumable_map'/f).is_file()for f in ('mapped.data','mapped.posegraph'))
            resumed=call('/api/save',{'reception':dict(operation='map_resume',map_id='resumable_map',region=[[0.,0.],[2.,0.],[2.,2.],[0.,2.]])})
            wait(lambda d:d['mode']=='mapping'and d.get('pose')and d['gate'].get('data_fresh'),35);time.sleep(2.)
            updated=call('/api/save',{'reception':dict(operation='map_save',map_id='resumable_map')})
            assert updated['local_update']['outside_region_changed']==0
            versions=call('/api/save',{'reception':dict(operation='map_versions',map_id='resumable_map')})
            assert len(versions['versions'])>=3
            events.append(dict(case='actual_slam_save_resume_local_patch_versions',passed=True,first_save=saved,local_update=updated,versions=versions))
            d=status()
        report={'passed':True,'cases':events,'final_pose':d['pose'],'truth':d['truth'],'elapsed_s':time.monotonic()-started,
                'stop_profile':args.stop_profile,'obstacle_offset':args.obstacle_offset,'fixture_seed':0,'amcl_seed':'Nav2 default; stochastic localization is not forced to match one successful run','hardware_commands':0,'interfaces':os.listdir('/sys/class/net'),'source_sha':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in [NAV/'console.py',NAV/'reception_mission.py',NAV/'reception_doors.py',NAV/'sim_world.py']},
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
        if presentation:presentation.stop()
        log.close();report['curved_forward_receipts']=sum('Curved forward input blended'in line for line in(runtime/'workflow.log').read_text(errors='replace').splitlines())if(runtime/'workflow.log').is_file()else None;Path(args.output).write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in report.items()if k not in ('latest','source_sha','truth')},ensure_ascii=False))
    return 0 if report.get('passed')else 1
if __name__=='__main__':sys.exit(main())
