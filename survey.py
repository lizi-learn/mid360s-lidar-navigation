"""Manual trolley survey evidence. No command output or estimator fusion."""
import csv,json,math,os,pathlib,shutil,signal,subprocess,threading,time,uuid
from collections import deque
import numpy as np
from scipy.spatial import cKDTree

def wrap(a):return math.atan2(math.sin(a),math.cos(a))

def distance_step(anchor,xy,minimum=.02):
    d=math.hypot(xy[0]-anchor[0],xy[1]-anchor[1])
    return (d,list(xy)) if d>=minimum else (0.,anchor)

def shifted_grid(points,res=.1):
    if not len(points):return np.array([0.,0.]),np.full((1,1),205,dtype=np.uint8)
    cells=np.floor(points[:,:2]/res).astype(int);lo=cells.min(axis=0)-5;hi=cells.max(axis=0)+5
    size=hi-lo+1
    if np.max(size)>2500:raise ValueError('累计地图超过250m导出范围；3D点云和轨迹仍保留')
    image=np.full((int(size[1]),int(size[0])),205,dtype=np.uint8)
    floor=np.abs(points[:,2])<.09;obstacle=(points[:,2]>.15)&(points[:,2]<2.1)
    for mask,value in [(floor,254),(obstacle,0)]:
        c=cells[mask]-lo;image[c[:,1],c[:,0]]=value
    return lo*res,image[::-1]

class Survey:
    def __init__(self,root,runtime=None):
        self.root=pathlib.Path(root);self.runtime=pathlib.Path(runtime) if runtime else self.root/'runtime';self.lock=threading.RLock();self.active=False;self.latest_pose=None
        self.session=None;self.proc=None;self.files=[];self.last_result=None;self.history=deque(maxlen=100)
        self.imu_recent={'lidar':deque(maxlen=400),'body':deque(maxlen=400)};self.imu_seen={'lidar':0,'body':0}
        self.cloud_last=0;self.voxels={};self.initial_cloud=None;self.current_cloud=None;self.view_cache={};self.cache_time=0
        self.saving=False;self.epoch=time.time();self.reason='尚未开始推架记录';self.samples=[]

    def imu(self,kind,msg):
        now=time.monotonic();a=msg.linear_acceleration;g=msg.angular_velocity
        scale=9.80665 if kind=="lidar" else 1.0
        v=[a.x*scale,a.y*scale,a.z*scale,g.x,g.y,g.z]
        if not np.isfinite(v).all():return
        with self.lock:
            self.imu_recent[kind].append((now,v));self.imu_seen[kind]+=1
            if self.active:self.writers[kind].writerow([time.time(),msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,*v])

    def pose(self,xy,z,yaw,pitch,stamp):
        now=time.monotonic();p={'t':now,'wall_time':time.time(),'x':float(xy[0]),'y':float(xy[1]),'z':float(z),'yaw':float(yaw),'pitch':float(pitch),'stamp':stamp}
        with self.lock:
            self.latest_pose=p;self.history.append(p)
            if not self.active:return
            if self.last_pose:
                dt=now-self.last_pose['t'];stamp_dt=stamp-self.last_pose['stamp'];d=math.hypot(p['x']-self.last_pose['x'],p['y']-self.last_pose['y'])
                if dt>1:self.issues.add('定位数据间断超过1秒')
                if stamp_dt<0:self.issues.add('定位时间戳回退，本段坐标需核查')
                if d/max(.02,stamp_dt)>1.5:self.issues.add('定位跳变或推行速度过高')
                increment,self.distance_anchor=distance_step(self.distance_anchor,[p["x"],p["y"]]);self.distance+=increment
            self.last_pose=p;self.trajectory.append([p['x'],p['y']]);self.writers['trajectory'].writerow([p['wall_time'],stamp,p['x'],p['y'],p['z'],yaw,pitch]);self.samples.append(p)

    def cloud(self,p):
        now=time.monotonic()
        with self.lock:
            if not self.active or now-self.cloud_last<.7:return
            self.cloud_last=now
            # Preserve registered scans and apply no loop-closure correction.
            finite=p[np.isfinite(p).all(axis=1)]
            if not len(finite):return
            keys=np.floor(finite/.15).astype(np.int64);unique,idx=np.unique(keys,axis=0,return_index=True)
            scan=finite[idx]
            self.current_cloud=scan
            if self.initial_cloud is None:self.initial_cloud=scan.copy()
            if len(self.voxels)<800000:
                for key,point in zip(map(tuple,unique),scan):
                    old=self.voxels.get(key)
                    if old is None:self.voxels[key]=[point.copy(),1]
                    else:old[0]+=(point-old[0])/min(old[1]+1,50);old[1]+=1
            else:self.issues.add('累计点云达到内存上限，需结束本段后另开一段')

    def start(self,health):
        with self.lock:
            if self.active or self.saving:raise ValueError('已有记录正在运行或保存')
            if not self.latest_pose or time.monotonic()-self.latest_pose['t']>.5 or health.get('map_age',10)>.7 or health.get('imu_age',10)>.2:raise ValueError('等待实时定位和雷达IMU稳定后再开始')
            poses=[p for p in self.history if time.monotonic()-p['t']<5]
            if len(poses)<30 or np.ptp([p['x'] for p in poses])>.05 or np.ptp([p['y'] for p in poses])>.05:raise ValueError('请在地面起点标记处静止5秒，待定位稳定后再开始')
            if shutil.disk_usage(self.runtime).free<5*1024**3:raise ValueError('剩余磁盘不足5GB，不能开始录包')
            self.session=time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6];self.folder=self.runtime/'surveys'/self.session;self.folder.mkdir(parents=True)
            self.voxels={};self.initial_cloud=None;self.current_cloud=None;self.view_cache={};self.cache_time=0;self.cloud_last=0
            self.distance=0.;self.start_time=time.monotonic();self.start_pose=dict(self.latest_pose);self.start_pose.update({k:float(np.mean([p[k] for p in poses[-10:]])) for k in ['x','y','z']});self.distance_anchor=[self.start_pose["x"],self.start_pose["y"]];self.last_pose=None;self.trajectory=[];self.samples=[];self.issues=set();self.returns=[]
            self.files=[];self.writers={}
            for k,header in [('trajectory',['host_time_s','lidar_stamp_s','x_m','y_m','z_m','yaw_rad','pitch_rad']),('lidar',['host_time_s','stamp_s','ax_m_s2','ay_m_s2','az_m_s2','wx_rad_s','wy_rad_s','wz_rad_s']),('body',['host_time_s','stamp_s','ax_m_s2','ay_m_s2','az_m_s2','wx_rad_s','wy_rad_s','wz_rad_s'])]:
                f=(self.folder/(k+'.csv')).open('w',buffering=65536);self.files.append(f);self.writers[k]=csv.writer(f);self.writers[k].writerow(header)
            for x in ['config/robot.local.json','config/mid360s_lio.yaml','config/MID360S.json','config/recording-qos.yaml','runtime/map_alignment.json','runtime/neck_status.json']:
                path=self.runtime/x.removeprefix('runtime/') if x.startswith('runtime/') else self.root/x
                if path.exists():shutil.copy2(path,self.folder/path.name)
            self.baglog=(self.folder/'rosbag.log').open('w')
            # Preserve directly readable originals during acquisition. A saved
            # stationary bag had a corrupt zstd split despite a clean exit.
            args=['ros2','bag','record','--storage','sqlite3','--output',str(self.folder/'raw_bag'),'--max-bag-size','268435456','--max-cache-size','67108864','--qos-profile-overrides-path',str(self.root/'config/recording-qos.yaml'),'/livox/lidar','/livox/imu','/Odometry','/p2_nav/body_imu','/p2_nav/body_imu_meta','/tf_static']
            try:self.proc=subprocess.Popen(args,stdout=self.baglog,stderr=subprocess.STDOUT,start_new_session=True)
            except Exception:
                for f in self.files:f.close()
                self.baglog.close();raise
            self.required_topics=['/livox/lidar','/livox/imu','/Odometry']+(['/p2_nav/body_imu','/p2_nav/body_imu_meta'] if self.imu_recent['body'] and time.monotonic()-self.imu_recent['body'][-1][0]<.3 else []);self.active=True;self.reason='记录准备中，请先静止几秒';self.persist({'state':'recording','start_pose':self.start_pose,'started_wall':time.time(),'execution_enabled':False,'max_duration_s':1200,'bag_topics':args[-6:],'storage_id':'sqlite3','compression_mode':'none'})
            return {'session':self.session,'active':True,'message':'等待页面显示原始录包就绪，再慢速推架'}

    def persist(self,value):
        (self.folder/'session.json').write_text(json.dumps(value,ensure_ascii=False,indent=2))

    def mark_return(self):
        with self.lock:
            if not self.active:raise ValueError('请先开始推架记录')
            now=time.monotonic();poses=[p for p in self.history if now-p['t']<1.5]
            if len(poses)<5:raise ValueError('定位样本不足')
            if np.ptp([p['x'] for p in poses])>.05 or np.ptp([p['y'] for p in poses])>.05:raise ValueError('请回到地面标记点并静止2秒，再确认朝向一致')
            p=dict(self.latest_pose)
            position=math.hypot(p['x']-self.start_pose['x'],p['y']-self.start_pose['y']);yaw=abs(math.degrees(wrap(p['yaw']-self.start_pose['yaw'])));height=p['z']-self.start_pose['z']
            result={'wall_time':time.time(),'physical_return_and_heading_confirmed_by_operator':True,'position_discrepancy_m':position,'heading_discrepancy_deg':yaw,'height_discrepancy_m':height,'travelled_m':self.distance,'closed_loop_fraction':position/self.distance if self.distance>3 else None,'note':'仅在实物回到同一标记点、朝向一致时，才可解释为闭合误差；不是独立真值定位。'}
            if self.initial_cloud is not None and self.current_cloud is not None:
                origin=np.array([self.start_pose['x'],self.start_pose['y']])
                def region(p):return p[(np.linalg.norm(p[:,:2]-origin,axis=1)<7)&(p[:,2]>.25)&(p[:,2]<2.4)]
                a,b=region(self.initial_cloud),region(self.current_cloud)
                if len(a)>100 and len(b)>100:
                    d=cKDTree(a).query(b,k=1)[0];result['overlap_cloud_check']={'points':len(b),'median_nn_m':float(np.median(d)),'p90_nn_m':float(np.quantile(d,.9)),'fraction_within_15cm':float(np.mean(d<.15)),'note':'注册点云重叠一致性；未做ICP重新对齐，也不等于绝对精度'}
            self.returns.append(result);(self.folder/'returns.json').write_text(json.dumps(self.returns,ensure_ascii=False,indent=2));self.last_result=result;return result

    def stop(self,reason='操作者结束并保存'):
        with self.lock:
            if not self.active:return {'active':False,'session':self.session,'message':self.reason}
            self.active=False;self.saving=True;self.reason=reason
        # Release the sensor lock before waiting for recorder flush/compression.
        if self.proc and self.proc.poll() is None:
            os.killpg(self.proc.pid,signal.SIGINT)
            try:self.proc.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid,signal.SIGTERM)
                try:self.proc.wait(timeout=2)
                except subprocess.TimeoutExpired:self.proc.kill();self.proc.wait()
                self.issues.add('原始录包退出超时，需检查metadata及完整性')
        for f in self.files:f.close()
        self.baglog.close();points=self.accumulated_points()
        header=f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n'
        with (self.folder/'map.pcd').open('wb') as f:f.write(header.encode());f.write(points.astype('<f4').tobytes())
        try:
            origin,pgm=shifted_grid(points)
            with (self.folder/'map.pgm').open('wb') as f:f.write(f'P5\n{pgm.shape[1]} {pgm.shape[0]}\n255\n'.encode());f.write(pgm.tobytes())
            (self.folder/'map.yaml').write_text(f'image: map.pgm\nresolution: 0.1\norigin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        except ValueError as e:self.issues.add(str(e))
        result={'session':self.session,'state':'saved','reason':reason,'duration_s':time.monotonic()-self.start_time,'travelled_m':self.distance,'map_voxels':len(points),'return_checks':self.returns,'issues':sorted(self.issues),'start_pose':self.start_pose,'last_pose':self.latest_pose,'bag_exit_code':self.proc.returncode if self.proc else None,'raw_bag_metadata_present':(self.folder/'raw_bag/metadata.yaml').exists(),'execution_enabled':False,'map_acceptance':'待移动闭合测试、地图几何检查与重定位测试，当前导出图未获导航验收','body_imu_use':'independent comparison only; no fusion; extrinsic/time offset uncalibrated','imu_csv_units':'acceleration m/s^2, gyro rad/s; Livox native acceleration g multiplied by9.80665 in CSV only; original bag unchanged','artifacts_directory':str(self.folder)}
        self.persist(result);self.last_saved=result;self.saving=False;return result
    def accumulated_points(self):return np.array([v[0] for v in self.voxels.values() if v[1]>=2],dtype=float).reshape(-1,3)

    def status(self):
        with self.lock:
            now=time.monotonic();imu={}
            for k,values in self.imu_recent.items():
                recent=[v for t,v in values if now-t<2]
                imu[k]={'age_s':now-values[-1][0] if values else None,'recent_samples':len(recent),'gyro_plot':[[round(t-now,3),float(np.linalg.norm(v[3:])*180/math.pi)] for t,v in list(values)[::5] if now-t<2]}
                if recent:
                    a=np.array(recent);imu[k].update({'acc_norm_mean':float(np.linalg.norm(a[:,:3],axis=1).mean()),'gyro_norm_mean_rad_s':float(np.linalg.norm(a[:,3:],axis=1).mean()),'gyro_mean_rad_s':a[:,3:].mean(axis=0).tolist(),'gyro_std_rad_s':a[:,3:].std(axis=0).tolist()})
            ready=False
            if self.active:
                text=(self.folder/'rosbag.log').read_text()[-12000:]
                ready=all("Subscribed to topic '"+topic+"'" in text for topic in self.required_topics) and self.proc.poll() is None
                self.reason='原始录包就绪 · 可以慢速推架' if ready else '原始录包准备中，请先静止'
                if self.proc.poll() is not None:self.issues.add('原始录包进程已退出，请结束本段检查')
            if self.session and now-self.cache_time>1:
                points=self.accumulated_points();self.cache_time=now
                def xy(mask):
                    a=points[mask,:2];a=np.unique(np.floor(a/.15).astype(int),axis=0)*.15+.075
                    return np.round(a[::max(1,len(a)//25000)],3).tolist()
                self.view_cache={'floor_xy':xy(np.abs(points[:,2])<.09),'obstacle_xy':xy((points[:,2]>.15)&(points[:,2]<2.1)),'voxel_count':len(points)}
            delta=None
            if self.session and self.latest_pose:
                p=self.latest_pose;delta={'position_m':math.hypot(p['x']-self.start_pose['x'],p['y']-self.start_pose['y']),'heading_deg':math.degrees(wrap(p['yaw']-self.start_pose['yaw'])),'height_m':p['z']-self.start_pose['z']}
            return {'active':self.active,'session':self.session,'state':self.reason,'bag_ready':ready,'duration_s':now-self.start_time if self.active else getattr(self,'last_saved',{}).get('duration_s',0),'travelled_m':self.distance if self.session else 0,'relative_to_start':delta,'imu':imu,'return_check':self.last_result,'issues':sorted(self.issues) if self.session else [],'trajectory':self.trajectory[::max(1,len(self.trajectory)//2500)] if self.session else [],'global_map':self.view_cache,'execution_enabled':False,'last_saved':getattr(self,'last_saved',None)}

    def check_limits(self):
        reason=None
        with self.lock:
            if not self.active:return
            if self.latest_pose and time.monotonic()-self.latest_pose['t']>1:self.issues.add('定位数据间断超过1秒')
            if time.monotonic()-self.start_time>1200:reason='单段达到20分钟，自动保存'
            elif shutil.disk_usage(self.runtime).free<3*1024**3:reason='磁盘余量低于3GB，自动保存'
            elif self.proc.poll() is not None:reason='原始录包进程退出，自动保存现有数据'
        if reason:self.stop(reason)
