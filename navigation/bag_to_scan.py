#!/usr/bin/env python3
"""Read CLOSED, uncompressed Livox bags and derive motion-compensated virtual scans.

Uses recorded FAST-LIO poses, not a fresh LIO solution or independent ground truth.
Original bag files are opened immutable/read-only and are never rewritten.
"""
import argparse,json,sqlite3,struct,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg
from terrain import local_floor,classify

POINT=np.dtype({'names':['offset','x','y','z','reflectivity','tag','line'],
    'formats':['<u4','<f4','<f4','<f4','u1','u1','u1'],'offsets':[0,4,8,12,16,17,18],'itemsize':20})

def custom_points(raw):
    if raw[:2]!=b'\0\1':raise ValueError('Only little-endian CDR supported')
    sec,nsec,length=struct.unpack_from('<iII',raw,4)
    offset=16+length;offset=4+((offset-4+7)//8)*8
    timebase,point_num=struct.unpack_from('<QI',raw,offset);offset+=16
    count=struct.unpack_from('<I',raw,offset)[0];offset+=4
    if count!=point_num or len(raw)-offset not in (count*20,count*20-1):raise ValueError('Unexpected CustomMsg layout')
    buffer=raw if len(raw)-offset==count*20 else raw+b'\0'
    return sec+nsec*1e-9,np.frombuffer(buffer,dtype=POINT,count=count,offset=offset)

def records(folder,topic):
    for db in sorted(Path(folder).glob('*.db3')):
        with sqlite3.connect(db.resolve().as_uri()+'?mode=ro&immutable=1',uri=True) as con:
            row=con.execute('SELECT id FROM topics WHERE name=?',(topic,)).fetchone()
            if row:
                for timestamp,blob in con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',(row[0],)):
                    yield timestamp,blob

def main():
    p=argparse.ArgumentParser();p.add_argument('--bag',required=True);p.add_argument('--alignment',required=True);p.add_argument('--output',required=True)
    p.add_argument('--every',type=int,default=1);p.add_argument('--max-points',type=int,default=18000);args=p.parse_args()
    folder=Path(args.bag)
    if not (folder/'metadata.yaml').is_file() or list(folder.glob('*.zstd')):raise ValueError('Need finalized uncompressed SQLite bag')
    alignment=json.loads(Path(args.alignment).read_text());level=np.asarray(alignment['rotation']);height=float(alignment['initial_height_m'])
    rows=[]
    for _,raw in records(folder,'/Odometry'):
        msg=deserialize_message(raw,Odometry);q=msg.pose.pose.orientation;v=msg.pose.pose.position
        rows.append([msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,v.x,v.y,v.z,q.x,q.y,q.z,q.w])
    poses=np.asarray(rows);_,indices=np.unique(poses[:,0],return_index=True);poses=poses[np.sort(indices)]
    times=poses[:,0];slerp=Slerp(times,Rotation.from_quat(poses[:,4:8]));offset=np.array([-.011,-.02329,.04412])
    scans=[];states=[];stamps=[];quality=[];bad_ground=0;verified=0;clock0=time.monotonic();input_frames=0
    for frame,(_,raw) in enumerate(records(folder,'/livox/lidar')):
        input_frames+=1
        if frame%args.every:continue
        stamp,points=custom_points(raw)
        if verified<3:
            official=deserialize_message(raw,CustomMsg)
            assert len(points)==len(official.points)
            for i in (0,len(points)//2,len(points)-1):
                ref=official.points[i];a=points[i]
                assert np.allclose([a['x'],a['y'],a['z']],[ref.x,ref.y,ref.z],atol=1e-7)
                assert int(a['offset'])==ref.offset_time
            verified+=1
        stride=max(1,int(np.ceil(len(points)/args.max_points)));points=points[::stride]
        xyz=np.column_stack([points[k] for k in ('x','y','z')]).astype(float)
        finite=np.isfinite(xyz).all(axis=1)&(np.linalg.norm(xyz,axis=1)>.45)&(np.linalg.norm(xyz,axis=1)<20.)
        xyz=xyz[finite];dt=points['offset'][finite].astype(float)*1e-9
        pt_time=np.clip(stamp+dt,times[0],times[-1]);rot=slerp(pt_time)
        pos=np.column_stack([np.interp(pt_time,times,poses[:,i]) for i in (1,2,3)])
        world=(rot.apply(xyz+offset)+pos)@level.T;world[:,2]+=height
        middle=float(np.clip(stamp+.05,times[0],times[-1]));r=slerp([middle])[0]
        position=np.array([np.interp(middle,times,poses[:,i]) for i in (1,2,3)])
        optical=level@(position+r.apply(offset));orientation=level@r.as_matrix()@np.diag([1.,-1.,-1.])
        yaw=np.arctan2(orientation[1,0],orientation[0,0]);delta=world[:,:2]-optical[:2]
        out=np.full(360,np.inf)
        try:
            plane,report=local_floor(world,optical[:2],expected_height=float(optical[2]))
            _,mask=classify(world,optical[:2],plane)
            quality.append([report['slope_deg'],report['rmse_m'],report['inlier_cells']])
        except ValueError:
            # Keep a missing scan as missing. Never substitute ground with empty/free rays.
            mask=np.zeros(len(world),dtype=bool);bad_ground+=1;quality.append([np.nan,np.nan,0])
        angle=(np.arctan2(delta[mask,1],delta[mask,0])-yaw+np.pi)%(2*np.pi)-np.pi
        bins=np.clip(((angle+np.pi)/(2*np.pi)*360).astype(int),0,359)
        np.minimum.at(out,bins,np.linalg.norm(delta[mask],axis=1));out[~np.isfinite(out)]=np.nan
        scans.append(out.astype(np.float32));states.append([optical[0],optical[1],yaw]);stamps.append(middle)
        if len(scans)%500==0:print(json.dumps({'exported_scans':len(scans),'ground_fit_failures':bad_ground,'wall_s':time.monotonic()-clock0}),flush=True)
    target=Path(args.output);target.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(target,stamps=stamps,poses=states,ranges=scans,ground_quality=quality)
    report={'input_frames':input_frames,'exported_scans':len(scans),'raw_cdr_frames_crosschecked_with_official_deserializer':verified,
        'every':args.every,'max_points_per_scan':args.max_points,'ground_fit_failures':bad_ground,'duration_s':stamps[-1]-stamps[0],
        'wall_s':time.monotonic()-clock0,'original_bags_modified':False,
        'scope':'Derived 2D height-slice scans; point motion compensated with recorded LIO pose interpolation. Not a fresh LIO run or independent truth.'}
    target.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)

if __name__=='__main__':main()
