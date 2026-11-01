"""Bounded offline PCD-to-private-scene import; never changes a navigation map."""
from pathlib import Path
import math
import numpy as np
from reception_catalog import ident

MAX_FILE = 256 * 1024 * 1024

def read_xyz(path):
    path=Path(path)
    if path.stat().st_size>MAX_FILE:raise ValueError('PCD文件超过256MB，请先离线裁剪')
    header={}
    with path.open('rb')as stream:
        for _ in range(100):
            line=stream.readline(4096)
            if not line or len(line)>=4096:raise ValueError('PCD头部无效')
            line=line.decode('ascii').strip()
            if not line or line.startswith('#'):continue
            key,*value=line.split();header[key.upper()]=value
            if key.upper()=='DATA':break
        else:raise ValueError('PCD头部过长')
        fields=header.get('FIELDS',[])
        try:
            sizes=list(map(int,header['SIZE']));types=header['TYPE'];counts=list(map(int,header.get('COUNT',['1']*len(fields))))
            total=int(header.get('POINTS',[str(int(header['WIDTH'][0])*int(header['HEIGHT'][0]))])[0])
            mode=header['DATA'][0]
        except (KeyError,ValueError,IndexError):raise ValueError('PCD字段定义缺失')
        if not 0<total<=3000000 or len(set(fields))!=len(fields)or not len(fields)==len(sizes)==len(types)==len(counts):raise ValueError('PCD字段数量或点数无效')
        if any(k not in fields or counts[fields.index(k)]!=1 for k in ('x','y','z')):raise ValueError('PCD需要单值x、y、z字段')
        if any(not 1<=n<=64 for n in counts):raise ValueError('PCD字段长度无效')
        formats=[]
        for size,kind,count in zip(sizes,types,counts):
            if (kind,size)not in {('F',4),('F',8),('I',1),('I',2),('I',4),('I',8),('U',1),('U',2),('U',4),('U',8)}:raise ValueError('PCD字段类型不受支持')
            code='<'+{'F':'f','I':'i','U':'u'}[kind]+str(size);formats.append(code if count==1 else (code,(count,)))
        if mode=='binary':
            dtype=np.dtype(dict(names=fields,formats=formats))
            expected=total*dtype.itemsize
            if expected>MAX_FILE:raise ValueError('PCD展开数据过大')
            raw=stream.read(expected+1)
            if len(raw)!=expected:raise ValueError('PCD二进制长度与头部不符')
            values=np.frombuffer(raw,dtype=dtype);points=np.column_stack([values[k]for k in ('x','y','z')]).astype(float)
        elif mode=='ascii':
            offsets=np.cumsum([0]+counts[:-1]).tolist();columns=[offsets[fields.index(k)]for k in ('x','y','z')]
            try:points=np.loadtxt(stream,usecols=columns,ndmin=2,dtype=float,max_rows=total+1)
            except ValueError:raise ValueError('PCD文本坐标无效')
            if points.shape!=(total,3):raise ValueError('PCD文本点数与头部不符')
        elif mode=='binary_compressed':raise ValueError('请将PCD另存为binary或ascii格式；本离线导入器暂不解压binary_compressed')
        else:raise ValueError('PCD DATA格式不受支持')
    return points

def convert(path,map_id,frame_id,transform=None,maximum=20000):
    ident(map_id)
    if type(maximum)is not int or not 1<=maximum<=20000:raise ValueError('预览最多20000点')
    if not isinstance(frame_id,str)or not frame_id:raise ValueError('请指定原点云坐标系')
    if frame_id!='map'and transform is None:raise ValueError('原点云不在map坐标系，请提供实际map变换矩阵')
    matrix=np.eye(4)if transform is None else np.asarray(transform,dtype=float)
    if matrix.shape!=(4,4)or not np.isfinite(matrix).all()or not np.allclose(matrix[3],[0,0,0,1],atol=1e-6):raise ValueError('map变换矩阵无效')
    rotation=matrix[:3,:3]
    if not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-5)or not math.isclose(float(np.linalg.det(rotation)),1.,abs_tol=1e-5):raise ValueError('map变换必须是米制刚体变换')
    points=read_xyz(path);input_count=len(points);points=points[np.isfinite(points).all(axis=1)]
    finite_count=len(points)
    if not finite_count:raise ValueError('点云没有有限坐标')
    # Deterministic actual-sample selection: no fabricated fill or interpolation.
    if len(points)>maximum:points=points[np.linspace(0,len(points)-1,maximum,dtype=int)]
    points=points@rotation.T+matrix[:3,3]
    if np.abs(points).max()>100000:raise ValueError('点云坐标超出场地预览范围')
    asset=dict(map_id=map_id,frame_id='map',units='m',points=np.round(points,3).tolist())
    return asset,dict(input_points=input_count,finite_points=finite_count,preview_points=len(points),
        source_frame=frame_id,transform_applied=transform is not None,scope='Private scene preview only; navigation occupancy and localization unchanged')
