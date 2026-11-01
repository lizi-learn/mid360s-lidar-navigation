"""Private map versions, bounded local raster updates and portable site bundles.

Bundles contain site assets, never motor configuration, calibration, credentials
or a previous robot's live localization state. Import always creates a new site.
"""
import base64
import hashlib
import io
import json
import math
import os
from pathlib import Path
import secrets
import shutil
import threading
import time
import uuid
import zipfile
import numpy as np
import yaml
from audio_feedback import atomic_json
from map_persistence import validate_map_metadata
from reception_catalog import ident, validate
from reception_annotations import inside

FILES=('mapped.yaml','mapped.pgm','mapped.data','mapped.posegraph')
MAX_BYTES=64*1024*1024


def sha(data):return hashlib.sha256(data).hexdigest()


def read_pgm(path):
    # P5/P2 input accepted; output is normalized P5. No Pillow/ROS dependency.
    data=Path(path).read_bytes();stream=io.BytesIO(data)
    def token():
        result=b''
        while True:
            c=stream.read(1)
            if not c:return result
            if c==b'#':stream.readline();continue
            if c.isspace():
                if result:
                    if c==b'\r':
                        following=stream.read(1)
                        if following!=b'\n':stream.seek(-len(following),1)
                    return result
            else:result+=c
    magic=token();w,h,maximum=(int(token())for _ in range(3))
    if magic not in (b'P2',b'P5')or not 0<w*h<=16_000_000 or maximum!=255:raise ValueError('地图PGM格式不受支持')
    if magic==b'P5':pixels=np.frombuffer(stream.read(w*h),dtype=np.uint8).copy()
    else:
        values=[int(token())for _ in range(w*h)]
        if any(not 0<=v<=255 for v in values):raise ValueError('PGM像素超出0到255')
        pixels=np.asarray(values,dtype=np.uint8)
    if len(pixels)!=w*h:raise ValueError('地图图像截断')
    return pixels.reshape(h,w)


def local_patch(base_dir,scan_dir,vertices):
    """Only observed cells in the selected same-frame region replace the raster."""
    if not isinstance(vertices,list)or not 3<=len(vertices)<=64:raise ValueError('局部更新区至少三个顶点')
    if any(not isinstance(p,list)or len(p)!=2 or any(type(v)not in(int,float)or not math.isfinite(v)for v in p)for p in vertices):raise ValueError('局部更新坐标无效')
    old=validate_map_metadata(Path(base_dir)/'mapped.yaml');new=validate_map_metadata(Path(scan_dir)/'mapped.yaml')
    if abs(old['resolution']-new['resolution'])>1e-8 or abs(old['origin'][2])+abs(new['origin'][2])>1e-8:raise ValueError('局部更新需要相同分辨率、未旋转的地图坐标')
    before=read_pgm(Path(base_dir)/'mapped.pgm');after=read_pgm(Path(scan_dir)/'mapped.pgm');res=old['resolution'];h,w=before.shape
    yy,xx=np.mgrid[:h,:w];wx=old['origin'][0]+(xx+.5)*res;wy=old['origin'][1]+(h-yy-.5)*res
    mask=inside(wx,wy,vertices)
    nx=np.floor((wx-new['origin'][0])/res).astype(int);ny=after.shape[0]-1-np.floor((wy-new['origin'][1])/res).astype(int)
    valid=mask&(nx>=0)&(ny>=0)&(nx<after.shape[1])&(ny<after.shape[0]);values=np.full(before.shape,205,dtype=np.uint8);values[valid]=after[ny[valid],nx[valid]]
    # New unknown observations do not erase existing measured map cells.
    valid&=values!=205;output=before.copy();output[valid]=values[valid]
    (Path(scan_dir)/'mapped.pgm').write_bytes(f'P5\n{w} {h}\n255\n'.encode()+output.tobytes())
    old['image']='mapped.pgm';(Path(scan_dir)/'mapped.yaml').write_text(yaml.safe_dump(old))
    return {'observed_region_cells':int(valid.sum()),'changed_cells':int(np.count_nonzero(output!=before)),
            'outside_region_changed':int(np.count_nonzero(output[~mask]!=before[~mask])),
            'scope':'selected same-frame raster; serialized graph retains all resumed scans'}


class MapLibrary:
    def __init__(self,root,catalog):
        self.root=Path(root);self.catalog=catalog;self.lock=threading.RLock()
    def directory(self,name):return self.root/ident(name)
    def versions(self,name):
        directory=self.directory(name)/'versions';rows=[]
        for file in sorted(directory.glob('*/manifest.json'),reverse=True)[:30]:
            d=json.loads(file.read_text());rows.append({k:d[k]for k in ('id','wall_time','reason','files')})
        return rows
    def snapshot(self,name,reason='saved'):
        with self.lock:
            directory=self.directory(name);validate_map_metadata(directory/'mapped.yaml')
            key=time.strftime('%Y%m%dT%H%M%S')+'-'+secrets.token_hex(3);target=directory/'versions'/key;target.mkdir(parents=True)
            files={}
            for filename in FILES:
                source=directory/filename
                if source.is_file():
                    data=source.read_bytes();files[filename]={'sha256':sha(data),'bytes':len(data)};(target/filename).write_bytes(data)
            manifest=dict(id=key,wall_time=time.time(),reason=reason,files=files)
            atomic_json(target/'manifest.json',manifest)
            for old in sorted((directory/'versions').glob('*'))[:-30]:shutil.rmtree(old)
            return manifest
    def restore(self,name,version):
        with self.lock:
            ident(version);directory=self.directory(name);source=directory/'versions'/version
            manifest=json.loads((source/'manifest.json').read_text())
            assets={}
            for filename,entry in manifest['files'].items():
                if filename not in FILES:raise ValueError('地图版本文件名无效')
                assets[filename]=(source/filename).read_bytes()
                if sha(assets[filename])!=entry['sha256']:raise ValueError('地图版本校验失败')
            self.snapshot(name,'before_restore')
            for filename in FILES:
                if filename in assets:(directory/filename).write_bytes(assets[filename])
                else:(directory/filename).unlink(missing_ok=True)
            return self.snapshot(name,'restored_from_'+version)
    def export(self,name):
        with self.lock:
            directory=self.directory(name);validate_map_metadata(directory/'mapped.yaml')
            files={}
            for filename in FILES:
                if (directory/filename).is_file():files['map/'+filename]=(directory/filename).read_bytes()
            files['catalog.json']=json.dumps(self.catalog.read(name),ensure_ascii=False,allow_nan=False).encode()
            if not {'map/mapped.yaml','map/mapped.pgm'}<=set(files):raise ValueError('地图包缺少mapped.yaml或mapped.pgm')
            if sum(map(len,files.values()))>MAX_BYTES:raise ValueError('场地包展开后超过64MiB')
            manifest={'schema':1,'kind':'p2-navigation-site','map_id':name,'created_at':time.time(),
                      'bundle_id':str(uuid.uuid4()),'frame_id':'map','robot_calibration_included':False,
                      'files':{k:{'bytes':len(v),'sha256':sha(v)}for k,v in files.items()}}
            memory=io.BytesIO()
            with zipfile.ZipFile(memory,'w',zipfile.ZIP_DEFLATED)as archive:
                for key,data in files.items():archive.writestr(key,data)
                archive.writestr('manifest.json',json.dumps(manifest,ensure_ascii=False))
            data=memory.getvalue()
            if len(data)>MAX_BYTES:raise ValueError('场地包超过64MiB')
            return data,manifest
    def import_bundle(self,data,new_name):
        try:return self._import_bundle(data,new_name)
        except (zipfile.BadZipFile,EOFError,RuntimeError,UnicodeError,OSError)as error:
            raise ValueError('场地包无法解包或写入：'+type(error).__name__)from error
    def _import_bundle(self,data,new_name):
        ident(new_name)
        if len(new_name)>48 or len(data)>MAX_BYTES:raise ValueError('新地图名称或包大小无效')
        with self.lock:
            target=self.directory(new_name)
            if target.exists()or self.catalog.read(new_name)['revision']:raise ValueError('请使用新的地图名称，已有场地不覆盖')
            with zipfile.ZipFile(io.BytesIO(data))as archive:
                names=archive.namelist();allowed={'manifest.json','catalog.json',*('map/'+f for f in FILES)}
                if len(set(names))!=len(names)or set(names)-allowed or sum(i.file_size for i in archive.infolist())>MAX_BYTES:
                    raise ValueError('场地包含重复、未知或过大文件')
                if not {'manifest.json','catalog.json','map/mapped.yaml','map/mapped.pgm'}<=set(names):raise ValueError('场地包缺少地图或点位内容')
                if archive.getinfo('manifest.json').file_size>65536:raise ValueError('场地包清单过大')
                manifest=json.loads(archive.read('manifest.json'))
                if manifest.get('schema')!=1 or manifest.get('kind')!='p2-navigation-site' or manifest.get('frame_id')!='map':raise ValueError('场地包类型无效')
                if set(manifest['files'])!=set(names)-{'manifest.json'}:raise ValueError('场地包清单不完整')
                assets={}
                for name,entry in manifest['files'].items():
                    value=archive.read(name)
                    if len(value)!=entry['bytes']or sha(value)!=entry['sha256']:raise ValueError('场地包文件校验失败')
                    assets[name]=value
            document=validate(json.loads(assets['catalog.json']))
            if document['map_id']!=manifest['map_id']:raise ValueError('场地包地图与点位不匹配')
            document.update(map_id=new_name,revision=0)
            temporary=self.root/('.import-'+secrets.token_hex(8));temporary.mkdir(parents=True)
            try:
                for name,value in assets.items():
                    if name.startswith('map/'):(temporary/name[4:]).write_bytes(value)
                metadata=validate_map_metadata(temporary/'mapped.yaml')
                if metadata['image']!='mapped.pgm':raise ValueError('场地包地图只能引用内部mapped.pgm')
                read_pgm(temporary/'mapped.pgm')
                atomic_json(temporary/'site-origin.json',{'source_bundle':manifest['bundle_id'],'source_map_id':manifest['map_id'],
                    'new_installation_id':str(uuid.uuid4()),'imported_at':time.time(),'calibration_copied':False,'localization_copied':False})
                os.replace(temporary,target)
                try:self.catalog.save(document,0)
                except Exception:shutil.rmtree(target);raise
            finally:
                if temporary.exists():shutil.rmtree(temporary)
            return {'imported':new_name,'points':len(document['points']),'routes':len(document['routes']),
                    'can_resume_mapping':all((target/f).is_file()for f in ('mapped.data','mapped.posegraph'))}


class BundleTransfer:
    """G12's existing 8192-byte gateway can carry a complete map in chunks."""
    def __init__(self,library):self.library=library;self.lock=threading.Lock();self.sessions={}
    def request(self,data):
        with self.lock:
            now=time.monotonic();self.sessions={k:v for k,v in self.sessions.items()if now-v['at']<600}
            op=data['operation']
            if op=='bundle_export':
                if len(self.sessions)>=2:raise ValueError('已有两个场地传输，请结束后重试')
                raw,manifest=self.library.export(data['map_id']);key=secrets.token_hex(16)
                self.sessions[key]={'raw':raw,'at':now,'kind':'export'}
                return {'transfer_id':key,'bytes':len(raw),'sha256':sha(raw),'chunks':math.ceil(len(raw)/4096),'manifest':manifest}
            if op=='bundle_begin':
                size=data.get('bytes');checksum=data.get('sha256')
                if type(size)is not int or not 0<size<=MAX_BYTES or not isinstance(checksum,str)or len(checksum)!=64:raise ValueError('场地包长度或摘要无效')
                ident(data['new_map_id'])
                if len(self.sessions)>=2:raise ValueError('已有两个场地传输，请结束后重试')
                key=secrets.token_hex(16);self.sessions[key]={'parts':{},'at':now,'kind':'import','bytes':size,'sha256':checksum,'name':data['new_map_id']}
                return {'transfer_id':key,'chunks':math.ceil(size/4096)}
            key=data.get('transfer_id');s=self.sessions.get(key)
            if not s:raise ValueError('传输不存在或已过期')
            s['at']=now
            if op=='bundle_abort':del self.sessions[key];return {'aborted':True}
            index=data.get('index')
            if op in ('bundle_chunk','bundle_part')and(type(index)is not int or index<0):raise ValueError('分片序号无效')
            if op=='bundle_chunk'and s['kind']=='export':
                value=s['raw'][index*4096:(index+1)*4096]
                if not value:raise ValueError('分片超出范围')
                return {'index':index,'data':base64.b64encode(value).decode()}
            if op=='bundle_part'and s['kind']=='import':
                value=base64.b64decode(data['data'],validate=True);expected=min(4096,s['bytes']-index*4096)
                if expected<=0 or len(value)!=expected:raise ValueError('分片长度无效')
                if index in s['parts']and value!=s['parts'][index]:raise ValueError('重复分片内容不一致')
                s['parts'][index]=value;return {'received':len(s['parts'])}
            if op=='bundle_commit'and s['kind']=='import':
                count=math.ceil(s['bytes']/4096)
                if len(s['parts'])!=count:raise ValueError('场地包分片不完整')
                raw=b''.join(s['parts'][i]for i in range(count))
                if sha(raw)!=s['sha256']:raise ValueError('场地包总摘要不一致')
                result=self.library.import_bundle(raw,s['name']);del self.sessions[key];return result
            raise ValueError('场地传输操作无效')
