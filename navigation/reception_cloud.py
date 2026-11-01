"""Bounded private XYZ preview assets, independent of control and costmaps."""
import json
import math
import threading
from pathlib import Path
from audio_feedback import atomic_json
from reception_catalog import ident

class CloudStore:
    def __init__(self,root):self.root=Path(root);self.lock=threading.Lock()
    def read(self,map_id):
        path=self.root/(ident(map_id)+'.json')
        with self.lock:
            try:
                if path.stat().st_size>600000:raise ValueError('三维预览文件过大')
                data=self.validate(json.loads(path.read_text()))
                if data['map_id']!=map_id:raise ValueError('三维预览地图标识不一致')
                return data
            except FileNotFoundError:return {'map_id':map_id,'frame_id':'map','units':'m','points':[]}
    @staticmethod
    def validate(data):
        if not isinstance(data,dict)or set(data)!={'map_id','frame_id','units','points'}:raise ValueError('三维预览字段无效')
        ident(data['map_id'])
        if data['frame_id']!='map'or data['units']!='m':raise ValueError('三维预览必须使用当前map坐标和米单位')
        points=data['points']
        if not isinstance(points,list)or len(points)>20000:raise ValueError('三维预览最多20000个点')
        for p in points:
            if not isinstance(p,list)or len(p)!=3 or any(type(v)not in (int,float)or not math.isfinite(v)or abs(v)>100000 for v in p):raise ValueError('三维点坐标无效')
        if len(json.dumps(data).encode())>590000:raise ValueError('三维预览数据过大，请先降采样')
        return data
    def save(self,data):
        data=self.validate(data)
        with self.lock:atomic_json(self.root/(data['map_id']+'.json'),data)
        return {'saved':True,'points':len(data['points']),'map_id':data['map_id'],'scope':'preview_only'}
