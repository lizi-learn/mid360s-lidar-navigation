"""Small gateway-compatible content chunks, committed once after full validation.

Existing G12 proxy limits POSTs to 8192 bytes. This avoids restarting its shared
factory service. Partial uploads never replace active map content.
"""
import base64
import binascii
import json
import threading
import time
from reception_catalog import ident

class ContentTransfer:
    def __init__(self,catalog,cloud):
        self.catalog=catalog;self.cloud=cloud;self.lock=threading.Lock();self.uploads={}
    def receive(self,data):
        if not isinstance(data,dict)or set(data)!={'operation','upload_id','map_id','kind','revision','index','total','chunk'}:raise ValueError('内容分片字段无效')
        key=ident(data['upload_id']);map_id=ident(data['map_id']);kind=data['kind'];revision=data['revision'];index=data['index'];total=data['total']
        if kind not in ('catalog','cloud')or type(revision)is not int or revision<0 or type(total)is not int or not 1<=total<=160 or type(index)is not int or not 0<=index<total:raise ValueError('内容分片参数无效')
        if not isinstance(data['chunk'],str)or len(data['chunk'])>5500:raise ValueError('内容分片过大')
        try:chunk=base64.b64decode(data['chunk'],validate=True)
        except (ValueError,binascii.Error):raise ValueError('内容分片编码无效')
        if len(chunk)>4096:raise ValueError('内容分片过大')
        meta=(map_id,kind,revision,total);now=time.monotonic()
        with self.lock:
            self.uploads={k:v for k,v in self.uploads.items()if now-v['at']<120}
            row=self.uploads.get(key)
            if row is None:
                if len(self.uploads)>=4:raise ValueError('其他内容传输尚未完成，请稍后重试')
                row={'meta':meta,'at':now,'parts':{},'result':None};self.uploads[key]=row
            if row['meta']!=meta:raise ValueError('内容传输标识被重复用于不同内容')
            if index in row['parts'] and row['parts'][index]!=chunk:raise ValueError('重复分片内容不一致')
            row['at']=now;row['parts'][index]=chunk
            if row['result']is not None:return row['result']
            if sum(map(len,row['parts'].values()))>590000:
                del self.uploads[key];raise ValueError('内容包过大')
            if len(row['parts'])<total:return {'upload_id':key,'received':len(row['parts']),'total':total,'committed':False}
            try:
                value=json.loads(b''.join(row['parts'][i]for i in range(total)))
                if not isinstance(value,dict)or value.get('map_id')!=map_id:raise ValueError('内容包地图标识不一致')
                result=self.catalog.save(value,revision)if kind=='catalog'else self.cloud.save(value)
                row['result']={'committed':True,'value':result};return row['result']
            except Exception:
                del self.uploads[key];raise
