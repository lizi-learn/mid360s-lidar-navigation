"""Historical Yunji cloud Open API scalar signing, separate from navigation.
Reference: yunji-ai/open-api d357b1a examples/JavaScriptSign.js.
No real cloud requests or order-state emulation are performed here.
"""
import base64
import hashlib
import hmac

def canonical_string(parameters):
    values=[]
    for key,value in sorted(parameters.items()):
        if not isinstance(key,str) or not key.isascii():raise ValueError('ASCII parameter keys required')
        if key=='signature' or value is None or value=='':continue
        if isinstance(value,(dict,list,tuple,bool)):
            raise ValueError('Pre-serialize complex/boolean values using the colleague SDK contract')
        if not isinstance(value,(str,int)):
            raise ValueError('Only strings and integers have an explicit scalar signing contract')
        values.append(key+'='+str(value))
    return '&'.join(values)

def signature(parameters, secret):
    payload=canonical_string(parameters).encode('utf-8')
    return base64.b64encode(hmac.new((secret+'&').encode('utf-8'),payload,hashlib.sha1).digest()).decode('ascii')
