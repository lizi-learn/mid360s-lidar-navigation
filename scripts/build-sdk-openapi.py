"""Build the checked-in OpenAPI contract from the implemented SDK surface."""
import json
from pathlib import Path
S={'Error':{'type':'object','required':['error'],'properties':{'error':{'type':'object','required':['code','message','details'],'properties':{'code':{'type':'string'},'message':{'type':'string'},'details':{'type':'object'}}}}},
   'Target':{'oneOf':[{'type':'object','additionalProperties':False,'required':['marker'],'properties':{'marker':{'type':'string','minLength':1,'maxLength':80}}},
                     {'type':'object','additionalProperties':False,'required':['x','y'],'properties':{'x':{'type':'number'},'y':{'type':'number'},'theta':{'type':['number','null'],'minimum':-3.141592653589793,'maximum':3.141592653589793}}}]},
   'TaskRequest':{'oneOf':[{'type':'object','additionalProperties':False,'required':['kind','target'],'properties':{'kind':{'const':'move'},'target':{'$ref':'#/components/schemas/Target'}}},
                         {'type':'object','additionalProperties':False,'required':['kind','marker'],'properties':{'kind':{'const':'visit'},'marker':{'type':'string','minLength':1,'maxLength':80}}},
                         {'type':'object','additionalProperties':False,'required':['kind','route_id'],'properties':{'kind':{'const':'route'},'route_id':{'type':'string','minLength':1,'maxLength':64}}}]},
   'Task':{'type':'object','required':['id','state','backend','request','created_at','updated_at','native_ref'],'properties':{'id':{'type':'string'},'state':{'type':'string','enum':['dispatching','accepted','running','blocked','paused','presenting','waiting','dwelling','succeeded','failed','cancelled','interrupted','unknown']},'backend':{'type':'string'},'request':{'$ref':'#/components/schemas/TaskRequest'},'native_ref':{'type':['object','null']},'created_at':{'type':'number'},'updated_at':{'type':'number'},'observation_stale':{'type':'boolean'},'observation':{'type':'object'}}},
   'Marker':{'type':'object','required':['name'],'properties':{'id':{'type':'string','readOnly':True},'name':{'type':'string','minLength':1,'maxLength':80},'x':{'type':'number'},'y':{'type':'number'},'theta':{'type':['number','null']},'script':{'type':'string'},'dwell':{'type':'number','minimum':0},'guest_wait':{'type':'boolean'},'kind':{'type':'string','enum':['visit','transit','home']}}},
   'Object':{'type':'object'},'Objects':{'type':'array','items':{'type':'object'}},'Empty':{'type':'object','maxProperties':0}}
for kind,key,spec in [('forward','meters',{'type':'number','exclusiveMinimum':0,'maximum':100000}),
                      ('rotate','radians',{'type':'number','minimum':-100000,'maximum':100000,'not':{'const':0},'description':'Accumulated angle, left positive; retain full revolutions.'}),
                      ('tour','markers',{'type':'array','minItems':1,'maxItems':100,'items':{'type':'string','minLength':1,'maxLength':80}})]:
    S['TaskRequest']['oneOf'].append(dict(type='object',additionalProperties=False,required=['kind',key],properties={'kind':{'const':kind},key:spec}))
S['RelativeAction']={'oneOf':S['TaskRequest']['oneOf'][3:5]}
S['TaskRequest']['oneOf'].append(dict(type='object',additionalProperties=False,required=['kind','steps'],properties={
    'kind':{'const':'sequence'},'steps':{'type':'array','minItems':2,'maxItems':100,'items':{'$ref':'#/components/schemas/RelativeAction'}}}))
def ref(name):return {'$ref':'#/components/schemas/'+name}
def operation(name,summary,response='Object',body=None,parameter=None,status='200'):
    obj={'operationId':name,'summary':summary,'responses':{status:{'description':'Result','content':{'application/json':{'schema':ref(response)}}},'default':{'description':'Structured error; unsupported capabilities are explicit','content':{'application/json':{'schema':ref('Error')}}}}}
    if body:obj['requestBody']={'required':True,'content':{'application/json':{'schema':ref(body)}}}
    if parameter:obj['parameters']=[{'name':parameter,'in':'path','required':True,'schema':{'type':'string'}}]
    return obj
paths={}
for path,name,summary,res in [('capabilities','capabilities','Provider capabilities','Object'),('status','status','Measured provider state; absent hardware fields are null','Object'),('maps','maps','Map list when supported','Objects'),('markers','markers','Marker list','Objects'),('routes','routes','Saved routes when supported','Objects')]:
    paths['/v1/'+path]={'get':operation(name,summary,res)}
paths['/v1/tasks']={'post':operation('submit','Dispatch move, visit, saved route, relative forward/rotation or temporary ordered tour; accepted is not arrival','Task','TaskRequest',status='202')}
paths['/v1/tasks']['post']['parameters']=[{'name':'Idempotency-Key','in':'header','required':False,'schema':{'type':'string','minLength':1,'maxLength':128},'description':'Persisted per state directory. Same key + different request returns 409. No automatic replay after uncertain dispatch.'}]
paths['/v1/tasks/{task_id}']={'get':operation('task','Observe this task only','Task',parameter='task_id')}
for action in ('cancel','pause','resume','continue'):
    paths['/v1/tasks/{task_id}/'+action]={'post':operation(action+'Task','Conditional task control; supported operations depend on provider','Task','Empty','task_id')}
paths['/v1/stop']={'post':operation('stop','Global current navigation stop; never motor power-off','Object','Empty')}
paths['/v1/markers']['post']=operation('saveMarker','Upsert marker by normalized unique name','Marker','Marker')
paths['/v1/markers/{name}']={'delete':operation('deleteMarker','Delete marker; referenced P2 points are not silently removed from routes','Object',parameter='name')}
obj={'openapi':'3.1.0','info':{'title':'P2 Navigation SDK / NavBridge','version':'0.1.0','description':'Unified P2 and documented WATER subset. Coordinates metres/radians. Static docs never connect to hardware. Not a vendor-certified all-product API.'},'servers':[{'url':'http://127.0.0.1:18940','description':'Local companion'}],
     'security':[{'bearerAuth':[]},{}],'components':{'schemas':S,'securitySchemes':{'bearerAuth':{'type':'http','scheme':'bearer','description':'Required when the gateway is configured with NAVBRIDGE_API_KEY'}}},'paths':paths}
Path('api/navigation.openapi.json').write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n')
