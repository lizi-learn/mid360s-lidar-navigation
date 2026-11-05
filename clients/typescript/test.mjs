import assert from 'node:assert/strict';
import {test, before, after} from 'node:test';
import {spawn} from 'node:child_process';
import {mkdtemp, rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import {join} from 'node:path';
import {createServer} from 'node:http';
import {NavigationClient, NavigationError} from './index.js';
let process, directory, client;
before(async () => {
  directory = await mkdtemp(join(tmpdir(),'navbridge-ts-'));
  process = spawn('python3',['-m','navigation_sdk.gateway','--backend','mock','--port','0','--state-dir',directory],{cwd:new URL('../../',import.meta.url),stdio:['ignore','pipe','pipe']});
  const line = await new Promise((resolve,reject) => {let text='';process.stdout.on('data',chunk=>{text+=chunk;if(text.includes('\n'))resolve(text.split('\n')[0]);});process.on('exit',code=>reject(Error('Gateway exit '+code)));});
  client = new NavigationClient(JSON.parse(line).http);
});
after(async()=>{process?.kill('SIGTERM');await new Promise(resolve=>process?.exitCode===null?process.once('exit',resolve):resolve());await rm(directory,{recursive:true,force:true});});
test('Node client traverses actual loopback HTTP task lifecycle',async()=>{
  assert.equal((await client.capabilities()).hardware,false);
  const first=await client.move({marker:'产品展示'},{idempotencyKey:'ts-1'});
  assert.equal(first.state,'accepted');assert.equal((await client.move({marker:'产品展示'},{idempotencyKey:'ts-1'})).id,first.id);
  assert.equal((await client.pause(first.id)).state,'paused');assert.equal((await client.resume(first.id)).state,'running');
  assert.equal((await client.cancel(first.id)).state,'cancelled');
});
test('TypeScript-client wait timeout leaves task active',async()=>{
  const t=await client.visit('产品展示');
  await assert.rejects(client.wait(t.id,{timeout:15,interval:5}),e=>e instanceof NavigationError&&e.code==='WAIT_TIMEOUT');
  assert.equal((await client.task(t.id)).state,'accepted');await client.cancel(t.id);
});
test('Marker CRUD preserves Unicode',async()=>{
  await client.saveMarker({name:'节点 &? 中文',x:1,y:2,theta:0});
  assert.ok((await client.markers()).some(m=>m.name==='节点 &? 中文'));
  await client.deleteMarker('节点 &? 中文');
});
test('Relative units, full revolutions and ordered tours retain exact request identity',async()=>{
  for(const [method,value,expected] of [['forward',.5,{kind:'forward',meters:.5}],['rotate',-4*Math.PI,{kind:'rotate',radians:-4*Math.PI}],['tour',['产品展示','接待起点'],{kind:'tour',markers:['产品展示','接待起点']}]]){
    const task=await client[method](value,{idempotencyKey:'ts-relative-'+method});
    assert.deepEqual(task.request,expected);
    assert.equal((await client[method](value,{idempotencyKey:'ts-relative-'+method})).id,task.id);
    await client.cancel(task.id);
  }
});

test('HTTP mutation 500 is uncertain and redirects are not followed',async()=>{
  const calls=[];const upstream=createServer((request,response)=>{
    calls.push(request.url);
    if(request.method==='POST'){response.writeHead(500,{'Content-Type':'application/json'});response.end('{"error":"after dispatch"}');}
    else{response.writeHead(302,{Location:'/unexpected'});response.end();}
  });
  await new Promise(resolve=>upstream.listen(0,'127.0.0.1',resolve));
  try{
    const nav=new NavigationClient('http://127.0.0.1:'+upstream.address().port);
    await assert.rejects(nav.move({x:1,y:0}),e=>e.code==='UPSTREAM_UNCERTAIN');
    await assert.rejects(nav.status(),e=>e.code==='UPSTREAM_UNAVAILABLE');
    assert.deepEqual(calls,['/v1/tasks','/v1/status']);
  }finally{await new Promise(resolve=>upstream.close(resolve));}
});
