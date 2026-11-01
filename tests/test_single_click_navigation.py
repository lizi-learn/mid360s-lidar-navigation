"""Run actual APP workbench JS: one explicit Start plans and starts exactly once."""
import json
from pathlib import Path
import re
import shutil
import subprocess
import unittest


@unittest.skipUnless(shutil.which('node'), 'Node.js required for actual WebView script')
class SingleClickNavigationTests(unittest.TestCase):
    def test_start_preview_cancellation_and_double_tap_use_actual_script(self):
        html=(Path(__file__).resolve().parents[1]/'navigation/console.html').read_text()
        script=re.findall(r'<script>(.*?)</script>',html,re.S)[-1]
        script=re.sub(r'refresh\(\);(?:const consoleRefreshTimer=)?setInterval\(refresh,250\);','',script,count=1)
        harness=r'''
const vm=require('node:vm'),assert=require('node:assert/strict');
function fixture(){
 const elements=new Map(),calls=[];
 function element(id){if(!elements.has(id))elements.set(id,{
  value:id==='speed'?'0.15':'0',clientWidth:800,clientHeight:480,
  classList:{toggle(){}},style:{},replaceChildren(){},addEventListener(){},
  getContext(){return {setTransform(){},clearRect(){},fillText(){}}}
 });return elements.get(id)}
 const sandbox={console,Math,Number,JSON,Map,Set,devicePixelRatio:1,
  location:{pathname:'/api/v1/modules/navigation/workbench'},navigator:{},Option:class {},
  document:{getElementById:element,activeElement:null},window:{addEventListener(){}},
  setInterval(){},setTimeout,clearTimeout,AbortController};
 vm.createContext(sandbox);vm.runInContext(SOURCE,sandbox);
 sandbox.fixture={simulation:false,gait_profile:'p2-flat-model',execution_ready:true,robot_name:'fixture',mode:'navigation',pose:[0,0,0],
  session:{generation:0,phase:'idle',navigation_permitted:true,operator_mode:'navigation_allowed'},
  gate:{ground_ok:true,localized:true,calibration_ready:true,collision_source_valid:true,
    data_fresh:true,scan_age:.1,odom_age:.1,command:[0,0,0]},maps:[],map:null,map_revision:0};
 vm.runInContext('selection=[1,2];prepared=null;get=async()=>fixture',sandbox);
 sandbox.api=async(path,data)=>{
  calls.push([path,data]);
  if(path==='goal'){
   Object.assign(sandbox.fixture.session,{generation:11,phase:'ready'});
   return {generation:11,planned:true};
  }
  if(path==='confirm'){
   assert.equal(data.generation,11);sandbox.fixture.session.phase='executing';return {accepted:true};
  }
  if(path==='stop'){sandbox.fixture.session.phase='cancelled';return {stopped:true}}
  throw Error(path);
 };
 return {sandbox,calls,element};
}
(async()=>{
 {
  const {sandbox,calls,element}=fixture();await sandbox.refresh();
  assert.equal(element('confirm').disabled,false,'Selected point can start without Preview');
  assert.equal(calls.length,0,'Entering left mode or refreshing never starts movement');
  await sandbox.confirmStart();assert.deepEqual(calls.map(c=>c[0]),['goal','confirm']);
  assert.equal(element('confirm').disabled,true,'Executing route cannot be started again');
  assert.equal(calls[0][1].heading_mode,'position','Point goal explicitly omits final heading');
 }
 {
  const {sandbox,calls}=fixture();await sandbox.refresh();
  await sandbox.prepare();assert.deepEqual(calls.map(c=>c[0]),['goal'],'Optional preview never starts');
  await sandbox.confirmStart();assert.deepEqual(calls.map(c=>c[0]),['goal','confirm'],'Preview does not add a second plan or extra prompt');
 }
 {
  const {sandbox,calls}=fixture();await sandbox.refresh();let release;
  const original=sandbox.api;
  sandbox.api=async(path,data)=>path==='goal'?await new Promise(resolve=>{
    calls.push([path,data]);release=()=>{Object.assign(sandbox.fixture.session,{generation:11,phase:'ready'});resolve({generation:11,planned:true})};
  }):await original(path,data);
  const first=sandbox.confirmStart();await sandbox.confirmStart();
  assert.deepEqual(calls.map(c=>c[0]),['goal'],'Double tap queues only one goal');
  release();await first;assert.deepEqual(calls.map(c=>c[0]),['goal','confirm']);
 }
 {
  const {sandbox,calls,element}=fixture();await sandbox.refresh();element('fixedYaw').checked=true;
  await sandbox.confirmStart();assert.equal(calls[0][1].yaw,0,'An explicit zero-degree arrival is preserved');assert.equal(calls[0][1].heading_mode,'pose');
 }
 for(const action of ['cancel','change_target']){
  const {sandbox,calls}=fixture();await sandbox.refresh();let release;
  const original=sandbox.api;
  sandbox.api=async(path,data)=>path==='goal'?await new Promise(resolve=>{
    calls.push([path,data]);release=()=>resolve({generation:11,planned:true});
  }):await original(path,data);
  const started=sandbox.confirmStart();
  if(action==='cancel')await sandbox.stop();
  else vm.runInContext('selection=[3,4];invalidate()',sandbox);
  release();await started;
  assert.equal(calls.filter(c=>c[0]==='confirm').length,0,action+' during planning cannot start late');
  assert.ok(calls.some(c=>c[0]==='stop'),'Old plan is cancelled');
 }
 for(const change of [s=>s.execution_ready=false,s=>s.session.navigation_permitted=false,
    s=>s.gate.ground_ok=false,s=>s.gate.collision_source_valid=false,s=>s.gate.localized=false]){
  const {sandbox,calls,element}=fixture();change(sandbox.fixture);await sandbox.refresh();
  assert.equal(element('confirm').disabled,true);await sandbox.confirmStart();assert.equal(calls.length,0);
 }
 {
  const {sandbox,calls}=fixture();await sandbox.refresh();
  sandbox.api=async(path,data)=>{calls.push([path,data]);throw Error('No feasible path')};
  await sandbox.confirmStart();assert.deepEqual(calls.map(c=>c[0]),['goal']);
 }
 console.log('single Start, preview-only, double tap, cancellation, target change and real-input failures: PASS');
})().catch(e=>{console.error(e);process.exitCode=1});
'''.replace('SOURCE',json.dumps(script))
        result=subprocess.run(['node','-e',harness],capture_output=True,text=True,timeout=15)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)


if __name__=='__main__':unittest.main()
