"""Execute the actual APP/WebView script against authoritative status fixtures."""
import json,re,shutil,subprocess,unittest
from pathlib import Path

@unittest.skipUnless(shutil.which('node'),'Node.js is needed for actual WebView JavaScript tests')
class ConsoleExecutionUITests(unittest.TestCase):
    def test_actual_refresh_requires_ready_physical_transport_and_live_health(self):
        html=(Path(__file__).resolve().parents[1]/'navigation/console.html').read_text()
        script=re.findall(r'<script>(.*?)</script>',html,re.S)[-1]
        script=script.replace('refresh();setInterval(refresh,250);','')
        harness=r'''
const vm=require('node:vm'),assert=require('node:assert/strict');
const elements=new Map(),calls=[];
function element(id){if(!elements.has(id))elements.set(id,{
 value:id==='speed'?'0.15':'',clientWidth:800,clientHeight:480,
 classList:{toggle(){}},style:{},replaceChildren(){},addEventListener(){},
 getContext(){return {setTransform(){},clearRect(){},fillText(){}}}
});return elements.get(id)}
const sandbox={console,Math,Number,JSON,Map,Set,devicePixelRatio:1,
 location:{pathname:'/'},navigator:{},Option:class {},
 document:{getElementById:element,activeElement:null},window:{addEventListener(){}},
 setInterval(){},setTimeout,clearTimeout,AbortController};
vm.createContext(sandbox);vm.runInContext(SOURCE,sandbox);
const ready={simulation:false,execution_ready:true,robot_name:'fixture',
 session:{generation:7,phase:'ready',navigation_permitted:true,operator_mode:'navigation_allowed'},
 gate:{ground_ok:true,localized:true,calibration_ready:true,collision_source_valid:true,
 data_fresh:true,scan_age:.1,odom_age:.1,command:[0,0,0]},maps:[],map:null,map_revision:0};
async function show(status,generation=7){
 sandbox.fixture=structuredClone(status);
 vm.runInContext('prepared='+generation+';get=async()=>fixture',sandbox);
 await sandbox.refresh();return !element('confirm').disabled;
}
(async()=>{
 assert.equal(await show(ready),true,'Ready real hardware must not be limited to simulation');
 sandbox.api=async(path,data)=>{calls.push([path,data]);return {accepted:true}};
 assert.equal(calls.length,0,'Rendering ready state cannot issue a motion request');
 await sandbox.confirmStart();assert.equal(calls[0][0],'confirm');
 assert.equal(calls[0][1].generation,7,'Only explicit confirmation submits the prepared generation');
 for(const [label,change] of [
 ['disconnected',s=>s.execution_ready=false],
 ['unknown readiness',s=>delete s.execution_ready],
 ['manual',s=>{s.session.operator_mode='manual';s.session.phase='manual';s.session.navigation_permitted=false}],
 ['paused',s=>{s.session.phase='paused';s.session.navigation_permitted=false}],
 ['no permission',s=>s.session.navigation_permitted=false],
 ['unaccepted real calibration',s=>s.gate.calibration_ready=false],
 ['no ground',s=>s.gate.ground_ok=false],['no localization',s=>s.gate.localized=false],
 ['invalid obstacle source',s=>s.gate.collision_source_valid=false],
 ['stale',s=>s.gate.data_fresh=false],['old scan',s=>s.gate.scan_age=.501],
 ['future odometry',s=>s.gate.odom_age=-.051],['missing age',s=>delete s.gate.odom_age],
 ['cancelled',s=>s.session.phase='cancelled']]){
  const s=structuredClone(ready);change(s);assert.equal(await show(s),false,label);
 }
 assert.equal(await show(ready,6),false,'A previous goal cannot start');
 assert.equal(await show(ready,'null'),false,'A new page has no prepared goal');
 assert.equal(await show({...ready,simulation:true}),true,'Ready simulation remains usable');
 assert.equal(await show({...ready,simulation:true,execution_ready:false}),false);
 await show(ready);sandbox.get=async()=>{throw Error('connection lost')};await sandbox.refresh();
 assert.equal(element('confirm').disabled,true,'Disconnect must disable confirmation');
 assert.match(element('message').textContent,/未连接/);
 assert.equal(calls.length,1,'Refresh, failures and reconnect never auto-confirm');
 console.log('actual refresh, explicit confirm, rejection/recovery cases: PASS');
})().catch(e=>{console.error(e);process.exitCode=1});
'''.replace('SOURCE',json.dumps(script))
        q=subprocess.run(['node','-e',harness],capture_output=True,text=True,timeout=15)
        self.assertEqual(q.returncode,0,q.stdout+q.stderr)

if __name__=='__main__':unittest.main()
