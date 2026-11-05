"""Exercise real reception JS with reboot/map-switch responses, without ROS."""
import json
from pathlib import Path
import re
import shutil
import subprocess
import unittest


@unittest.skipUnless(shutil.which('node'), 'Node.js required for UI execution')
class ReceptionMapStateUITests(unittest.TestCase):
    def test_map_load_is_explicit_and_never_replays_start(self):
        root = Path(__file__).resolve().parents[1] / 'navigation'
        html = (root / 'reception.html').read_text()
        quick = (root / 'reception_quick.js').read_text()
        source = re.search(r'^async function missionCommand\(.*$', html, re.M)[0]
        for name in ('navigationMapContext', 'loadEditedNavigationMap'):
            source += '\n' + re.search(
                r'(?:async )?function ' + name + r'\([^)]*\)\{.*?\n\}', quick, re.S)[0]
        harness = r'''
const vm=require('node:vm'),assert=require('node:assert/strict');
function fixture(){
 const calls=[],toasts=[];
 const status={mode:'navigation',process_alive:true,reception:{map_id:'floor4',mission:{epoch:42}}};
 const sandbox={console,Error,doc:{map_id:'floor4',revision:49,points:[{id:'end',name:'终点'}]},
  state:structuredClone(status),missionPending:false,mapLoadPending:false,dirty:false,operator:false,
  selectedRoute:'tour',runKind:'point',temporaryTarget:null,selected:'end',selectedDock:null,
  renderStatus(){},renderQuickState(){},renderInspector(){},toast:s=>toasts.push(s),
  async refresh(){},async saveAll(){calls.push(['save']);return true;},
  async request(path,data){calls.push([path,data]);return path.startsWith('/api/status')?structuredClone(status):{accepted:true};},
  async mapCall(operation,data){calls.push([operation,data]);return {mode:'navigation'};}};
 vm.createContext(sandbox);vm.runInContext(SOURCE,sandbox);return{sandbox,calls,toasts,status};
}
(async()=>{
 {
  const {sandbox:s,calls,toasts,status}=fixture();status.mode='idle';status.process_alive=false;status.reception.map_id='unselected';
  assert.equal(await s.missionCommand('start'),false);
  assert.deepEqual(calls.map(c=>c[0]),['/api/status?summary=1']);
  assert.match(toasts[0],/尚未载入/,'An unloaded map is not a two-map conflict');
  assert.equal(s.missionPending,false);
  s.dirty=true;await s.loadEditedNavigationMap();
  assert.deepEqual(calls.map(c=>c[0]),['/api/status?summary=1','/api/status?summary=1','map_load']);
  assert.equal(calls.at(-1)[1].map_id,'floor4');
  assert.equal(s.selected,'end');assert.equal(s.dirty,true,'Loading the raster preserves unsaved point edits');
  assert.equal(calls.some(c=>c[0]==='/api/confirm'),false,'Loading cannot replay the earlier failed Start');
 }
 {
  const {sandbox:s,calls}=fixture();s.state.reception.map_id='unselected';
  assert.equal(await s.missionCommand('start'),true,'Fresh map status overrides a stale browser snapshot');
  assert.deepEqual(calls.map(c=>c[0]),['/api/status?summary=1','/api/confirm']);
  assert.equal(calls[1][1].reception_command.point_id,'end');
  assert.equal(calls[1][1].reception_command.epoch,42);
 }
 for(const [change,pattern] of [
  [s=>s.reception.map_id='floor5',/另一张地图|地图不同/],
  [s=>s.process_alive=false,/进程已停止/],
  [s=>s.mode='mapping',/正在建图/]
 ]){
  const {sandbox:s,calls,toasts,status}=fixture();change(status);
  await s.missionCommand('resume');assert.match(toasts.at(-1),pattern);
  assert.equal(calls.some(c=>c[0]==='/api/confirm'),false);
 }
 for(const command of ['pause','cancel']){
  const {sandbox:s,calls}=fixture();s.state.reception.map_id='unselected';
  assert.equal(await s.missionCommand(command),true,'Map selection cannot prevent a stop command');
  assert.equal(calls[0][0],'/api/confirm');assert.equal(calls[0][1].reception_command.command,command);
 }
 {
  const {sandbox:s,calls,toasts}=fixture();s.request=async()=>{throw Error('connection lost');};
  await s.missionCommand('start');assert.equal(calls.length,0);assert.match(toasts[0],/connection lost/);
 }
 {
  const {sandbox:s,calls}=fixture();let release;s.mapCall=async(operation,data)=>{calls.push([operation,data]);await new Promise(resolve=>release=resolve);};
  const load=s.loadEditedNavigationMap();await new Promise(resolve=>setImmediate(resolve));
  await s.loadEditedNavigationMap();assert.equal(calls.filter(c=>c[0]==='map_load').length,1);
  release();await load;assert.equal(s.mapLoadPending,false);
 }
 console.log('Reboot, fresh status, real mismatch, stop, disconnect and duplicate load: PASS');
})().catch(e=>{console.error(e);process.exitCode=1});
'''.replace('SOURCE', json.dumps(source))
        result = subprocess.run(['node', '-e', harness], capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
