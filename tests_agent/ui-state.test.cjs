const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const html=fs.readFileSync('navigation_agent/ui.html','utf8');
const phases=html.match(/const phases=\{[^\n]+\};/)[0];
const view=html.match(/function statusView\(d\)\{[^\n]+\}/)[0];
function render(d){return JSON.parse(JSON.stringify(vm.runInNewContext(phases+view+';statusView(d)',{d})));}
test('completed visit stays completed when the idle control channel is cancelled',()=>{
 assert.deepEqual(render({navigation:{task_active:false,navigation_state:'cancelled'},task:{state:'succeeded',observation:{station_index:1,station_total:1}}}),{phase:'待命',progress:'已完成 · 1/1'});
});
test('an active native task remains visible independently of the old conversation result',()=>{
 assert.equal(render({navigation:{task_active:true,navigation_state:'running'},task:{state:'succeeded'}}).phase,'导航中');
});
test('an actually cancelled task is not relabelled completed',()=>{
 assert.deepEqual(render({navigation:{task_active:false,navigation_state:'cancelled'},task:{state:'cancelled'}}),{phase:'待命',progress:'已取消'});
});
