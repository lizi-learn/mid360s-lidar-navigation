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
const reply=html.match(/function taskReply\(t\)\{[^\n]+\}/)[0];
function taskReply(t){return vm.runInNewContext(reply+';taskReply(t)',{t});}
test('accepted forward bubble is replaced by the real later failure and reason',()=>{
 assert.equal(taskReply({state:'failed',observation:{native_phase:'blocked',reason:'无法完成导航，速度已归零，请手动处理'}}),'导航未完成。无法完成导航，速度已归零，请手动处理');
});
test('completion, manual takeover and cancellation remain distinct',()=>{
 assert.equal(taskReply({state:'succeeded',observation:{navigation_state:'cancelled'}}),'导航任务已完成。');
 assert.equal(taskReply({state:'interrupted'}),'导航任务已被手动接管。');
 assert.equal(taskReply({state:'cancelled'}),'导航任务已取消。');
});
test('uncertain observation never claims execution or automatically retries',()=>{
 assert.equal(taskReply({state:'running',observation_stale:true}),'暂时无法确认导航执行状态，未自动重发。');
 assert.equal(taskReply({state:'running'}),'导航任务执行中，尚未完成。');
});
test('polling replaces the actual accepted chat bubble without a new model or motion request',async()=>{
 const elements=new Map(),answer={textContent:'导航任务已受理，正在执行。'};
 const $=id=>{if(!elements.has(id))elements.set(id,{classList:{remove(){},add(){}}});return elements.get(id)};
 const calls=[],data={navigation:{operator_mode:'navigation_allowed',task_active:false,navigation_state:'failed'},task:{id:'forward-5',state:'failed',observation:{reason:'无法完成导航，速度已归零，请手动处理'}}};
 const refresh=html.match(/async function refresh\(\)\{[^\n]+\}/)[0];
 const ctx={$ ,conversation:'this-chat',taskBubbles:new Map([['forward-5',answer]]),showReply:(node,text)=>node.textContent=text,api:async path=>{calls.push(path);return data}};
 await vm.runInNewContext(phases+view+reply+refresh+';refresh()',ctx);
 assert.equal(answer.textContent,'导航未完成。无法完成导航，速度已归零，请手动处理');
 assert.equal($('progress').textContent,'未完成');
 assert.deepEqual(calls,['status?conversation=this-chat']);
});
