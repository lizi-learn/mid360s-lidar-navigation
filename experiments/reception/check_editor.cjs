// Synthetic DOM regression: real rendered editor and its save handlers.
// Fetch is an in-memory fixture; this test has no robot/network interfaces.
const {JSDOM,VirtualConsole}=require('jsdom');
const fs=require('node:fs'),assert=require('node:assert/strict');
const input=JSON.parse(fs.readFileSync(0,'utf8'));
const tick=()=>new Promise(r=>setImmediate(r));
async function page(html,embedded=false,fixture=input.fixture){
 const requests=[],errors=[];let hold=false,release=null;
 let catalog=structuredClone(fixture);
 const dom=new JSDOM(html,{url:embedded?'http://fixture/api/v1/modules/navigation/workbench?route=tour':'http://fixture/reception?route=tour',runScripts:'dangerously',virtualConsole:new VirtualConsole().on('jsdomError',e=>errors.push(e.message)),beforeParse(w){
  w.ResizeObserver=class{observe(){}};w.HTMLCanvasElement.prototype.getContext=()=>({});w.requestAnimationFrame=()=>1;w.cancelAnimationFrame=()=>{};w.setInterval=()=>1;w.TextEncoder=TextEncoder;
  w.fetch=async(url,options)=>{
   const response=value=>({ok:true,json:async()=>structuredClone(value)});
   if(!options?.body)return response({robot_name:'synthetic editor',simulation:true,maps:['fixture'],reception:{map_id:'fixture'},session:{operator_verified:true,operator_mode:'manual'},request_token:'synthetic',map_revision:-1});
   const q=JSON.parse(options.body),r=q.reception;
   if(r?.operation==='read')return response({document:catalog,voice:{}});
   if(r?.operation==='map_raster')return{ok:false,json:async()=>({error:'synthetic fixture has no raster'})};
   assert.equal(r?.operation,'save','Only content saving is allowed in editor fixture');requests.push(structuredClone(q));
   if(hold)await new Promise(resolve=>release=resolve);
   catalog=structuredClone(r.document);catalog.revision++;
   return response(catalog);
  };
 }});
 const d=dom.window.document;
 async function until(test){for(let i=0;i<100;i++){if(test())return;await tick()}throw Error('Synthetic editor did not settle')}
 await until(()=>d.getElementById('pName'));for(let i=0;i<5;i++)await tick();
 function field(id,value,event=true){const e=d.getElementById(id);assert.ok(e,id);e.value=value;if(event)e.dispatchEvent(new dom.window.Event('input',{bubbles:true}));}
 async function save(button='saveButton'){const n=requests.length;d.getElementById(button).click();await until(()=>requests.length>n);for(let i=0;i<8;i++)await tick()}
 return{dom,d,field,save,until,requests,errors,hold(){hold=true},release(){hold=false;release()},close(){dom.window.close()}};
}
(async()=>{
 const results=[],payloads=[];
 for(const embedded of [false,true]){
  const variant=embedded?'embedded':'direct';
  for(const button of ['saveButton','applyPoint']){
   const q=await page(embedded?input.embedded:input.direct,embedded);
   q.d.getElementById('addButton').click();q.field('pName','  旧点名  ',false);q.d.getElementById(button).click();for(let i=0;i<8;i++)await tick();
   assert.equal(q.requests.length,0);assert.match(q.d.getElementById('toast').textContent,/第2个点位.*第1个点位重复/);assert.equal(q.d.activeElement.id,'pName');
   q.field('pName','独立讲解点');await q.save(button);assert.equal(q.requests.at(-1).reception.document.points[1].name,'独立讲解点');payloads.push(q.requests.at(-1));results.push({variant,case:button+'_rejects_duplicate_and_saves_corrected_name',passed:true});q.close();
  }
  const gaps=structuredClone(input.fixture);gaps.points[0].name='点位 1';gaps.points.push({...gaps.points[0],id:'third',name:'点位 3'});
  const generated=await page(embedded?input.embedded:input.direct,embedded,gaps);generated.d.getElementById('addButton').click();assert.equal(generated.d.getElementById('pName').value,'点位 2');await generated.save();payloads.push(generated.requests.at(-1));results.push({variant,case:'generated_name_skips_existing_names_after_deletion_gap',passed:true});generated.close();
  let p=await page(embedded?input.embedded:input.direct,embedded);
  p.field('pName','');p.field('pName','接待起点',false);await p.save();
  assert.equal(p.requests.at(-1).reception.document.points[0].name,'接待起点');assert.equal(p.d.getElementById('pName').value,'接待起点');payloads.push(p.requests.at(-1));results.push({variant,case:'top_save_flushes_visible_name_after_missing_final_input_event',passed:true});p.close();
  p=await page(embedded?input.embedded:input.direct,embedded);
  p.field('pName','接待起点',false);p.field('pScript','欢迎光临，介绍我们的产品。',false);p.field('pDwell','12',false);await p.save('applyPoint');
  const point=p.requests.at(-1).reception.document.points[0];assert.equal(point.name,'接待起点');assert.equal(point.dwell,12);assert.equal(point.script,'欢迎光临，介绍我们的产品。');payloads.push(p.requests.at(-1));results.push({variant,case:'point_button_preserves_name_script_dwell',passed:true});p.close();
  p=await page(embedded?input.embedded:input.direct,embedded);
  p.d.querySelector('[data-tab="routes"]').click();p.d.querySelector('[data-remove="0"]').click();await p.save('saveRoute');assert.equal(p.requests.at(-1).reception.document.routes[0].steps.length,0);payloads.push(p.requests.at(-1));results.push({variant,case:'last_route_point_removed_then_saved_as_draft',passed:true});p.close();
  p=await page(embedded?input.embedded:input.direct,embedded);
  p.field('pName','');p.d.getElementById('addButton').click();p.field('pName','新增接待点',false);p.d.getElementById('saveButton').click();await tick();assert.equal(p.requests.length,0);assert.match(p.d.getElementById('toast').textContent,/第1个点位/);assert.equal(p.d.activeElement.id,'pName');p.field('pName','接待起点');await p.save();assert.equal(p.requests.at(-1).reception.document.points[1].name,'新增接待点');payloads.push(p.requests.at(-1));results.push({variant,case:'other_unnamed_point_is_focused_without_losing_current_edits',passed:true});p.close();
  const emptyRoute=structuredClone(input.fixture);emptyRoute.routes[0].steps=[];p=await page(embedded?input.embedded:input.direct,embedded,emptyRoute);assert.notEqual(p.d.getElementById('taskStatus').textContent,'连接中断');assert.equal(p.d.getElementById('pName').value,'旧点名');results.push({variant,case:'reload_url_selecting_empty_route_keeps_editor_connected',passed:true});p.close();
  p=await page(embedded?input.embedded:input.direct,embedded);
  p.hold();p.field('pName','首次提交');p.d.getElementById('saveButton').click();await p.until(()=>p.requests.length===1);p.field('pName','保存过程中继续编辑');p.release();for(let i=0;i<8;i++)await tick();assert.equal(p.d.getElementById('pName').value,'保存过程中继续编辑');assert.match(p.d.getElementById('revisionBadge').textContent,/草稿/);await p.save();assert.equal(p.requests.at(-1).reception.document.points[0].name,'保存过程中继续编辑');assert.equal(p.requests.at(-1).reception.expected_revision,2);payloads.push(p.requests.at(-1));results.push({variant,case:'save_response_preserves_new_edits_and_advances_revision',passed:true});p.close();
 }
 console.log(JSON.stringify({passed:true,cases:results,payloads,scope:'Real direct/embedded HTML and event handlers in synthetic DOM; no browser visual or robot interaction'}));
})().catch(e=>{console.error(e);process.exit(1)});
