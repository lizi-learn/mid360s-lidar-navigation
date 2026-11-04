// Synthetic DOM regression: real rendered editor and its save handlers.
// Fetch is an in-memory fixture; this test has no robot/network interfaces.
const {JSDOM,VirtualConsole}=require('jsdom');
const fs=require('node:fs'),assert=require('node:assert/strict');
const input=JSON.parse(fs.readFileSync(0,'utf8'));
const tick=()=>new Promise(r=>setImmediate(r));
async function page(html,embedded=false,fixture=input.fixture,overrides={},autoEdit=true){
 const requests=[],missions=[],stops=[],mapOperations=[],errors=[],intervals=[];let hold=false,release=null,offline=false;
 let liveStatus={robot_name:'synthetic editor',simulation:true,maps:['fixture'],reception:{map_id:'fixture'},session:{operator_verified:true,operator_mode:'manual',navigation_permitted:false},gait_transport:{controller_instance:1,readiness:3,phase:'stopped'},request_token:'synthetic',map_revision:-1,...overrides};
 let catalog=structuredClone(fixture);
 const dom=new JSDOM(html,{url:embedded?'http://fixture/api/v1/modules/navigation/workbench?route=tour':'http://fixture/reception?route=tour',runScripts:'dangerously',pretendToBeVisual:true,virtualConsole:new VirtualConsole().on('jsdomError',e=>errors.push(e.message)),beforeParse(w){
  w.ResizeObserver=class{observe(){}};w.HTMLCanvasElement.prototype.getContext=()=>new Proxy({measureText:t=>({width:String(t).length*8})},{get:(o,k)=>o[k]??(()=>{})});w.HTMLCanvasElement.prototype.setPointerCapture=()=>{};w.requestAnimationFrame=()=>1;w.cancelAnimationFrame=()=>{};w.setInterval=(fn,ms)=>{intervals.push([fn,ms]);return intervals.length};w.TextEncoder=TextEncoder;
  w.fetch=async(url,options)=>{
   const response=value=>({ok:true,json:async()=>structuredClone(value)});
   if(!options?.body){if(offline)throw Error('synthetic disconnect');return response(liveStatus)}
   const q=JSON.parse(options.body),r=q.reception;
   if(String(url).endsWith('/api/confirm')){missions.push(structuredClone(q));return liveStatus.session.navigation_permitted?response({phase:'navigating'}):{ok:false,json:async()=>({error:'当前 E 档位或摇杆未允许导航'})}}
   if(String(url).endsWith('/api/stop')){stops.push(q);return response({stopped:true})}
   if(r?.operation==='read')return response({document:catalog,voice:{}});
   if(r?.operation==='map_raster')return{ok:false,json:async()=>({error:'synthetic fixture has no raster'})};
   if(['map_observations','map_observations_reset','map_observations_merge'].includes(r?.operation)){mapOperations.push(structuredClone(r));liveStatus.reception={...liveStatus.reception,map_observations:r.operation==='map_observations'?{enabled:r.enabled,cleared_cells:7,pending_cells:3}:r.operation==='map_observations_merge'?{enabled:false,cleared_cells:7,pending_cells:3,merge_requested:true}:{enabled:false,cleared_cells:0,pending_cells:0}};return response(liveStatus.reception.map_observations)}
   assert.equal(r?.operation,'save','Only content saving is allowed in editor fixture');requests.push(structuredClone(q));
   if(hold)await new Promise(resolve=>release=resolve);
   catalog=structuredClone(r.document);catalog.revision++;
   return response(catalog);
  };
 }});
 const d=dom.window.document;
 async function until(test){for(let i=0;i<100;i++){if(test())return;await tick()}throw Error('Synthetic editor did not settle')}
 await until(()=>d.getElementById('pName'));for(let i=0;i<5;i++)await tick();
 if(autoEdit)d.getElementById('editPoint')?.click();
 function field(id,value,event=true){const e=d.getElementById(id);assert.ok(e,id);e.value=value;if(event)e.dispatchEvent(new dom.window.Event('input',{bubbles:true}));}
 async function save(button='saveButton'){const n=requests.length;d.getElementById(button).click();await until(()=>requests.length>n);for(let i=0;i<8;i++)await tick()}
 return{dom,d,field,save,until,requests,missions,stops,mapOperations,errors,hold(){hold=true},release(){hold=false;release()},async poll(value,disconnect=false){if(value)liveStatus={...liveStatus,...value};offline=disconnect;await intervals.find(x=>x[1]===1000)[0]();for(let i=0;i<8;i++)await tick()},map(){const c=d.getElementById('mapCanvas');Object.defineProperties(c,{clientWidth:{configurable:true,value:800},clientHeight:{configurable:true,value:600}});d.querySelector('[onclick="fitMap()"]').click();return c},close(){dom.window.close()}};
}
(async()=>{
 const results=[],payloads=[];
 for(const embedded of [false,true]){
  const variant=embedded?'embedded':'direct';
  const mapPage=await page(embedded?input.embedded:input.direct,embedded);
  mapPage.d.querySelector('[data-tab="maps"]').click();
  await mapPage.until(()=>mapPage.d.getElementById('autoMapUpdate'));
  const update=mapPage.d.getElementById('autoMapUpdate');assert.equal(update.checked,true);
  update.checked=false;update.dispatchEvent(new mapPage.dom.window.Event('change',{bubbles:true}));
  await mapPage.until(()=>mapPage.mapOperations.length===1);await mapPage.poll();
  assert.equal(mapPage.mapOperations[0].enabled,false);assert.match(mapPage.d.getElementById('observedMapStatus').textContent,/暂停.*7/);
  mapPage.d.getElementById('mergeObservedMap').click();await mapPage.until(()=>mapPage.mapOperations.length===2);await mapPage.poll();
  assert.equal(mapPage.mapOperations[1].operation,'map_observations_merge');assert.match(mapPage.d.getElementById('observedMapStatus').textContent,/已安排空闲/);
  mapPage.d.getElementById('resetObservedMap').click();await mapPage.until(()=>mapPage.mapOperations.length===3);await mapPage.poll();
  assert.equal(mapPage.mapOperations[2].operation,'map_observations_reset');assert.match(mapPage.d.getElementById('observedMapStatus').textContent,/暂停.*0/);
  assert.equal(mapPage.requests.length,0);assert.equal(mapPage.missions.length,0);assert.equal(mapPage.stops.length,0);
  results.push({variant,case:'map_auto_update_pause_and_versioned_reset_without_navigation_actions',passed:true});mapPage.close();
  const left={operator_verified:true,operator_mode:'navigation_allowed',navigation_permitted:true};
  let locked=await page(embedded?input.embedded:input.direct,embedded,input.fixture,{},false);
  assert.ok(locked.d.getElementById('pName').readOnly);assert.ok(locked.d.getElementById('pX').readOnly);assert.ok(locked.d.getElementById('fixedYaw').disabled);assert.ok(locked.d.getElementById('deletePoint').classList.contains('hidden'));assert.ok(locked.d.getElementById('applyPoint').classList.contains('hidden'));
  const mouse=(x,y)=>({pointerId:1,pointerType:'mouse',button:0,clientX:x,clientY:y});let lockedMap=locked.map();lockedMap.onpointerdown(mouse(400,300));lockedMap.onpointermove(mouse(452,326));lockedMap.onpointerup(mouse(452,326));
  assert.equal(locked.d.getElementById('pX').value,'0');assert.equal(locked.d.getElementById('pY').value,'0');locked.field('pName','误触修改');await locked.save();assert.equal(locked.requests[0].reception.document.points[0].name,'旧点名');assert.equal(locked.requests[0].reception.document.points[0].x,0);assert.equal(locked.missions.length,0);payloads.push(locked.requests[0]);results.push({variant,case:'saved_point_locked_pan_or_input_cannot_mutate_it',passed:true});locked.close();
  locked=await page(embedded?input.embedded:input.direct,embedded,input.fixture,{},false);locked.d.getElementById('editPoint').click();assert.ok(!locked.d.getElementById('pX').readOnly);lockedMap=locked.map();lockedMap.onpointerdown(mouse(400,300));lockedMap.onpointermove(mouse(452,326));lockedMap.onpointerup(mouse(452,326));assert.notEqual(locked.d.getElementById('pX').value,'0');await locked.save('applyPoint');assert.notEqual(locked.requests[0].reception.document.points[0].x,0);assert.ok(locked.d.getElementById('pX').readOnly);assert.ok(locked.d.getElementById('editPoint'));payloads.push(locked.requests[0]);results.push({variant,case:'explicit_edit_enables_drag_save_locks_again',passed:true});locked.close();
  locked=await page(embedded?input.embedded:input.direct,embedded,input.fixture,{},false);locked.d.getElementById('editPoint').click();locked.field('pName','临时改名');locked.field('pX','7');locked.field('pScript','不应保存的介绍');locked.d.getElementById('cancelPointEdit').click();assert.equal(locked.d.getElementById('pName').value,'旧点名');assert.equal(locked.d.getElementById('pX').value,'0');assert.equal(locked.d.getElementById('pScript').value,'');assert.ok(locked.d.getElementById('pName').readOnly);assert.match(locked.d.getElementById('revisionBadge').textContent,/已保存/);assert.equal(locked.requests.length,0);results.push({variant,case:'cancel_edit_restores_saved_point_without_request',passed:true});locked.close();
  locked=await page(embedded?input.embedded:input.direct,embedded,input.fixture,{},false);locked.d.getElementById('addButton').click();assert.ok(!locked.d.getElementById('pName').readOnly);locked.field('pName','新建讲解点');await locked.save('applyPoint');assert.ok(locked.d.getElementById('pName').readOnly);assert.ok(locked.d.getElementById('editPoint'));payloads.push(locked.requests[0]);results.push({variant,case:'new_point_is_editable_until_first_save_then_locks',passed:true});locked.close();
  const deletable=structuredClone(input.fixture);deletable.routes[0].steps=[];locked=await page(embedded?input.embedded:input.direct,embedded,deletable,{},false);locked.d.getElementById('editPoint').click();assert.ok(!locked.d.getElementById('deletePoint').classList.contains('hidden'));locked.d.getElementById('deletePoint').click();assert.equal(locked.requests.length,0);await locked.save();assert.equal(locked.requests[0].reception.document.points.length,0);payloads.push(locked.requests[0]);results.push({variant,case:'delete_appears_after_edit_and_persists_only_on_save',passed:true});locked.close();
  locked=await page(embedded?input.embedded:input.direct,embedded,input.fixture,{session:left},false);locked.d.getElementById('quickVisit').click();await locked.until(()=>locked.missions.length===1);assert.equal(locked.missions[0].reception_command.point_id,'home');assert.equal(locked.requests.length,0);await locked.until(()=>locked.d.body.classList.contains('operator'));results.push({variant,case:'locked_point_can_still_be_navigated_without_edit_or_catalog_write',passed:true});locked.close();
  let quick=await page(embedded?input.embedded:input.direct,embedded,input.fixture,{session:left});
  quick.field('pName','单点试走',false);quick.field('pScript','测试介绍',false);quick.d.getElementById('quickVisit').click();quick.d.getElementById('quickVisit').click();await quick.until(()=>quick.missions.length===1);for(let i=0;i<8;i++)await tick();
  assert.equal(quick.requests.length,1);assert.equal(quick.requests[0].reception.document.points[0].script,'测试介绍');assert.equal(quick.missions[0].reception_command.point_id,'home');assert.equal(quick.missions[0].reception_command.revision,2);assert.ok(quick.d.body.classList.contains('operator'));payloads.push(quick.requests[0]);results.push({variant,case:'point_test_saves_visible_edits_then_sends_one_start_without_confirmation',passed:true});quick.close();
  quick=await page(embedded?input.embedded:input.direct,embedded,input.fixture,{session:left});
  assert.equal(quick.d.getElementById('gaitBadge').textContent,'D 策略');assert.equal(quick.d.getElementById('automationBadge').textContent,'E 左 · 自动化已开');
  await quick.poll({session:{operator_verified:true,operator_mode:'manual',navigation_permitted:false}});assert.match(quick.d.getElementById('automationBadge').textContent,/E 右/);
  await quick.poll({session:{operator_verified:true,operator_mode:'paused',navigation_permitted:false}});assert.match(quick.d.getElementById('automationBadge').textContent,/E 中/);
  await quick.poll({session:{...left,manual_activity:true}});assert.match(quick.d.getElementById('automationBadge').textContent,/摇杆接管/);
  await quick.poll({session:{operator_verified:false,operator_mode:'navigation_allowed'},gait_transport:{controller_instance:1,readiness:0,phase:'disconnected'}});assert.match(quick.d.getElementById('automationBadge').textContent,/未连接/);assert.match(quick.d.getElementById('gaitBadge').textContent,/未连接/);
  await quick.poll({session:left,gait_transport:{controller_instance:1,readiness:2,phase:'stopped'}});assert.match(quick.d.getElementById('gaitBadge').textContent,/未进入 D/);
  await quick.poll(null,true);assert.match(quick.d.getElementById('gaitBadge').textContent,/未连接/);assert.match(quick.d.getElementById('automationBadge').textContent,/未连接/);assert.equal(quick.missions.length,0);results.push({variant,case:'badges_follow_actual_D_and_E_manual_pause_stick_unknown_and_disconnect',passed:true});quick.close();
  quick=await page(embedded?input.embedded:input.direct,embedded,input.fixture,{session:left});const c=quick.map();quick.d.getElementById('mapNavigate').click();for(let i=0;i<8;i++)await tick();assert.equal(quick.missions.length,0);const savedBeforeMapTap=quick.requests.length;
  const e=(x,y,extra={})=>({pointerId:1,pointerType:'mouse',button:0,clientX:x,clientY:y,...extra});
  c.onpointerdown(e(400,300));c.onpointerup(e(400,300));await quick.until(()=>quick.missions.length===1);assert.equal(quick.missions[0].reception_command.point_id,'home');
  c.onpointerdown(e(500,300));c.onpointermove(e(530,310));c.onpointerup(e(530,310));for(let i=0;i<8;i++)await tick();assert.equal(quick.missions.length,1,'Pan must not start navigation');
  c.onpointerdown(e(600,400));c.onpointerup(e(600,400));await quick.until(()=>quick.missions.length===2);assert.ok(Number.isFinite(quick.missions[1].reception_command.target.x));assert.ok(Number.isFinite(quick.missions[1].reception_command.target.y));assert.equal(quick.requests.length,savedBeforeMapTap,'Temporary destination must not create saved points');
  quick.d.getElementById('cancelAll').click();await quick.until(()=>quick.stops.length===1);quick.d.getElementById('mapNavigate').click();for(let i=0;i<8;i++)await tick();c.onpointerdown(e(650,420));c.onpointerup(e(650,420));for(let i=0;i<8;i++)await tick();assert.equal(quick.missions.length,2);assert.equal(quick.errors.length,0);results.push({variant,case:'explicit_map_mode_starts_saved_or_temporary_tap_pan_is_inert_cancel_works_mode_can_close',passed:true});quick.close();
  quick=await page(embedded?input.embedded:input.direct,embedded);quick.d.getElementById('quickVisit').click();await quick.until(()=>quick.missions.length===1);for(let i=0;i<8;i++)await tick();assert.match(quick.d.getElementById('toast').textContent,/未允许导航/);assert.ok(!quick.d.body.classList.contains('operator'));results.push({variant,case:'manual_mode_keeps_actual_server_rejection_visible',passed:true});quick.close();
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
