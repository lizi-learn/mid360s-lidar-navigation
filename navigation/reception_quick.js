// Explicit web test controls reuse the normal mission API and manual takeover.
let quickMapEnabled=false,mapLoadPending=false;
const quickStyle=document.createElement('style');quickStyle.textContent=`
.control-state{display:flex;gap:5px;align-items:center;flex-wrap:wrap;justify-content:flex-end}.control-state .chip{font-size:11px}.chip.offline{background:#edf0ef;color:#71807b}#mapNavigate{white-space:nowrap}#quickVisit{margin-top:8px;width:100%}.quick-visit-note{font-size:11px;line-height:1.6;color:#71847c;margin-top:7px}.nav-map-state{display:flex;align-items:center;gap:6px;flex-wrap:wrap}.nav-map-state button{white-space:nowrap}.map-header{flex-wrap:wrap}.handheld-native .map-header{height:auto;min-height:32px}.handheld-native .nav-map-state .chip{font-size:10px}.handheld-native .nav-map-state button{font-size:11px;padding:4px 7px;min-height:24px}@media(max-width:700px){.topbar .control-state{order:3;width:100%;justify-content:flex-start}}.handheld-native #mapNavigate{font-size:11px;padding:4px 7px;min-height:24px}
`;
document.head.append(quickStyle);
const controlState=document.createElement('div');controlState.className='control-state';controlState.id='controlState';controlState.role='status';controlState.innerHTML='<span class="chip offline" id="gaitBadge">策略未连接</span><span class="chip offline" id="automationBadge">E 档位未连接</span>';$('mapSelect').after(controlState);
const mapNavigate=document.createElement('button');mapNavigate.id='mapNavigate';mapNavigate.type='button';mapNavigate.textContent='点图导航';$('mapHint').after(mapNavigate);
const navigationMapView=document.createElement('div');navigationMapView.className='nav-map-state';navigationMapView.innerHTML='<span class="chip amber" id="navigationMapBadge" role="status">正在读取导航地图状态</span><button type="button" id="loadEditedMap">载入这张地图</button>';mapNavigate.before(navigationMapView);
$('loadEditedMap').onclick=loadEditedNavigationMap;
function navigationMapContext(connected=true){
    const active=state?.reception?.map_id;
    if(!connected||!active)return{matches:false,label:'导航地图状态未连接',message:'未收到导航地图状态，请检查连接',load:false};
    if(active==='unselected')return{matches:false,label:'导航地图尚未载入',message:'导航地图尚未载入，请点地图上方“载入这张地图”；载入本身不会行走',load:true};
    if(doc?.map_id!==active)return{matches:false,label:'导航使用另一张地图',message:'导航使用的地图与当前编辑地图不同，请点地图上方“载入这张地图”，或切回导航使用的地图',load:true};
    if(!state.process_alive)return{matches:false,label:'导航地图已停止',message:'这张地图的导航进程已停止，请点地图上方“重新载入地图”',load:true};
    if(state.mode!=='navigation')return{matches:false,label:'当前正在建图',message:'当前正在建图，请载入导航地图后再前往点位',load:true};
    return{matches:true,label:'导航地图已载入',message:'当前编辑地图与导航地图一致；定位状态以实时数据为准',load:false};
}
async function loadEditedNavigationMap(){
    if(mapLoadPending||!doc||doc.map_id==='unselected')return;
    const mapId=doc.map_id;mapLoadPending=true;renderQuickState();
    try{
        // Loading a saved map never submits a goal or replays an earlier Start.
        state=await request('/api/status?summary=1');
        await mapCall('map_load',{map_id:mapId});
        await refresh();toast('已载入所选地图，正在准备定位；没有开始行走');
    }catch(e){toast('地图载入失败：'+e.message);}
    finally{mapLoadPending=false;renderQuickState();}
}
function quickAuthorityState(connected=true){
    const g=state?.gait_transport,s=state?.session,linked=!!(connected&&g?.controller_instance&&g.phase!=='disconnected');
    const d=linked&&!!(g.readiness&1),walking=d&&!!(g.readiness&2),verified=!!(connected&&s?.operator_verified);
    let automation='E 档位未连接',allowed=false;
    if(verified){
        if(s.manual_activity)automation='摇杆接管 · 自动化已退出';
        else if(s.operator_mode==='manual')automation='E 右 · 手动控制';
        else if(s.operator_mode==='paused')automation='E 中 · 自动化暂停';
        else if(s.operator_mode==='navigation_allowed'){
            allowed=!!s.navigation_permitted;
            automation=allowed?'E 左 · 自动化已开':'E 左 · 自动化未允许';
        }
    }
    return{gait:!linked?'策略未连接':!d?'未进入 D 策略':walking?'D 策略':'D 策略 · 步态未就绪',automation,allowed,ready:walking&&allowed};
}
function renderQuickState(connected=true){
    const q=quickAuthorityState(connected);$('gaitBadge').textContent=q.gait;$('automationBadge').textContent=q.automation;
    const context=navigationMapContext(connected);$('navigationMapBadge').textContent=context.label;$('navigationMapBadge').className='chip'+(context.matches?'':' amber');navigationMapView.title=context.message;
    $('loadEditedMap').hidden=!context.load;$('loadEditedMap').disabled=mapLoadPending||!doc||doc.map_id==='unselected';$('loadEditedMap').textContent=mapLoadPending?'正在载入…':context.label==='导航地图已停止'?'重新载入地图':'载入这张地图';
    $('gaitBadge').className='chip'+(q.gait==='D 策略'?'':' offline');$('automationBadge').className='chip'+(q.allowed?'':' amber');
    controlState.title=connected?(state?.gait_transport?.blockers||'已收到实际策略与 E 档位回执'):'未收到当前连接状态';
    mapNavigate.textContent=quickMapEnabled?'点图即走：开':'点图导航';mapNavigate.classList.toggle('primary',quickMapEnabled);
    mapNavigate.title=quickMapEnabled?'点击点位或空白位置立即开始；拖动只移动地图。再次点击关闭。':'进入点图即走模式；启用本身不会移动机器人';
}
const quickRenderStatus=renderStatus;
renderStatus=function(){quickRenderStatus();renderQuickState();};
const quickRefresh=refresh;
refresh=async function(){await quickRefresh();renderQuickState(!!state&&$('taskStatus').textContent!=='连接中断');};
const quickSwitchMode=switchMode;
switchMode=function(value){if(!value)quickMapEnabled=false;quickSwitchMode(value);renderQuickState();};
async function quickVisitSelected(){
    if(missionPending)return;
    syncEditorDraft();
    const p=doc?.points.find(p=>p.id===($('inspector').dataset.pointId||selected));if(!p)return toast('先选择一个点位');
    selected=p.id;temporaryTarget=null;selectedDock=null;runKind='point';
    if(await missionCommand('start')){quickMapEnabled=false;switchMode(true);}
}
const quickInspector=renderInspector;
renderInspector=function(){
    quickInspector();
    if(operator||tab!=='points'||!$('applyPoint'))return;
    const button=document.createElement('button');button.id='quickVisit';button.type='button';button.className='primary';button.textContent=missionPending?'正在提交…':'前往此点测试';button.disabled=missionPending;button.onclick=quickVisitSelected;$('applyPoint').after(button);
    const note=document.createElement('div');note.className='quick-visit-note';note.textContent='一次点击保存当前编辑并前往；到点按设置讲解。右档、摇杆和取消可结束导航。';button.after(note);
};
mapNavigate.onclick=async()=>{
    if(quickMapEnabled){quickMapEnabled=false;renderQuickState();toast('点图即走已关闭；已开始的任务可点取消导航');return;}
    syncEditorDraft();
    if(dirty&&!(await saveAll()))return;
    quickMapEnabled=true;runKind='point';clearTool();switchMode(true);renderQuickState();toast('点图即走已开：点位或空白位置点一下即前往；拖动仅平移地图');
};
const quickPointerUp=canvas.onpointerup;
canvas.onpointerup=e=>{
    const tap=quickMapEnabled&&operator&&viewMode==='2d'&&pointer&&!pointer.moved&&!mapEdit&&!tool&&(e.button===undefined||e.button===0);
    const result=quickPointerUp(e);
    if(tap&&(temporaryTarget||selected))void missionCommand('start');
    return result;
};
renderQuickState(false);

// Saved points are selected by default; edits need the user's explicit Edit.
let savedPointDocument=null;
const editingPointIds=new Set();
function savedPoint(id){return savedPointDocument?.map_id===doc?.map_id?savedPointDocument.points.find(p=>p.id===id):null}
function pointCanEdit(p){return !!p&&(!savedPoint(p.id)||editingPointIds.has(p.id))}
function rememberSavedPoints(value,reset=false){
    savedPointDocument=JSON.parse(JSON.stringify(value));
    if(reset)editingPointIds.clear();
    else for(const id of editingPointIds){const p=doc.points.find(p=>p.id===id),q=savedPoint(id);if(!p||q&&['name','x','y','yaw','product_id','script','dwell','guest_wait','kind'].every(k=>p[k]===q[k]))editingPointIds.delete(id);}
}
function beginPointEdit(p){editingPointIds.add(p.id);mapEdit=null;renderInspector();$('pName')?.focus();}
function cancelPointEdit(p){
    const original=savedPoint(p.id);if(!original)return;
    doc.points[doc.points.findIndex(q=>q.id===p.id)]=JSON.parse(JSON.stringify(original));editingPointIds.delete(p.id);mapEdit=null;canvas.style.cursor='';
    touch();dirty=JSON.stringify(doc)!==JSON.stringify(savedPointDocument);
    $('saveButton').textContent=dirty?'保存更改 · 未保存':'保存更改';$('revisionBadge').textContent=(dirty?'草稿 · v':'已保存 · v')+doc.revision;
    renderInspector();renderCards();draw();toast('已取消编辑，点位恢复为已保存的内容');
}
const editStyle=document.createElement('style');editStyle.textContent=`
.point-locked input,.point-locked textarea,.point-locked select{background:#f5f7f4;color:#596e66}.point-edit-actions{display:flex;gap:8px;align-items:center;margin:12px 0 4px}.point-edit-actions button{flex:1}.point-edit-state{font-size:11px;color:#71847c;line-height:1.6;margin:6px 0}.point-edit-actions .primary{margin:0;width:auto}
`;document.head.append(editStyle);
const pointEditInspector=renderInspector;
renderInspector=function(){
    pointEditInspector();
    if(operator||tab!=='points'||!$('pName'))return;
    const p=doc.points.find(p=>p.id===$('inspector').dataset.pointId);if(!p)return;
    const editable=pointCanEdit(p),persisted=!!savedPoint(p.id),pane=$('pName').closest('.pane');pane.classList.toggle('point-locked',!editable);
    for(const id of ['pName','pX','pY','pYaw','pScript','pDwell'])$(id).readOnly=!editable;
    for(const id of ['pKind','fixedYaw','pProduct','pWait'])$(id).disabled=!editable;
    for(const id of ['applyPoint','deletePoint','currentPose','poseYaw','mapPosition','mapDirection'])$(id).classList.toggle('hidden',!editable);
    const row=document.createElement('div');row.className='point-edit-actions';
    if(!editable){const b=document.createElement('button');b.id='editPoint';b.className='primary';b.textContent='编辑点位';b.onclick=()=>beginPointEdit(p);row.append(b);}
    else if(persisted){const b=document.createElement('button');b.id='cancelPointEdit';b.textContent='取消编辑';b.onclick=()=>cancelPointEdit(p);row.append(b);}
    $('applyPoint').before(row);
    const note=document.createElement('div');note.className='point-edit-state';note.id='pointEditState';note.textContent=editable?'编辑中 · 保存后重新锁定，可拖动该点微调位置。':'已保存 · 当前只读；拖动仅平移地图，修改或删除请先点编辑。';row.after(note);
    if($('quickVisit'))$('quickVisit').nextElementSibling.textContent=editable?'前往测试会先保存当前编辑；右档、摇杆和取消可结束导航。':'直接前往已保存的位置，到点按设置讲解；无需进入编辑。';
};
