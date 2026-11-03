// Explicit web test controls reuse the normal mission API and manual takeover.
let quickMapEnabled=false;
const quickStyle=document.createElement('style');quickStyle.textContent=`
.control-state{display:flex;gap:5px;align-items:center;flex-wrap:wrap;justify-content:flex-end}.control-state .chip{font-size:11px}.chip.offline{background:#edf0ef;color:#71807b}#mapNavigate{white-space:nowrap}#quickVisit{margin-top:8px;width:100%}.quick-visit-note{font-size:11px;line-height:1.6;color:#71847c;margin-top:7px}@media(max-width:700px){.topbar .control-state{order:3;width:100%;justify-content:flex-start}}.handheld-native #mapNavigate{font-size:11px;padding:4px 7px;min-height:24px}
`;
document.head.append(quickStyle);
const controlState=document.createElement('div');controlState.className='control-state';controlState.id='controlState';controlState.role='status';controlState.innerHTML='<span class="chip offline" id="gaitBadge">策略未连接</span><span class="chip offline" id="automationBadge">E 档位未连接</span>';$('mapSelect').after(controlState);
const mapNavigate=document.createElement('button');mapNavigate.id='mapNavigate';mapNavigate.type='button';mapNavigate.textContent='点图导航';$('mapHint').after(mapNavigate);
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
