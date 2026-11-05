// Map tools share the same request transport as G12; no new gateway paths.
let labelRects=[],mapRotation=0,tool=null,toolVertices=[],correction=null,updateRegion=null,
    mapVersions=[],scanVisible=true,replayRuns=[],replaySamples=[],replayEvents=[],replayIndex=0,
    replayPlaying=false,replayClock=null,replayMapId='',selectedReplayRun='',replayLoadGeneration=0,nearbyPoses=[],selectedDock=null;
const mapCall=(operation,data={})=>request('/api/save',{reception:{operation,...data}});
const originalCatalogLoad=loadCatalog;
loadCatalog=async function(mapId){if(doc?.map_id!==mapId){temporaryTarget=null;tool=null;toolVertices=[];correction=null;updateRegion=null;mapVersions=[];replayRuns=[];replaySamples=[];replayEvents=[];replayIndex=0;replayPlaying=false;replayMapId=mapId;selectedReplayRun='';replayLoadGeneration++;selectedDock=null;nearbyPoses=[];}return originalCatalogLoad(mapId);};
const originalRender=render,originalInspector=renderInspector,originalDraw=draw,originalStatus=renderStatus;
const voiceChip=document.createElement('span');voiceChip.className='chip';voiceChip.id='voiceStatus';$('revisionBadge').after(voiceChip);
renderStatus=function(){originalStatus();const v=state?.reception?.voice_progress;voiceChip.textContent=!v?.configured?'介绍：文字模式':v.busy?'音频准备 '+v.ready+'/'+v.total:v.failed?'音频 '+v.failed+' 段未生成':v.total?'音频已备好 '+v.ready+' 段':'';const m=state?.reception?.map_observations,active=doc?.map_id===state?.reception?.map_id;if($('observedMapStatus'))$('observedMapStatus').textContent=!active?'载入这张地图后可学习':m?.error?'地图学习暂不可用，固定底图保留':m?`${m.enabled?'低频学习中':'学习已暂停'} · 待融合 ${m.pending_cells||0} 格 · 已融合 ${m.cleared_cells} 格${m.merge_requested?' · 已安排空闲时融合':''}`:'等待地图观测';if($('autoMapUpdate')){$('autoMapUpdate').disabled=!active;if(m&&active)$('autoMapUpdate').checked=m.enabled;}if($('resetObservedMap'))$('resetObservedMap').disabled=!active;if($('mergeObservedMap'))$('mergeObservedMap').disabled=!active;};
render=function(){
    if(['maps','replay'].includes(tab)&&!operator){
        if(!doc)return;
        document.querySelectorAll('[data-tab]').forEach(b=>b.classList.toggle('active',b.dataset.tab===tab));
        $('pageTitle').textContent=tab==='maps'?'让地图始终贴近现场':'回看实际走过的每一段';
        $('pageDescription').textContent=tab==='maps'?'激光叠加、位置校正、续扫、禁行线与减速区，统一在地图上操作。':'轨迹、输入、回波与失败事件使用各自原始时间对齐。';
        $('addButton').classList.add('hidden');$('pointCards').classList.add('hidden');
        $('tableView').classList.add('hidden');$('mapBoard').classList.remove('hidden');renderInspector();draw();return;
    }
    originalRender();
};
renderInspector=function(){
    if(!doc)return;
    if(!operator&&tab==='maps'){renderMapTools();return;}
    if(!operator&&tab==='replay'){renderReplay();return;}
    originalInspector();
    if(operator){
        const box=document.querySelector('#inspector .pane');
        if(!box)return;
        box.insertAdjacentHTML('beforeend','<button class="wide" id="goHome" style="margin-top:8px">一键返回出发点</button><details style="margin-top:14px"><summary>目标被占用？附近停靠</summary><p class="small muted">先查看附近候选，再选择停靠位置。原点位保持原样。</p><button id="nearbyButton">查看附近停靠点</button><div id="nearbyOptions"></div></details>');
        const homeButton=$('goHome');document.querySelector('.mission-actions')?.append(homeButton);homeButton.disabled=!!state?.reception?.mission&&['dispatching','navigating','rerouting','waiting_route','presenting','guest_wait','dwell'].includes(state.reception.mission.phase);
        $('goHome').onclick=async()=>{const p=doc.points.find(p=>p.kind==='home');if(!p)return toast('先在管理页保存一个出发 / 返回点');selected=p.id;temporaryTarget=null;runKind='point';selectedDock=null;await missionCommand('start');};
        $('nearbyButton').onclick=loadNearby;
    }else if(tab==='points'){
        const box=document.querySelector('#inspector .pane');if(box){box.insertAdjacentHTML('beforeend','<label>常用点位模板</label><div class="row"><button data-common="出发点">出发点</button><button data-common="办公室另一端">另一端</button><button data-common="会议室">会议室</button></div>');box.querySelectorAll('[data-common]').forEach(b=>b.onclick=()=>{const name=b.dataset.common;addItem();const p=doc.points.find(p=>p.id===selected);p.name=name;p.kind=name==='出发点'?'home':'visit';touch();renderInspector();renderCards();toast('已添加 '+name+' 草稿，可用当前位置或地图取点');});}
    }else if(tab==='routes'){
        const box=document.querySelector('#inspector .pane');
        if(box){box.insertAdjacentHTML('beforeend','<button class="wide" id="makeRoundTrip" style="margin-top:12px">把当前路线变为往返</button><p class="small muted">A→B→C 转为 A→B→C→B→A，保留停留与讲解设置。</p>');
            $('makeRoundTrip').onclick=()=>{const r=doc.routes.find(r=>r.id===selectedRoute);if(!r||r.steps.length<2)return toast('路线至少需要两个点');r.steps.push(...r.steps.slice(0,-1).reverse().map(s=>({...s})));touch();renderInspector();draw();};}
    }
};
function renderMapTools(){
    $('inspector').innerHTML=`<div class="pane-title"><h3>地图工具</h3></div><div class="pane">
    <label class="inline-check"><input type="checkbox" id="scanVisible" ${scanVisible?'checked':''}>实时激光叠加</label>
    <label class="inline-check"><input type="checkbox" id="autoMapUpdate" ${state?.reception?.map_observations?.enabled!==false?'checked':''}>低频学习 · 每日融合</label>
    <p class="small muted" id="observedMapStatus"></p><p class="small muted">平时使用固定底图，人群只参与实时避障。路过发现旧位置已扫通时低频记录，每24小时在导航空闲时统一融合；也可手动安排融合。原底图和版本保留。</p>
    <button class="quiet wide" id="mergeObservedMap">空闲时融合已观测变化</button>
    <button class="quiet wide" id="resetObservedMap">恢复原底图并暂停更新</button>
    <div class="grid2"><button id="correctPose">拖动校正位置</button><button id="rotateMap">地图转 90°</button></div>
    <div class="section-note" id="toolNote">拖动校正：从真实位置拖向机身朝向，激光副本随之移动；提交后只更新定位。</div>
    <div class="row" style="margin-top:8px"><button id="applyCorrection" ${correction?'':'disabled'}>应用校正</button><button id="cancelTool">取消操作</button></div>
    <label>通行标注</label><div class="grid2"><button id="drawKeepout">绘制禁行线</button><button id="drawSpeed">绘制减速区</button></div>
    <div class="grid2"><div><label>线宽 / m</label><input id="lineWidth" type="number" min="0.02" max="2" step="0.01" value="0.10"></div><div><label>区域速度倍率</label><input id="zoneFactor" type="number" min="0.1" max="1" step="0.1" value="0.5"></div></div>
    <button class="wide" id="finishShape" style="margin-top:8px">完成绘制</button>
    <div style="margin-top:12px">${(doc.annotations||[]).map(a=>`<div class="row spread small" style="margin:8px 0"><span>${esc(a.name)} · ${a.kind==='speed'?Math.round(a.factor*100)+'%':'禁行'}</span><button data-remove-annotation="${esc(a.id)}">移除</button></div>`).join('')}</div>
    <label>续扫与地图版本</label><button id="loadMap" class="wide" style="margin-bottom:8px">载入这张导航地图</button><div class="grid2"><button id="resumeMapping">继续扫描</button><button id="drawRegion">选择局部更新区</button></div>
    <button id="saveMapping" class="wide" style="margin-top:8px">保存本轮地图</button>
    <p class="small muted" style="margin-top:8px">续扫使用已保存的位姿图。局部更新保留选区之外的原始栅格；保存后重新载入导航地图。</p>
    <div class="row" style="margin-top:10px"><button id="readVersions">查看版本</button><button id="snapshotMap">保存版本</button></div>
    <div id="mapVersions">${mapVersions.map(v=>`<div class="row spread small" style="margin-top:8px"><span>${new Date(v.wall_time*1000).toLocaleString()}<br>${esc(v.reason)}</span><button data-restore-map="${esc(v.id)}">恢复</button></div>`).join('')}</div>
    <label>换机与备份</label><button id="exportSite" class="wide">导出地图＋点位＋路线</button>
    <input id="siteName" placeholder="导入为新地图名称，例如 office_1005" style="margin-top:8px"><button id="importSite" class="wide" style="margin-top:8px">导入场地包</button><input id="siteFile" type="file" accept=".zip" hidden>
    <p class="small muted" id="transferProgress">地图、位姿图、产品话术和通行标注同包校验。换机不复制旧机器的标定和实时定位。</p></div>`;
    $('scanVisible').onchange=e=>{scanVisible=e.target.checked;draw();};
    $('autoMapUpdate').onchange=async e=>{try{await mapCall('map_observations',{map_id:doc.map_id,enabled:e.target.checked});await refresh();}catch(error){e.target.checked=!e.target.checked;toast(error.message);}};
    $('mergeObservedMap').onclick=async()=>{try{await mapCall('map_observations_merge',{map_id:doc.map_id});toast('已安排在导航空闲时融合，不中断当前任务');await refresh();}catch(error){toast(error.message);}};
    $('resetObservedMap').onclick=async()=>{try{await mapCall('map_observations_reset',{map_id:doc.map_id});toast('已保留版本，恢复原底图并暂停自动更新');await refresh();}catch(error){toast(error.message);}};
    renderStatus();
    $('rotateMap').onclick=()=>{mapRotation=(mapRotation+Math.PI/2)%(2*Math.PI);fitMap();};
    $('correctPose').onclick=()=>activateTool('correct');$('cancelTool').onclick=clearTool;
    $('drawKeepout').onclick=()=>activateTool('keepout');$('drawSpeed').onclick=()=>activateTool('speed');$('drawRegion').onclick=()=>activateTool('region');
    $('applyCorrection').onclick=async()=>{if(!correction)return;try{await mapCall('pose_correct',{map_id:doc.map_id,pose:{x:correction[0],y:correction[1],yaw:correction[2]}});clearTool();toast('已提交位置与朝向，正在更新定位');}catch(e){toast(e.message);}};
    $('finishShape').onclick=finishShape;
    document.querySelectorAll('[data-remove-annotation]').forEach(b=>b.onclick=()=>{doc.annotations=doc.annotations.filter(a=>a.id!==b.dataset.removeAnnotation);touch();renderMapTools();draw();});
    $('readVersions').onclick=async()=>{try{mapVersions=(await mapCall('map_versions',{map_id:doc.map_id})).versions;renderMapTools();}catch(e){toast(e.message);}};
    $('snapshotMap').onclick=async()=>{try{await mapCall('map_snapshot',{map_id:doc.map_id});$('readVersions').click();}catch(e){toast(e.message);}};
    document.querySelectorAll('[data-restore-map]').forEach(b=>b.onclick=async()=>{try{await mapCall('map_restore',{map_id:doc.map_id,version:b.dataset.restoreMap});toast('地图版本已恢复，当前导航停止；可重新载入地图');await refresh();}catch(e){toast(e.message);}});
    $('loadMap').onclick=loadEditedNavigationMap;
    $('resumeMapping').onclick=async()=>{try{await mapCall('map_resume',{map_id:doc.map_id,...(updateRegion?{region:updateRegion}:{})});toast(updateRegion?'已启动所选区域续扫':'已从保存的位姿图继续扫描');}catch(e){toast(e.message);}};
    $('saveMapping').onclick=async()=>{try{const r=await mapCall('map_save',{map_id:doc.map_id});toast('地图已保存'+(r.local_update?' · 局部更改 '+r.local_update.changed_cells+' 格':''));}catch(e){toast(e.message);}};
    $('exportSite').onclick=exportSite;$('importSite').onclick=()=>$('siteFile').click();$('siteFile').onchange=importSite;
}
function activateTool(kind){viewMode='2d';mapEdit=null;tool=kind;toolVertices=[];correction=null;canvas.style.cursor='crosshair';$('mapHint').textContent=kind==='correct'?'从真实位置按下并拖向朝向':'依次点击地图取顶点，点「完成绘制」结束';draw();}
function clearTool(){tool=null;toolVertices=[];correction=null;canvas.style.cursor='';$('mapHint').textContent='点击选点 · 滚轮缩放 · 拖动空白平移';draw();if(tab==='maps')renderMapTools();}
function finishShape(){
    if(!['keepout','speed','region'].includes(tool))return toast('先选择绘图工具');
    if(toolVertices.length<(tool==='keepout'?2:3))return toast('顶点还不够，请继续在地图上点击');
    if(tool==='region'){updateRegion=toolVertices.map(p=>p.slice());toast('已选择局部更新区，点「继续扫描」开始');}
    else{const a={id:uid('annotation'),name:tool==='speed'?'减速区':'禁行线',kind:tool,vertices:toolVertices.map(p=>p.map(v=>+v.toFixed(3)))};if(tool==='speed')a.factor=+$('zoneFactor').value;else a.width=+$('lineWidth').value;(doc.annotations??=[]).push(a);touch();toast('标注已加入草稿，保存更改后参与导航');}
    clearTool();
}
screen=function(x,y){const dx=(x-view.x-view.width/2)*view.scale,dy=(view.y+view.height/2-y)*view.scale,c=Math.cos(mapRotation),s=Math.sin(mapRotation);return [canvas.clientWidth/2+pan[0]+c*dx-s*dy,canvas.clientHeight/2+pan[1]+s*dx+c*dy];};
world=function(x,y){const dx=x-canvas.clientWidth/2-pan[0],dy=y-canvas.clientHeight/2-pan[1],c=Math.cos(mapRotation),s=Math.sin(mapRotation);return [view.x+view.width/2+(c*dx+s*dy)/view.scale,view.y+view.height/2-(-s*dx+c*dy)/view.scale];};
draw=function(){labelRects=[];originalDraw();if(!view||viewMode==='3d')return;
    for(const a of doc?.annotations||[])drawShape(a.vertices,a.kind==='speed'?'#d19840':'#bd5a56',a.kind==='speed',a.kind==='keepout'?Math.max(3,a.width*view.scale):2);
    if(updateRegion)drawShape(updateRegion,'#657fba',true,2);
    if(toolVertices.length)drawShape(toolVertices,'#b66f40',false,3);
    const sample=tab==='replay'&&replayMapId===doc?.map_id?replaySamples[replayIndex]:null,scan=sample?.scan||state?.scan_overlay;
    const scanAge=sample?(scan?.age_s??999):(scan?.age_s??999);
    if(scanVisible&&scan&&scanAge>=-.05&&scanAge<=.5){ctx.fillStyle=correction?'#d98b52':'#279b91';for(const p of scan.points||[]){let q=p;if(correction&&scan.origin){const dx=p[0]-scan.origin[0],dy=p[1]-scan.origin[1],a=correction[2]-scan.origin[2];q=[correction[0]+Math.cos(a)*dx-Math.sin(a)*dy,correction[1]+Math.sin(a)*dx+Math.cos(a)*dy];}const xy=screen(...q);ctx.fillRect(xy[0]-1,xy[1]-1,2,2);}}
    if(correction){const p=screen(...correction);ctx.strokeStyle='#d98b52';ctx.beginPath();ctx.arc(...p,10,0,Math.PI*2);ctx.stroke();}
    if(sample){const line=replaySamples.slice(0,replayIndex+1).filter(s=>s.pose&&s.pose_valid).map(s=>s.pose.slice(0,2));drawShape(line,'#db8e46',false,3);if(sample.pose){const xy=screen(...sample.pose);ctx.fillStyle=sample.pose_valid?'#d48545':'#888';ctx.beginPath();ctx.arc(...xy,7,0,Math.PI*2);ctx.fill();}}
};
function drawShape(vertices,color,closed,width){if(!vertices.length)return;ctx.strokeStyle=color;ctx.fillStyle=color+'22';ctx.lineWidth=width;ctx.beginPath();vertices.forEach((p,i)=>{const q=screen(...p);i?ctx.lineTo(...q):ctx.moveTo(...q);});if(closed){ctx.closePath();ctx.fill();}ctx.stroke();}
const oldDown=canvas.onpointerdown,oldMove=canvas.onpointermove,oldUp=canvas.onpointerup;
canvas.onpointerdown=e=>{if(!tool)return oldDown(e);const r=canvas.getBoundingClientRect(),p=world(e.clientX-r.left,e.clientY-r.top);canvas.setPointerCapture(e.pointerId);pointer={toolStart:p};if(tool==='correct')correction=[...p,state?.pose?.[2]||0];};
canvas.onpointermove=e=>{if(!tool)return oldMove(e);if(tool==='correct'&&pointer?.toolStart){const r=canvas.getBoundingClientRect(),p=world(e.clientX-r.left,e.clientY-r.top);correction=[...pointer.toolStart,Math.atan2(p[1]-pointer.toolStart[1],p[0]-pointer.toolStart[0])];draw();}};
canvas.onpointerup=e=>{if(!tool)return oldUp(e);if(!pointer?.toolStart)return;if(tool==='correct'){pointer=null;renderMapTools();draw();}else{toolVertices.push(pointer.toolStart);pointer=null;draw();}};
async function loadNearby(){try{if(runKind!=='point')return toast('先切换到单个目的地并选择点位');const p=temporaryTarget||doc.points.find(p=>p.id===selected);if(!p)return toast('先选择目的地');nearbyPoses=(await mapCall('nearby',{target:{x:p.x,y:p.y,yaw:p.yaw||0},radius:1.})).candidates;$('nearbyOptions').innerHTML=nearbyPoses.map((p,i)=>`<button class="wide" data-dock="${i}" style="margin-top:6px">停靠候选 ${i+1} · 距原目标 ${p.distance.toFixed(2)}m</button>`).join('')||'<p class="small">附近没有完整包络可用的候选</p>';$('nearbyOptions').querySelectorAll('[data-dock]').forEach(b=>b.onclick=()=>{selectedDock=nearbyPoses[+b.dataset.dock];toast('已选择附近停靠，点「开始导览」后规划前往；原点位不变');});}catch(e){toast(e.message);}}
function downloadBlob(data,name,type){const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([data],{type}));a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);}
async function exportSite(){let transfer;try{if(dirty&&!(await saveAll()))return;const meta=await mapCall('bundle_export',{map_id:doc.map_id});transfer=meta.transfer_id;const parts=[];for(let i=0;i<meta.chunks;i++){const r=await mapCall('bundle_chunk',{transfer_id:transfer,index:i});parts.push(Uint8Array.from(atob(r.data),c=>c.charCodeAt(0)));if($('transferProgress'))$('transferProgress').textContent=`导出中 ${i+1}/${meta.chunks}`;}const blob=new Blob(parts);if(await sha256Bytes(new Uint8Array(await blob.arrayBuffer()))!==meta.sha256)throw Error('下载场地包总摘要不一致，请重试');downloadBlob(blob,doc.map_id+'-site.zip','application/zip');toast('场地包已导出，含地图、点位、路线和文件摘要');}catch(e){toast(e.message);}finally{if(transfer)await mapCall('bundle_abort',{transfer_id:transfer}).catch(()=>{});}}
async function importSite(e){let transfer;try{const f=e.target.files[0],name=$('siteName').value.trim();if(!f)return;if(f.size>64*1024*1024)throw Error('场地包超过64MiB');if(!/^[A-Za-z0-9_-]{1,48}$/.test(name))throw Error('请输入新的地图名称，仅字母、数字、短横线和下划线');const raw=new Uint8Array(await f.arrayBuffer());const digest=await sha256Bytes(raw);const r=await mapCall('bundle_begin',{new_map_id:name,bytes:raw.length,sha256:digest});transfer=r.transfer_id;for(let i=0;i<r.chunks;i++){const b=raw.slice(i*4096,(i+1)*4096);await mapCall('bundle_part',{transfer_id:transfer,index:i,data:btoa(String.fromCharCode(...b))});if($('transferProgress'))$('transferProgress').textContent=`导入中 ${i+1}/${r.chunks}`;}await mapCall('bundle_commit',{transfer_id:transfer});transfer=null;toast('场地包已校验导入，未复制机器标定和实时定位');await refresh();await loadCatalog(name);}catch(e){toast(e.message);}finally{if(transfer)await mapCall('bundle_abort',{transfer_id:transfer}).catch(()=>{});}}
async function loadRuns(){try{const mapId=doc.map_id,rows=(await mapCall('trace_runs',{map_id:mapId})).runs;if(doc.map_id!==mapId)return;replayMapId=mapId;replayRuns=rows;renderReplay();}catch(e){toast(e.message);}}
function renderReplay(){const s=replaySamples[replayIndex];$('inspector').innerHTML=`<div class="pane-title"><h3>任务回放</h3></div><div class="pane"><button id="loadRuns" class="wide">刷新任务列表</button><label>选择已记录任务</label><select id="replayRun"><option value="">选择任务</option>${replayRuns.map(r=>`<option value="${esc(r.run_id)}" ${r.run_id===selectedReplayRun?'selected':''}>${new Date(r.start*1000).toLocaleString()} · ${r.samples} 帧</option>`).join('')}</select><button id="loadTrace" class="wide" style="margin-top:8px">载入轨迹</button><label>时间轴 · ${replaySamples.length} 帧</label><input id="replaySlider" type="range" min="0" max="${Math.max(0,replaySamples.length-1)}" value="${replayIndex}"><div class="row"><button id="playReplay">${replayPlaying?'暂停回放':'播放回放'}</button><button id="exportTrace">导出记录</button></div><div class="section-note" id="replayDetail">${s?traceDetail(s):'选择任务后查看真实采样轨迹、导航输入和障碍回波。回放不会控制机器人。'}</div><p class="small muted">5Hz 采样，原始定位、扫描和控制源时间分开保留；没有收到的数据不会补成有效值。</p></div>`;
    $('replayRun').onchange=e=>selectedReplayRun=e.target.value;$('loadRuns').onclick=loadRuns;$('loadTrace').onclick=()=>loadTrace($('replayRun').value);$('replaySlider').oninput=e=>{replayIndex=+e.target.value;updateReplay();};$('playReplay').onclick=()=>{replayPlaying=!replayPlaying;$('playReplay').textContent=replayPlaying?'暂停回放':'播放回放';};$('exportTrace').onclick=()=>downloadBlob(JSON.stringify({map_id:doc.map_id,samples:replaySamples,events:replayEvents},null,2),doc.map_id+'-task.json','application/json');
}
function traceDetail(s){return esc(`${new Date(s.wall_time*1000).toLocaleTimeString()}\n任务：${s.mission_phase} / ${s.session_phase}\n输入：${JSON.stringify(s.native?.velocity||s.gate?.command||null)}\n停车：${s.collision_stop?'是':'否'}\n原因：${s.gate?.reason||s.session_notice||'—'}\n定位源：${s.pose_source_ros_ns??'缺失'}\n控制源：${s.gate?.evaluated_monotonic_ns??'缺失'}`);}
function updateReplay(){if($('replaySlider'))$('replaySlider').value=replayIndex;if($('replayDetail')&&replaySamples[replayIndex])$('replayDetail').innerHTML=traceDetail(replaySamples[replayIndex]);draw();}
async function loadTrace(run){if(!run)return;const mapId=doc.map_id,generation=++replayLoadGeneration;try{const samples=[];let events=[],after=0;do{const r=await mapCall('trace_read',{map_id:mapId,run_id:run,after,limit:200});if(doc.map_id!==mapId||generation!==replayLoadGeneration)return;samples.push(...r.samples);events=r.events;after=r.after;}while(after&&samples.length<25000);replayMapId=mapId;selectedReplayRun=run;replaySamples=samples;replayEvents=events;replayIndex=0;renderReplay();draw();}catch(e){toast(e.message);}}

replayClock=setInterval(()=>{if(tab==='replay'&&replayPlaying&&!document.hidden){replayIndex=Math.min(replayIndex+1,replaySamples.length-1);if(replayIndex>=replaySamples.length-1)replayPlaying=false;updateReplay();}},200);

function pointLabel(name,x,y,active){
 const width=ctx.measureText(name).width+8,height=16;
 const choices=[[18,-16],[18,22],[-width-18,-16],[-width-18,22],[18,-36],[18,42],[-width-18,-36],[-width-18,42]];
 let best=null,score=Infinity;
 for(const [dx,dy]of choices){const r={x:x+dx,y:y+dy-height/2,w:width,h:height};let cost=0;for(const b of labelRects)cost+=Math.max(0,Math.min(r.x+r.w,b.x+b.w)-Math.max(r.x,b.x))*Math.max(0,Math.min(r.y+r.h,b.y+b.h)-Math.max(r.y,b.y));if(r.x<8||r.y<8||r.x+r.w>canvas.clientWidth-8||r.y+r.h>canvas.clientHeight-8)cost+=100000;if(cost<score){score=cost;best=r;}if(cost===0)break;}
 if(best){labelRects.push(best);ctx.fillStyle='#f9fbf6d9';ctx.fillRect(best.x-2,best.y-1,best.w,best.h);ctx.fillStyle=active?'#137e79':'#38594d';ctx.fillText(name,best.x+2,best.y+best.h/2);}
}

const handsetMapCache=cacheMap;let handsetFitted=false;
cacheMap=function(value){handsetMapCache(value);if(!handsetFitted&&value&&document.body.classList.contains('handheld-native')){handsetFitted=true;if(value.height>value.width*1.5&&innerWidth>innerHeight){mapRotation=Math.PI/2;fitMap();}}};
// Pinch zoom composes with the existing point-editing and map tools. A pinch
// never becomes a map click when the remaining finger is lifted.
const touchPointers=new Map();let pinch=null,pinchEnding=false;
const handsetDown=canvas.onpointerdown,handsetMove=canvas.onpointermove,handsetUp=canvas.onpointerup,handsetCancel=canvas.onpointercancel;
canvas.onpointerdown=e=>{if(e.pointerType!=='touch'||tool)return handsetDown(e);touchPointers.set(e.pointerId,[e.clientX,e.clientY]);if(touchPointers.size===2){const p=[...touchPointers.values()];pinch={distance:Math.hypot(p[0][0]-p[1][0],p[0][1]-p[1][1]),zoom};pointer=null;pinchEnding=true;canvas.setPointerCapture(e.pointerId);return;}if(touchPointers.size>2)return;handsetDown(e);};
canvas.onpointermove=e=>{if(touchPointers.has(e.pointerId))touchPointers.set(e.pointerId,[e.clientX,e.clientY]);if(pinch&&touchPointers.size===2){const p=[...touchPointers.values()];zoom=Math.min(20,Math.max(.5,pinch.zoom*Math.hypot(p[0][0]-p[1][0],p[0][1]-p[1][1])/Math.max(1,pinch.distance)));draw();return;}if(pinchEnding)return;handsetMove(e);};
canvas.onpointerup=e=>{touchPointers.delete(e.pointerId);if(pinchEnding){pointer=null;pinch=null;if(!touchPointers.size)pinchEnding=false;return;}handsetUp(e);};
canvas.onpointercancel=e=>{touchPointers.delete(e.pointerId);pinch=null;pointer=null;pinchEnding=touchPointers.size>0;if(handsetCancel)handsetCancel(e);};

const destinationDraw=draw;draw=function(){destinationDraw();if(temporaryTarget&&viewMode==='2d'&&view){const p=screen(temporaryTarget.x,temporaryTarget.y);ctx.strokeStyle='#ba7843';ctx.lineWidth=3;ctx.beginPath();ctx.arc(p[0],p[1],12,0,2*Math.PI);ctx.moveTo(p[0]-18,p[1]);ctx.lineTo(p[0]+18,p[1]);ctx.moveTo(p[0],p[1]-18);ctx.lineTo(p[0],p[1]+18);ctx.stroke();}};
