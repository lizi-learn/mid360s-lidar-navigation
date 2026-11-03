// One-map authoring. Preview requests are read-only and never use /api/confirm.
let studioPanel='point',studioAction='',studioPreview=null,studioClock=0,studioStarted=0,
    studioPlaying=false,studioFrame=null,studioRequest=0,savedMapRequest=0,studioRate=4;
const studioStyle=document.createElement('style');studioStyle.textContent=`
.studio-authoring #pointCards{display:none!important}.studio-authoring .content{gap:12px;padding:16px 24px}.studio-authoring .map-header{padding:9px 14px}.studio-authoring .map-card{min-height:380px}.studio-authoring .map-stamp{max-width:85%;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.studio-authoring #pScript{min-height:86px}.studio-authoring #pYaw{width:65px;flex:1}.studio-authoring #poseYaw,.studio-authoring #mapDirection{white-space:nowrap;flex:none;font-size:12px}.studio-bar{padding:9px 12px;border-bottom:1px solid var(--line);display:flex;align-items:center;gap:6px;flex-wrap:wrap;background:#fafcf9}.studio-bar button{font-size:12px;padding:7px 10px}.studio-bar select{width:190px;padding:7px;font-size:12px}.studio-bar .on{background:#e6f3ec;border-color:#137e79;color:#137e79}.studio-tabs{display:flex;padding:9px;gap:4px;border-bottom:1px solid var(--line);position:sticky;top:0;background:#fff;z-index:2}.studio-tabs button{flex:1;font-size:12px;padding:8px 3px}.studio-tabs .on{background:#e8f3ed;color:#137e79}.preview-strip{position:absolute;left:12px;right:58px;top:12px;z-index:3;background:#173c3af2;color:#fff;border-radius:10px;padding:12px 15px;pointer-events:none;box-shadow:0 4px 18px #18342c22}.preview-strip b{font-size:13px}.preview-strip p{font-size:12px;line-height:1.65;margin-top:5px;white-space:pre-wrap}.preview-strip small{font-size:10px;color:#b8d2ca}.studio-step{cursor:pointer}.studio-step:hover{background:#eff7f2}.studio-help{font-size:11px;line-height:1.7;color:#6e8580;padding:9px 13px;border-bottom:1px solid var(--line)}.studio-bar .push{margin-left:auto}.studio-annotation{border-bottom:1px solid var(--line);padding:10px 0}.studio-annotation input{font-size:12px}.studio-bar .preview-only{background:#f2f6ef}.studio-route-title{padding:12px 19px 0;font-size:11px;color:#70867c}
@media(max-width:900px){.studio-bar select{width:150px}.studio-bar .push{margin-left:0}}`;
document.head.append(studioStyle);
const studioBar=document.createElement('div');studioBar.id='studioBar';studioBar.className='studio-bar';
document.querySelector('.map-header').after(studioBar);
const studioNote=document.createElement('div');studioNote.className='studio-help';studioNote.id='studioHelp';studioBar.after(studioNote);
const previewStrip=document.createElement('div');previewStrip.className='preview-strip hidden';previewStrip.id='previewStrip';document.querySelector('.map-wrap').append(previewStrip);

function preferredMap(options,active){
    if(active!=='unselected'&&options.includes(active))return active;
    let saved='';try{saved=localStorage.getItem('reception-editor-map')||'';}catch(e){}
    return options.includes(saved)?saved:options.find(m=>m.endsWith('_unknownsafe'))||options.find(m=>m!=='unselected')||active;
}
const studioLoadCatalog=loadCatalog;
loadCatalog=async function(mapId){
    const id=++savedMapRequest;studioStopPreview(true);if(map?.map_id!==mapId)cacheMap(null);
    await studioLoadCatalog(mapId);const wantedRoute=new URLSearchParams(location.search).get('route');if(doc?.routes.some(r=>r.id===wantedRoute)){selectedRoute=wantedRoute;selected=doc.routes.find(r=>r.id===selectedRoute).steps[0]?.point_id||doc.points[0]?.id||null;}render();
    if(id!==savedMapRequest||doc?.map_id!==mapId)return;
    try{
        const raster=await mapCall('map_raster',{map_id:mapId});
        if(id!==savedMapRequest||doc?.map_id!==mapId)return;
        cacheMap(raster);if(raster.height>raster.width*1.8&&canvas.clientWidth>canvas.clientHeight)mapRotation=-Math.PI/2;fitMap();try{localStorage.setItem('reception-editor-map',mapId);}catch(e){}
    }catch(e){if(id===savedMapRequest){cacheMap(null);draw();toast('已保存地图读取失败：'+e.message);}}
};
const studioTouch=touch;
touch=function(){studioStopPreview(true);studioTouch();};
const studioRender=render,studioInspector=renderInspector,studioDraw=draw;
render=function(){if(operator||tab!=='points')studioStopPreview(true);document.body.classList.toggle('studio-authoring',!operator&&tab==='points');studioRender();renderStudioToolbar();};
function renderStudioToolbar(){
    const enabled=!operator&&tab==='points';studioBar.classList.toggle('hidden',!enabled);studioNote.classList.toggle('hidden',!enabled);
    if(!enabled)return;
    $('pageTitle').textContent='地图编排 · 一张图安排整段导览';
    $('pageDescription').textContent='点地图放站点，右侧填写说什么、朝向哪里、停留多久；路线与通行限制也在这里。';
    studioBar.innerHTML=`<button id="studioSelect">选择 / 拖动</button><button id="studioVisit">＋ 讲解点</button><button id="studioTransit">＋ 途经点</button><button id="studioKeepout">禁行线</button><button id="studioSpeed">减速区</button><button id="studioFinish">完成绘制</button><button id="studioUndo">撤回一笔</button><select id="studioRoute" aria-label="当前编排路线"><option value="">选择一条路线</option>${doc.routes.map(r=>`<option value="${esc(r.id)}" ${r.id===selectedRoute?'selected':''}>${esc(r.name)}</option>`).join('')}</select><button id="studioAddRoute">点选编入路线</button><button class="push preview-only" id="studioPreview">▶ 离线预演</button><button class="preview-only" id="studioStopPreview">停止预演</button>`;
    $('studioSelect').onclick=()=>{studioAction='';clearTool();mapEdit=null;renderStudioToolbar();};
    $('studioVisit').onclick=()=>studioActivate('visit');$('studioTransit').onclick=()=>studioActivate('transit');
    $('studioKeepout').onclick=()=>{studioAction='';studioPanel='limits';activateTool('keepout');renderInspector();renderStudioToolbar();};
    $('studioSpeed').onclick=()=>{studioAction='';studioPanel='limits';activateTool('speed');renderInspector();renderStudioToolbar();};
    $('studioFinish').onclick=finishShape;$('studioUndo').onclick=()=>{toolVertices.pop();draw();};
    $('studioRoute').onchange=e=>{selectedRoute=e.target.value;studioPanel='route';studioStopPreview(true);renderInspector();draw();};
    $('studioAddRoute').onclick=()=>{if(!selectedRoute)return toast('先选择或新建一条路线');studioAction='route';studioPanel='route';clearTool();renderInspector();renderStudioToolbar();};
    $('studioPreview').onclick=studioStartPreview;$('studioStopPreview').onclick=()=>studioStopPreview();
    const active=tool==='keepout'?'studioKeepout':tool==='speed'?'studioSpeed':studioAction==='visit'?'studioVisit':studioAction==='transit'?'studioTransit':studioAction==='route'?'studioAddRoute':'studioSelect';$(active).classList.add('on');
    studioNote.textContent=tool?'依次点击地图取顶点 → 完成绘制 → 保存更改。':studioAction==='route'?'按行走顺序点击地图上的点位；右侧可调整顺序、讲解与循环。':studioAction?'在地图上点一下放置新点；右侧设置介绍、停留时间和朝向。':'点站点即可编辑、拖动微调；虚线表示站点顺序，预演实线表示计算路径。';
}
function studioActivate(kind){studioStopPreview(true);studioAction=kind;studioPanel='point';clearTool();mapEdit=null;canvas.style.cursor='crosshair';renderInspector();renderStudioToolbar();}
function studioCreateAt(x,y,kind){
    const p={id:uid('point'),name:(kind==='transit'?'途经点 ':'讲解点 ')+(doc.points.length+1),x:+x.toFixed(3),y:+y.toFixed(3),yaw:null,product_id:'',script:'',dwell:0,guest_wait:false,kind};
    doc.points.push(p);selected=p.id;studioAction='';studioPanel='point';touch();render();return p;
}
renderInspector=function(){
    if(operator||tab!=='points')return studioInspector();
    if(studioPanel==='route')renderRouteInspector();else if(studioPanel==='limits')renderStudioLimits();else studioInspector();
    const tabs=document.createElement('div');tabs.className='studio-tabs';tabs.innerHTML=['point','route','limits'].map((p,i)=>`<button data-studio-panel="${p}" class="${p===studioPanel?'on':''}">${['点位设置','路线顺序','通行限制'][i]}</button>`).join('');$('inspector').prepend(tabs);
    tabs.querySelectorAll('[data-studio-panel]').forEach(b=>b.onclick=()=>{studioPanel=b.dataset.studioPanel;renderInspector();});
    if(studioPanel==='point')compactPointPanel();
    if(studioPanel==='route'){
        if($('routeSelect'))$('routeSelect').addEventListener('change',renderStudioToolbar);
        const steps=$('steps'),r=doc.routes.find(r=>r.id===selectedRoute);
        if(steps&&r)steps.querySelectorAll('.step').forEach((row,i)=>{
            const p=doc.points.find(p=>p.id===r.steps[i].point_id),button=document.createElement('button');button.textContent='编辑';button.onclick=()=>{selected=p.id;studioPanel='point';renderInspector();renderCards();draw();};row.append(button);
            const detail=document.createElement('div');detail.className='sub';detail.textContent=p.kind==='transit'?'连续途经':`${p.yaw===null?'自然朝向':Math.round(p.yaw*180/Math.PI)+'°'} · 停留${p.dwell}秒${p.guest_wait?' · 等待继续':''}`;row.querySelector('.grow').append(detail);
        });
        const add=document.createElement('button');add.className='wide';add.textContent='＋ 新建编排路线';add.style.marginTop='10px';add.onclick=()=>{const old=tab;tab='routes';addItem();tab=old;studioPanel='route';render();};$('inspector').append(add);
    }
};

function compactPointPanel(){
    const pane=document.querySelector('#inspector .pane');if(!pane||!$('pScript'))return;
    const name=$('pName'),script=$('pScript'),yaw=$('fixedYaw').closest('label'),yawRow=$('pYaw').parentElement,dwell=$('pDwell').parentElement.parentElement,apply=$('applyPoint'),buttons=$('mapPosition').parentElement;
    const nameLabel=name.previousElementSibling,scriptLabel=script.previousElementSibling,coordinates=$('pX').parentElement.parentElement,kind=$('pKind'),kindLabel=kind.previousElementSibling,product=$('pProduct'),productLabel=product.previousElementSibling;
    scriptLabel.textContent='在这个点说什么';
    const details=document.createElement('details');details.className='section-note';const summary=document.createElement('summary');summary.textContent='精确坐标、点位类型与关联产品';details.append(summary,coordinates,kindLabel,kind,productLabel,product);
    pane.replaceChildren(nameLabel,name,scriptLabel,script,yaw,yawRow,dwell,apply,buttons,details);
}

function renderStudioLimits(){
    $('inspector').innerHTML=`<div class="pane-title"><h3>通行限制 · 与点位同图保存</h3></div><div class="pane"><p class="small muted">红线禁止通过，金色区域降低输入倍率；不会擦除原雷达地图。</p><label>禁行线宽 / m</label><input id="lineWidth" type="number" min="0.02" max="2" step="0.01" value="0.10"><label>新减速区倍率</label><input id="zoneFactor" type="number" min="0.1" max="1" step="0.1" value="0.5"><div class="row" style="margin-top:10px"><button id="limitFinish">完成绘制</button><button id="limitCancel">取消绘制</button></div>${doc.annotations.map(a=>`<div class="studio-annotation"><input data-annotation-name="${esc(a.id)}" aria-label="标注名称" value="${esc(a.name)}"><div class="row spread" style="margin-top:8px"><small>${a.kind==='speed'?'减速倍率 '+a.factor:'禁行线宽 '+a.width+'m'}</small><button data-annotation-remove="${esc(a.id)}">删除标注</button></div></div>`).join('')||'<div class="section-note">点击地图上方“禁行线”或“减速区”，再依次点取顶点。</div>'}</div>`;
    $('limitFinish').onclick=finishShape;$('limitCancel').onclick=()=>{clearTool();renderStudioToolbar();};
    document.querySelectorAll('[data-annotation-name]').forEach(b=>b.oninput=()=>{doc.annotations.find(a=>a.id===b.dataset.annotationName).name=b.value;touch();});
    document.querySelectorAll('[data-annotation-remove]').forEach(b=>b.onclick=()=>{doc.annotations=doc.annotations.filter(a=>a.id!==b.dataset.annotationRemove);touch();renderInspector();draw();});
}
const studioFinishShape=finishShape;
finishShape=function(){studioFinishShape();if(tab==='points'){renderInspector();renderStudioToolbar();}};
const studioDown=canvas.onpointerdown,studioUp=canvas.onpointerup;
canvas.onpointerdown=e=>{
    if(!operator&&tab==='points'&&viewMode==='2d'&&!tool&&!mapEdit){
        const r=canvas.getBoundingClientRect(),xy=[e.clientX-r.left,e.clientY-r.top];
        const hit=studioVisiblePoints().find(p=>Math.hypot(...screen(p.x,p.y).map((v,i)=>v-xy[i]))<18);
        if(studioAction==='route'&&hit){appendStep(hit.id);renderInspector();renderStudioToolbar();return;}
        if(['visit','transit'].includes(studioAction)){const p=world(...xy);studioCreateAt(p[0],p[1],studioAction);return;}
        if(hit)studioPanel='point';
    }
    return studioDown(e);
};
canvas.onpointerup=e=>{const result=studioUp(e);if(tab==='points')renderStudioToolbar();return result;};

function studioStopPreview(clear=true){studioPlaying=false;++studioRequest;if(studioFrame!==null)cancelAnimationFrame(studioFrame);studioFrame=null;if(clear){studioPreview=null;studioClock=0;}previewStrip.classList.add('hidden');if(typeof draw==='function'&&view)draw();}
async function studioStartPreview(){
    if(!selectedRoute)return toast('先在地图上方选择一条路线');
    studioStopPreview(true);const requestId=studioRequest;
    toast('正在计算保存地图上的预演路线…');
    try{
        const result=await mapCall('route_preview',{document:doc,route_id:selectedRoute});
        if(studioRequest!==requestId)return;
        studioPreview=result;studioPlaying=true;studioClock=0;studioStarted=performance.now();
        toast('离线预演开始：等待继续将在3秒后模拟点击，不发送真机指令');
        function frame(){if(!studioPlaying)return;studioClock=Math.min(result.duration_s,(performance.now()-studioStarted)/1000*studioRate);draw();if(studioClock<result.duration_s)studioFrame=requestAnimationFrame(frame);else studioPlaying=false;}
        frame();
    }catch(e){if(studioRequest===requestId)toast(e.message);}
}
function studioSample(){
    const seq=studioPreview?.samples;if(!seq?.length)return null;let i=seq.findIndex(s=>s.t>studioClock);if(i<0)return seq[seq.length-1];if(i===0)return seq[0];
    const a=seq[i-1],b=seq[i],f=Math.min(1,(studioClock-a.t)/Math.max(.001,b.t-a.t)),delta=Math.atan2(Math.sin(b.pose[2]-a.pose[2]),Math.cos(b.pose[2]-a.pose[2]));
    return {...a,pose:[a.pose[0]+(b.pose[0]-a.pose[0])*f,a.pose[1]+(b.pose[1]-a.pose[1])*f,a.pose[2]+delta*f]};
}
function studioVisiblePoints(){const route=doc?.routes.find(r=>r.id===selectedRoute);if(operator||tab!=='points'||!route)return doc?.points||[];const ids=new Set(route.steps.map(s=>s.point_id));if(selected)ids.add(selected);return doc.points.filter(p=>ids.has(p.id));}
draw=function(){
    const points=doc?.points;if(points)doc.points=studioVisiblePoints();try{studioDraw();}finally{if(points)doc.points=points;}if(!studioPreview||viewMode!=='2d'||!view)return;
    const seq=studioPreview.samples,s=studioSample();if(!s)return;
    drawShape(seq.filter(s=>s.phase==='moving').map(s=>s.pose),'#127b77',false,2);
    const p=screen(...s.pose),half=.275*view.scale;ctx.save();ctx.translate(...p);ctx.rotate(mapRotation-s.pose[2]);ctx.fillStyle='#dc9555';ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.fillRect(-half,-half,half*2,half*2);ctx.strokeRect(-half,-half,half*2,half*2);ctx.beginPath();ctx.moveTo(Math.max(half,9)+6,0);ctx.lineTo(-2,-5);ctx.lineTo(-2,5);ctx.closePath();ctx.fillStyle='#173f3d';ctx.fill();ctx.restore();
    previewStrip.classList.remove('hidden');const names={moving:'前往下一站',turning:'调整朝向',presenting:'到点介绍',guest_wait:'等待继续 · 3秒后模拟点击',dwell:'到点停留',arrived:'到达',completed:'导览完成'};
    previewStrip.innerHTML=`<b>离线预演 · ${esc(s.point)} · ${names[s.phase]||s.phase}</b><p>${esc(s.script||((s.phase==='moving'&&s.factor<1)?'正在通过减速区 · '+Math.round(s.factor*100)+'%':''))}</p><small>${Math.round(studioClock)} / ${Math.round(studioPreview.duration_s)} 秒 · 播放×${studioRate} · 路径与任务示意，非步态仿真</small>`;
};
