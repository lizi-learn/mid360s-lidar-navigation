/* Synthetic workflow preview. No hardware endpoints, cloud API or Nav2 execution. */
const svg=document.getElementById('demo-map'), ns='http://www.w3.org/2000/svg';
const stops=[{name:'接待起点',x:95,y:250,say:'您好，欢迎参观。我们从这里开始。',dwell:1},
 {name:'产品展示',x:275,y:95,say:'这里是示例产品介绍，实际话术在管理地图上编辑。',dwell:2},
 {name:'演示区',x:480,y:115,say:'您可以在这里观看演示。准备好后，点击下一站。',wait:true},
 {name:'返回接待',x:95,y:250,say:'本次参观结束，谢谢。',dwell:1}];
let state='idle',index=0,position={x:95,y:250},target=[],timer=0,blocked=false,delay=0,pausedState='running';
const route=document.getElementById('demo-route'),robot=document.getElementById('demo-robot'),status=document.getElementById('demo-status'),say=document.getElementById('demo-speech'),log=document.getElementById('demo-log');
function note(text){const p=document.createElement('div');p.textContent=text;log.prepend(p);while(log.children.length>15)log.lastChild.remove();}
function render(){
 robot.setAttribute('transform',`translate(${position.x},${position.y})`);
 const labels={idle:'选择路线后开始预览',running:`前往 ${stops[index].name}`,blocked:'前方被挡 · 演示停车等待',paused:'已暂停 · 主动继续后恢复',presenting:'已到点 · 播放介绍（文字演示）',waiting:'等待访客 · 请点下一站',dwelling:'到点停留',succeeded:'路线完成 · 已停车'};
 status.textContent=labels[state];document.getElementById('demo-station').textContent=stops[index]?.name??'接待起点';
 document.getElementById('demo-step').textContent=`${Math.min(index+1,stops.length)} / ${stops.length}`;
 document.getElementById('demo-next').disabled=state!=='waiting';document.getElementById('demo-start').textContent=state==='paused'?'继续预览':'开始预览';
 document.getElementById('demo-obstacle').setAttribute('aria-pressed',String(blocked));document.getElementById('obstacle').toggleAttribute('hidden',!blocked);
 const shown=[position,...target];route.setAttribute('points',shown.map(p=>`${p.x},${p.y}`).join(' '));
}
function plan(){
 // Authored synthetic route, not a collision/physical simulation.
 const destination=stops[index];target=[];
 if(index===1)target.push({x:190,y:250},{x:190,y:95});
 if(index===2)target.push({x:275,y:250},{x:480,y:250});
 if(index===3)target.push({x:480,y:250});
 target.push({x:destination.x,y:destination.y});state='running';say.textContent='行进中，到点后显示介绍。';note(`前往：${destination.name}`);render();
}
function next(){if(index===stops.length-1){state='succeeded';note('整条合成路线已完成');render();return;}index++;plan();}
function reset(){clearInterval(timer);timer=0;state='idle';index=0;position={x:95,y:250};target=[];blocked=false;say.textContent='本页只演示交互，不播放声音、不连接真实机器人。';log.replaceChildren();render();}
function tick(){
 if(blocked&&state==='running'){state='blocked';note('合成障碍：速度归零，等待解除');render();return;}
 if(!blocked&&state==='blocked'){state='running';note('合成障碍已解除，继续原路线');}
 if(state==='running'&&target.length){const goal=target[0],dx=goal.x-position.x,dy=goal.y-position.y,length=Math.hypot(dx,dy),step=2.2;
  if(length<=step){position={...goal};target.shift();if(!target.length){state='presenting';say.textContent=stops[index].say;delay=35;note('到点：'+stops[index].name);}}
  else{position.x+=step*dx/length;position.y+=step*dy/length;robot.querySelector('path').setAttribute('transform',`rotate(${Math.atan2(dy,dx)*180/Math.PI+90})`);}}
 else if(state==='presenting'&&--delay<=0){if(stops[index].wait)state='waiting';else{state='dwelling';delay=stops[index].dwell*20;}}
 else if(state==='dwelling'&&--delay<=0)next();render();
}
document.getElementById('demo-start').addEventListener('click',()=>{if(state==='paused'){state=pausedState;note('主动继续');}else if(state==='idle'||state==='succeeded'){reset();plan();}if(!timer)timer=setInterval(tick,50);render();});
document.getElementById('demo-pause').addEventListener('click',()=>{if(['running','blocked','presenting','dwelling'].includes(state)){pausedState=state;state='paused';note('用户暂停，保持位置');render();}});
document.getElementById('demo-next').addEventListener('click',()=>{if(state==='waiting')next();});
document.getElementById('demo-reset').addEventListener('click',reset);
document.getElementById('demo-obstacle').addEventListener('click',()=>{blocked=!blocked;render();});
for(const [i,stop]of stops.entries()){
 if(i===3)continue;
 const g=document.createElementNS(ns,'g');g.setAttribute('class','station');g.setAttribute('tabindex','0');g.setAttribute('role','button');g.setAttribute('aria-label','查看'+stop.name);
 g.innerHTML=`<circle cx="${stop.x}" cy="${stop.y}" r="14" fill="#08766c"/><text x="${stop.x}" y="${stop.y+4}" text-anchor="middle" font-size="11" fill="white">${i+1}</text><text x="${stop.x+20}" y="${stop.y+4}" font-size="11" fill="#183934">${stop.name}</text>`;
 const select=()=>{say.textContent=stop.say;document.getElementById('demo-station').textContent=stop.name;};g.addEventListener('click',select);g.addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' ')select();});svg.append(g);
}
reset();
