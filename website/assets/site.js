const menu=document.querySelector('.menu'), navigation=document.querySelector('.nav');
menu?.addEventListener('click',()=>{const on=navigation.classList.toggle('open');menu.setAttribute('aria-expanded',String(on));});
for(const pre of document.querySelectorAll('pre')){
 const wrap=document.createElement('div');wrap.className='codeblock';pre.before(wrap);wrap.append(pre);
 const button=document.createElement('button');button.className='copy';button.textContent='复制';button.type='button';button.setAttribute('aria-label','复制代码');
 button.addEventListener('click',async()=>{try{await navigator.clipboard.writeText(pre.textContent);button.textContent='已复制';setTimeout(()=>button.textContent='复制',1500);}catch{button.textContent='请选择代码复制';}});wrap.append(button);
}
for(const group of document.querySelectorAll('[data-tabs]')){
 const buttons=group.querySelectorAll('[role=tab]');
 for(const button of buttons)button.addEventListener('click',()=>{
   for(const b of buttons){const active=b===button;b.setAttribute('aria-selected',String(active));document.getElementById(b.getAttribute('aria-controls')).hidden=!active;}
 });
}
let searchIndex;
const input=document.getElementById('search'), results=document.getElementById('search-results');
input?.addEventListener('input',async()=>{
 const query=input.value.trim().toLowerCase();results.replaceChildren();results.hidden=!query;if(!query)return;
 if(!searchIndex)searchIndex=await fetch('search-index.json').then(r=>r.json());
 const hits=searchIndex.filter(x=>(x.title+' '+x.text).toLowerCase().includes(query)).slice(0,9);
 for(const hit of hits){const a=document.createElement('a');a.href=hit.url;a.textContent=hit.title;const small=document.createElement('small');small.textContent=hit.section;a.append(small);results.append(a);}
 if(!hits.length){const p=document.createElement('div');p.className='emptysearch';p.textContent='没有匹配内容，请试“点位”“WATER”或“任务”。';results.append(p);}
});
document.addEventListener('keydown',e=>{if(e.key==='Escape'&&results)results.hidden=true;if(e.key==='/'&&document.activeElement.tagName!=='INPUT'){e.preventDefault();input?.focus();}});
document.addEventListener('click',e=>{if(results&&!e.target.closest('.searchbox'))results.hidden=true;});
for(const b of document.querySelectorAll('[data-api-filter]'))b.addEventListener('click',()=>{
 for(const x of document.querySelectorAll('[data-api-filter]'))x.classList.toggle('selected',x===b);
 for(const x of document.querySelectorAll('.api-method'))x.hidden=b.dataset.apiFilter!=='all'&&x.dataset.method!==b.dataset.apiFilter;
});
