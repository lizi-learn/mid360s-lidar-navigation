#!/usr/bin/env python3
"""Syntax-check both the direct UI and the HTML embedded through the old G12 gateway."""
import json,re,subprocess,tempfile,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'navigation'))
from reception_ui import reception_html,console_html,embedded_reception_html
app=reception_html(ROOT/'navigation');console=(ROOT/'navigation/console.html').read_text()
rendered=console_html(ROOT/'navigation')
with tempfile.TemporaryDirectory()as folder:
    count=0
    for document in (app,rendered):
        for script in re.findall(r'<script[^>]*>(.*?)</script>',document,re.S):
            path=Path(folder)/f'{count}.js';path.write_text(script);subprocess.run(['node','--check',str(path)],check=True);count+=1
print(json.dumps({'passed':True,'inline_scripts_checked':count,'scope':'JS syntax and G12 nested HTML escaping; not visual QA or physical G12 interaction'}))

# Exercise the G12 HTTP fallback without SubtleCrypto, including padding edges.
script=r"""const fs=require('fs'),vm=require('vm'),crypto=require('crypto');
const ctx=vm.createContext({Uint8Array,Uint32Array,DataView,Math,Promise,setTimeout});
vm.runInContext(fs.readFileSync(process.argv[1],'utf8'),ctx);
(async()=>{for(const n of [0,3,55,56,63,64,65,1000000]){const input=Uint8Array.from({length:n},(_,i)=>i%251);ctx.input=input;const result=await vm.runInContext('sha256Bytes(input)',ctx);if(result!==crypto.createHash('sha256').update(input).digest('hex'))throw Error('SHA256 fallback '+n);}console.log('HTTP G12 SHA256 fallback:8 vectors passed');})().catch(e=>{console.error(e);process.exit(1)});"""
subprocess.run(['node','-e',script,str(ROOT/'navigation/reception_hash.js')],check=True)

# Reproduce the actual old-entry lexical collision and verify the rendered fix.
probe=r"""const fs=require('fs'),vm=require('vm');
const input=JSON.parse(fs.readFileSync(0,'utf8'));
const node=()=>({style:{},dataset:{},classList:{toggle(){},add(){},remove(){}},getContext(){return {}},after(){},append(){},querySelectorAll(){return []}});
const elements=new Map(),document={getElementById(id){if(!elements.has(id))elements.set(id,node());return elements.get(id)},createElement:node,querySelector:node,head:node(),querySelectorAll(){return []},addEventListener(){},body:node()};
const ctx=vm.createContext({document,location:{pathname:'/api/v1/modules/navigation/workbench',search:''},URLSearchParams,AbortController,fetch:async()=>{throw Error('offline lexical fixture')},setInterval(){return 1},setTimeout(){return 1},clearTimeout(){},ResizeObserver:class{observe(){}},addEventListener(){}});ctx.window=ctx;
vm.runInContext("const API_BASE='parent-console',ctx={};let state='parent-state',zoom=1,view=null,pan=[0,0];",ctx);
let oldRejected=false;try{vm.runInContext(input.direct,ctx)}catch(e){oldRejected=e.name==='SyntaxError'}
if(!oldRejected)throw Error('Fixture did not reproduce old lexical collision');
vm.runInContext(input.embedded,ctx);
if(vm.runInContext('API_BASE',ctx)!=='parent-console'||vm.runInContext('state',ctx)!=='parent-state')throw Error('Parent state overwritten');
for(const name of input.exports)if(typeof ctx[name]!=='function')throw Error('Inline action missing: '+name);
console.log('Legacy global collision reproduced; isolated startup and all inline actions passed');"""
embedded=embedded_reception_html(ROOT/'navigation')
subprocess.run(['node','-e',probe],input=json.dumps({'direct':re.findall(r'<script[^>]*>(.*?)</script>',app,re.S)[0],'embedded':re.findall(r'<script[^>]*>(.*?)</script>',embedded,re.S)[0],'exports':sorted(set(re.findall(r'onclick="([A-Za-z_$][\w$]*)\(',app)))}).encode(),check=True)

# The compact panel must never label an unknown/unverified switch as left.
mode_helper=app[app.index('function operatorInstruction()'):app.index('function toast(')]
mode_probe=r"""const vm=require('vm'),fs=require('fs'),input=JSON.parse(fs.readFileSync(0,'utf8'));const c=vm.createContext({});vm.runInContext('let state=null;'+input.source,c);
for(const [session,expected]of [[null,'连接'],[{operator_verified:false,operator_mode:'navigation_allowed'},'连接'],[{operator_verified:true,operator_mode:'manual'},'右档'],[{operator_verified:true,operator_mode:'paused'},'中档'],[{operator_verified:true,operator_mode:'navigation_allowed'},'左档'],[{operator_verified:true,operator_mode:'unknown'},'未允许'],[{operator_verified:true,operator_mode:'navigation_allowed',manual_activity:true},'摇杆']]){c.fixture={session};vm.runInContext('state=fixture',c);if(!vm.runInContext('operatorInstruction()',c).includes(expected))throw Error('Wrong operator mode presentation: '+expected);}console.log('Actual operator hint:7 verified/manual/pause/unknown/takeover cases passed');"""
subprocess.run(['node','-e',mode_probe],input=json.dumps({'source':mode_helper}).encode(),check=True)
