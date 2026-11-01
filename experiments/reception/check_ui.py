#!/usr/bin/env python3
"""Syntax-check both the direct UI and the HTML embedded through the old G12 gateway."""
import json,re,subprocess,tempfile,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'navigation'))
from reception_ui import reception_html
app=reception_html(ROOT/'navigation');console=(ROOT/'navigation/console.html').read_text()
rendered=console.replace('/*RECEPTION_APP*/','function openReception(){document.open();document.write('+json.dumps(app,ensure_ascii=False).replace('</','<\\/')+');document.close()}')
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
