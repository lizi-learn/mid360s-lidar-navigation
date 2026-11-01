#!/usr/bin/env python3
"""Syntax-check both the direct UI and the HTML embedded through the old G12 gateway."""
import json,re,subprocess,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
app=(ROOT/'navigation/reception.html').read_text();console=(ROOT/'navigation/console.html').read_text()
rendered=console.replace('/*RECEPTION_APP*/','function openReception(){document.open();document.write('+json.dumps(app,ensure_ascii=False).replace('</','<\\/')+');document.close()}')
with tempfile.TemporaryDirectory()as folder:
    count=0
    for document in (app,rendered):
        for script in re.findall(r'<script[^>]*>(.*?)</script>',document,re.S):
            path=Path(folder)/f'{count}.js';path.write_text(script);subprocess.run(['node','--check',str(path)],check=True);count+=1
print(json.dumps({'passed':True,'inline_scripts_checked':count,'scope':'JS syntax and G12 nested HTML escaping; not visual QA or physical G12 interaction'}))
