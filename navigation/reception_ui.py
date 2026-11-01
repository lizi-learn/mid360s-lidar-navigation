"""One UI renderer for direct HTTP and the existing G12 embedded document."""
from pathlib import Path
import json
import re

def reception_html(root):
    root=Path(root)
    return (root/'reception.html').read_text().replace('/*RECEPTION_TOOLS*/',
        (root/'reception_hash.js').read_text()+'\n'+(root/'reception_tools.js').read_text())

def embedded_reception_html(root):
    app=reception_html(root)
    # Isolate script declarations from the old console's surviving lexical
    # environment while preserving its URL, gateway and inline button actions.
    names=sorted(set(re.findall(r'onclick="([A-Za-z_$][\w$]*)\(',app)))
    exports='Object.assign(window,{'+','.join(names)+'});'
    app=re.sub(r'(<script[^>]*>)(.*?)(</script>)',
        lambda m:m[1]+'(function(){\n'+m[2]+'\n'+exports+'\n})();'+m[3],app,flags=re.S)
    return app

def console_html(root):
    root=Path(root)
    literal=json.dumps(embedded_reception_html(root),ensure_ascii=False).replace('</','<\\/')
    entry='function openReception(){consoleDisposed=true;clearInterval(consoleRefreshTimer);document.open();document.write('+literal+');document.close()}'
    # The factory gateway preserves the browser query but serves the root
    # document upstream. Enter the operator view without changing that gateway.
    entry += ";window.addEventListener('load',()=>{if(new URLSearchParams(location.search).get('view')==='operator')openReception()});"
    return (root/'console.html').read_text().replace('/*RECEPTION_APP*/',entry)
