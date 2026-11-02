"""Generate a bounded Nav2 recovery tree for the current service wait policy."""
from pathlib import Path
import hashlib,math,os,xml.etree.ElementTree as ET

def recovery_tree(base,directory,max_wait):
    if type(max_wait)not in(int,float)or not math.isfinite(max_wait)or not .1<=max_wait<=600:raise ValueError('Invalid recovery wait duration')
    base=Path(base);tree=ET.parse(base);root=tree.getroot()
    recoveries=root.findall('.//RecoveryNode')
    if len(recoveries)!=1:raise ValueError('Expected one navigation recovery node')
    waits=recoveries[0].findall('.//Wait')
    if len(waits)!=1:raise ValueError('Expected one observed route wait')
    waits[0].set('name','ObservedRouteWait')
    duration=float(waits[0].get('wait_duration','2'))
    if not math.isfinite(duration)or duration<=0:raise ValueError('Invalid native wait duration')
    # Session owns the configurable timeout and cancellation. The native tree
    # must remain available long enough to carry that policy rather than abort
    # after the old two waits. A bounded fallback remains if used standalone.
    recoveries[0].set('number_of_retries',str(math.ceil(max_wait/duration)+2))
    data=ET.tostring(root,encoding='utf-8',xml_declaration=True)
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    target=directory/(base.stem+'-'+hashlib.sha256(data).hexdigest()[:16]+'.xml')
    if not target.exists():
        temporary=target.with_suffix('.'+str(os.getpid())+'.tmp');temporary.write_bytes(data);temporary.replace(target)
    return str(target)
