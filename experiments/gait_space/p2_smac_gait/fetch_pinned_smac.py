"""Fetch checked Apache-2.0 Nav2 sources into an owned CMake build directory."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time
import urllib.error
import urllib.request

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    package=Path(__file__).resolve().parent
    provenance=json.loads((package/'upstream-provenance.json').read_text())
    commit=provenance['upstream_commit']
    if not re.fullmatch('[0-9a-f]{40}',commit):raise ValueError('Invalid pinned commit')
    patch=package/'continuous-primitive.patch'
    identity={'upstream':provenance,'patch_sha256':digest(patch)}
    output=args.output.resolve()
    if output.exists():
        saved=json.loads((output/'verified.json').read_text())
        if saved['identity']!=identity:raise RuntimeError('Build input changed; use a fresh build directory')
        for name,value in saved['patched_source_sha256'].items():
            if digest(output/'src'/name)!=value:raise RuntimeError('Previously fetched source was modified: '+name)
        return
    output.parent.mkdir(parents=True,exist_ok=True)
    temporary=Path(tempfile.mkdtemp(prefix='smac-verified-',dir=output.parent))
    try:
        (temporary/'src').mkdir()
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        for name,value in provenance['sources'].items():
            if Path(name).name!=name or not name.endswith('.cpp') or not re.fullmatch('[0-9a-f]{64}',value):raise ValueError('Invalid pinned source entry')
            url=f'https://raw.githubusercontent.com/ros-navigation/navigation2/{commit}/nav2_smac_planner/src/{name}'
            for attempt in range(3):
                try:
                    with opener.open(url,timeout=15) as response:data=response.read()
                    break
                except (urllib.error.URLError,TimeoutError):
                    if attempt==2:raise
                    time.sleep(.2*(attempt+1))
            if hashlib.sha256(data).hexdigest()!=value:raise RuntimeError('Upstream digest mismatch: '+name)
            (temporary/'src'/name).write_bytes(data)
        subprocess.run(['patch','--batch','--fuzz=0','-p1','-i',str(patch)],cwd=temporary,check=True)
        record={'identity':identity,'patched_source_sha256':{p.name:digest(p) for p in (temporary/'src').glob('*.cpp')}}
        (temporary/'verified.json').write_text(json.dumps(record,indent=2)+'\n')
        temporary.rename(output)
    finally:
        if temporary.exists():shutil.rmtree(temporary)

if __name__=='__main__':main()
