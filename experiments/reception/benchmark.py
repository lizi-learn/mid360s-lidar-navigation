#!/usr/bin/env python3
"""Measure content summary cost independently of ROS and actuators."""
import argparse,json,resource,statistics,sys,tempfile,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'navigation'))
from reception_catalog import Catalog,blank

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);args=ap.parse_args()
    doc=blank('synthetic_large')
    doc['points']=[{'id':f'p{i}','name':'演示点位'+str(i),'x':i%50,'y':i//50,'yaw':None,'kind':'visit','script':'','product_id':'','dwell':0,'guest_wait':False}for i in range(1000)]
    doc['routes']=[{'id':f'r{i}','name':'演示路线'+str(i),'mode':'showcase','loops':1,'description':'','steps':[{'point_id':f'p{j}','speak':False}for j in range(20)]}for i in range(20)]
    with tempfile.TemporaryDirectory()as folder:
        catalog=Catalog(folder);catalog.save(doc,0);catalog.summary(doc['map_id']);samples=[]
        for _ in range(1000):
            start=time.perf_counter();catalog.summary(doc['map_id']);samples.append((time.perf_counter()-start)*1000)
        start=time.perf_counter()
        for _ in range(100):catalog.read(doc['map_id'])
        full_ms=(time.perf_counter()-start)*10
        memory={}
        if Path('/proc/self/status').is_file():
            memory={l.split(':',1)[0]:int(l.split()[1])for l in Path('/proc/self/status').read_text().splitlines()if l.startswith(('VmRSS:','VmHWM:'))}
        report={'points':1000,'routes':20,'bytes':catalog.path(doc['map_id']).stat().st_size,'summary_median_ms':statistics.median(samples),'summary_p95_ms':sorted(samples)[949],'full_snapshot_average_ms':full_ms,'process_peak_rss_KiB':memory.get('VmHWM'),'process_current_rss_KiB':memory.get('VmRSS'),'rusage_maxrss_KiB':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,'memory_measurement':'Linux /proc/self/status VmHWM for current executable; rusage may inherit an earlier executable high-water mark' ,'hardware_commands':0,'scope':'Local x86 Python content store microbenchmark only; not production ARM/ROS load validation'}
    Path(args.output).parent.mkdir(parents=True,exist_ok=True);Path(args.output).write_text(json.dumps(report,indent=2));print(json.dumps(report))
if __name__=='__main__':main()
