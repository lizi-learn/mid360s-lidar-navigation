"""Supervise only this sensor preview stack; no motor or locomotion access."""
import json,pathlib,time,urllib.request,sys,os
def data_stale(status):
    return any(status.get(k) is not None and status[k]>3 for k in ('raw_age','imu_age'))
def main():
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    failures=0;seen=False
    while True:
        time.sleep(1)
        try:
            with opener.open('http://127.0.0.1:'+str(int(os.environ.get('VIEW_PORT','8920')))+'/api/status',timeout=3) as response:s=json.load(response)
            seen=seen or s['counts']['imu']>0
            failures=failures+1 if seen and data_stale(s) else 0
            if failures>=3:
                for k in ('points','free_cells','obstacle_cells'):s.pop(k,None)
                pathlib.Path(os.getenv('P2_NAV_RUNTIME',str(pathlib.Path(__file__).with_name('runtime')))).joinpath('last-data-stall.json').write_text(json.dumps(s,ensure_ascii=False,indent=2))
                print('Sensor data stalled; restarting this preview stack with a new map origin.',flush=True)
                return 2
        except (OSError,ValueError,KeyError):
            failures+=1
            if seen and failures>=3:return 3
if __name__=='__main__':sys.exit(main())
