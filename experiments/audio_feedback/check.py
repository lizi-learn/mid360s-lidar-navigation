#!/usr/bin/env python3
"""Actual audio worker + fake local GET-only server/player; no robot interface."""
import argparse,json,os,subprocess,sys,tempfile,threading,time
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'navigation'))
from audio_feedback import AudioSettings

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);args=ap.parse_args()
    with tempfile.TemporaryDirectory()as tmp:
        folder=Path(tmp);seen=[];state={'session':{'phase':'idle','generation':0},'request_token':'synthetic-local-only'}
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*_):pass
            def do_GET(self):
                seen.append(('GET',self.path));self.send_response(200);self.end_headers();self.wfile.write(json.dumps(state).encode())
            def do_POST(self):
                seen.append(('POST',self.path));self.send_response(405);self.end_headers()
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler);threading.Thread(target=server.serve_forever,daemon=True).start()
        fake=folder/'paplay';fake.write_text('#!'+sys.executable+'\nimport os,json,sys,time\nfrom pathlib import Path\nwith open(os.environ["CUE_TRACE"],"a")as f:f.write(json.dumps(sys.argv[1:])+"\\n")\ntime.sleep(.1)\nsys.exit(9 if Path(os.environ["CUE_FAIL"]).exists()else 0)\n');fake.chmod(0o755)
        env=dict(os.environ,PATH=str(folder)+os.pathsep+os.environ['PATH'],CUE_TRACE=str(folder/'played.jsonl'),CUE_FAIL=str(folder/'fail'))
        settings=AudioSettings(folder/'audio-settings.json');cmd=[sys.executable,str(ROOT/'navigation/audio_worker.py'),'--settings',str(settings.path),'--voice-dir',str(folder/'voices'),'--url',f'http://127.0.0.1:{server.server_port}']
        log=(folder/'worker.log').open('w');p=subprocess.Popen(cmd,env=env,stdout=log,stderr=log)
        def events():
            q=folder/'played.jsonl';return [json.loads(x)for x in q.read_text().splitlines()]if q.exists()else[]
        def kinds():return [next(x.split('=',1)[1]for x in row if x.startswith('--stream-name='))for row in events()]
        def wait(test,seconds=4.):
            until=time.monotonic()+seconds
            while time.monotonic()<until:
                assert p.poll()is None,'worker exited unexpectedly'
                if test():return
                time.sleep(.05)
            raise AssertionError({'observed':kinds(),'expected_condition_timeout':True})
        def phase(name,generation=1):state['session']={'phase':name,'generation':generation}
        try:
            wait(lambda:settings.status()['player']is not None)
            assert not kinds()
            for name in ['planning','ready','starting']:phase(name);time.sleep(.3)
            assert not kinds();phase('executing');wait(lambda:kinds()==['started'])
            time.sleep(.6);assert kinds()==['started']
            phase('replanning');time.sleep(.4);phase('executing');time.sleep(.4);assert kinds()==['started']
            phase('replanning');wait(lambda:kinds()==['started','obstacle']);time.sleep(3.1)
            phase('executing');wait(lambda:kinds()[-1]=='continued')
            phase('arrived');wait(lambda:kinds()[-1]=='arrived')
            phase('planning',2);time.sleep(.3);phase('blocked',2);wait(lambda:kinds()[-1]=='no_path')
            phase('starting',3);time.sleep(.3);phase('executing',3);wait(lambda:kinds()[-1]=='started')
            phase('manual',3);wait(lambda:kinds()[-1]=='manual')
            settings.update({'enabled':False,'test':True});before=len(kinds());time.sleep(.5);assert len(kinds())==before
            settings.update({'enabled':True,'volume':40,'test':True});wait(lambda:kinds()[-1]=='test');assert '--volume=26214'in events()[-1]
            (folder/'fail').touch();settings.update({'test':True});wait(lambda:(settings.status().get('player')or{}).get('error')=='playback_failed')
            assert all(method=='GET'and path=='/api/status?summary=1'for method,path in seen)
            result={'passed':True,'events':kinds(),'status_requests':len(seen),'navigation_post_requests':0,'hardware_commands':0,'scope':'Actual independent audio worker, synthetic state changes and local mock paplay. No ROS, robot process, actuator socket or physical speaker.'}
        finally:
            p.terminate();p.wait(timeout=3);server.shutdown();server.server_close();log.close()
        Path(args.output).parent.mkdir(parents=True,exist_ok=True);Path(args.output).write_text(json.dumps(result,indent=2));print(json.dumps(result))
if __name__=='__main__':main()
