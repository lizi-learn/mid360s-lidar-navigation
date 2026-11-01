#!/usr/bin/env python3
"""Actual cache subprocess and local player lifecycle with synthetic WAV/paplay."""
import argparse,json,os,sys,tempfile,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'navigation'))
from reception_catalog import content_key
from reception_voice import VoiceCache,PresentationPlayer

def wait(test):
    until=time.monotonic()+3
    while time.monotonic()<until:
        if test():return
        time.sleep(.02)
    raise AssertionError('bounded voice test timeout')

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);args=ap.parse_args()
    with tempfile.TemporaryDirectory()as folder:
        root=Path(folder);tts=root/'fake_tts.py'
        tts.write_text('import sys,json,wave\nd=json.load(sys.stdin)\nw=wave.open(d["output"],"wb");w.setparams((1,2,24000,0,"NONE","none"));w.writeframes(b"\\0\\0"*12000);w.close()\n')
        player=root/'paplay';trace=root/'played.txt'
        player.write_text('#!'+sys.executable+'\nimport os,sys,time\nfrom pathlib import Path\nwith open(os.environ["RECEPTION_TEST_TRACE"],"a")as f:f.write(" ".join(sys.argv[1:])+"\\n")\ntime.sleep(.3)\n');player.chmod(0o755)
        original_path=os.environ['PATH'];os.environ['PATH']=str(root)+os.pathsep+original_path;os.environ['RECEPTION_TEST_TRACE']=str(trace)
        try:
            text='合成测试，不代表真实语音。';key=content_key(text)
            doc={'map_id':'synthetic_voice','points':[{'script':text,'product_id':''}],'products':[]}
            cache=VoiceCache(root/'cache',[sys.executable,str(tts)]);cache.prepare(doc);wait(lambda:not cache.busy)
            assert cache.status(doc)['clips'][key]['available']
            result=root/'result.json';p=PresentationPlayer(cache.root,result)
            mission={'phase':'presenting','epoch':123,'speech_key':key}
            p.observe(mission,enabled=True,volume=55,now=time.monotonic())
            assert json.loads(result.read_text())['result']=='started'
            time.sleep(.4);p.observe(mission,enabled=True,volume=55,now=time.monotonic())
            assert json.loads(result.read_text())['result']=='completed'
            assert '--volume=36045'in trace.read_text()
            # A worker restart must not replay a completed presentation.
            q=PresentationPlayer(cache.root,result);q.observe(mission,enabled=True,volume=55,now=time.monotonic());assert q.process is None
            # Interrupt a new presentation; reconnect cannot restart old speech.
            mission['epoch']=124;p.observe(mission,enabled=True,volume=55,now=time.monotonic());p.stop()
            assert json.loads(result.read_text())['result']=='failed'
            p.observe(mission,enabled=True,volume=55,now=time.monotonic());assert p.process is None
            # Unknown content must be missing, never reported as completed.
            mission['epoch']=125;mission['speech_key']='0'*64;p.observe(mission,enabled=True,volume=55,now=time.monotonic())
            assert json.loads(result.read_text())['result']=='missing'
            q.stop();p.stop()
            report={'passed':True,'cases':['background_cache_subprocess','playback_completion_and_volume','worker_restart_no_replay','interrupt_reconnect_no_replay','missing_not_success'],
                    'hardware_commands':0,'scope':'Actual subprocesses and 0.5s silent synthetic WAV; fake paplay. Not subjective speech or physical speaker proof.'}
        finally:
            os.environ['PATH']=original_path;os.environ.pop('RECEPTION_TEST_TRACE',None)
    Path(args.output).parent.mkdir(parents=True,exist_ok=True);Path(args.output).write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report))
if __name__=='__main__':main()
