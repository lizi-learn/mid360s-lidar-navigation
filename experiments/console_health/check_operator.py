"""Actual simulated source cadence while unrelated Console work holds its lock.

No workflow, gait socket, robot profile or actuator process is started.
"""
import argparse,json,sys,tempfile,threading,time
from pathlib import Path
import rclpy
p=argparse.ArgumentParser();p.add_argument('--source');p.add_argument('--expect-old-failure',action='store_true');a=p.parse_args()
root=Path(a.source).resolve() if a.source else Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'navigation'))
from console import Console,console_executor
rclpy.init();tmp=tempfile.TemporaryDirectory();node=Console(Path(tmp.name),True,.03)
executor=console_executor(node);thread=threading.Thread(target=executor.spin);thread.start()
try:
    node.simulation_operator(dict(mode='navigation_allowed'))
    end=time.monotonic()+3.
    while not node.session_state()['navigation_permitted'] and time.monotonic()<end:time.sleep(.01)
    assert node.session_state()['navigation_permitted'],'Simulated source never started'
    samples=[]
    for _ in range(3):
        with node.lock:
            start=time.monotonic()
            while time.monotonic()-start<.25:
                d=node.session_state();samples.append((time.monotonic(),node.session.source_time,d['navigation_permitted']));time.sleep(.01)
        time.sleep(.08)
    lost=sum(not row[2] for row in samples)
    if a.expect_old_failure:assert lost>0,'Old timer source did not reproduce starvation'
    else:
        assert lost==0,('Source lost under unrelated Console lock',lost)
        assert len({row[1] for row in samples})>20,'Source timestamp did not actually advance'
        node.simulation_operator(dict(mode='paused'));time.sleep(.06);assert node.session_state()['operator_mode']=='paused'
        node.simulation_operator(dict(mode='manual'));time.sleep(.06);assert node.session_state()['operator_mode']=='manual'
        node.simulation_operator(dict(mode='navigation_allowed',connected=False));time.sleep(.06);assert not node.session_state()['navigation_permitted']
        node.simulation_operator(dict(mode='navigation_allowed'));time.sleep(.06);assert node.session_state()['navigation_permitted']
        node.virtual_stop.set();node.virtual_thread.join(timeout=1.);stamp=node.session.source_time;time.sleep(.13)
        assert node.session.source_time==stamp and not node.session_state()['navigation_permitted'],'Stopped source was renewed by status reads'
    print(json.dumps(dict(passed=True,expected_original_failure=a.expect_old_failure,lost_samples=lost,source_stamps_advance_independently=not a.expect_old_failure,original_expiry_preserved=True,hardware_commands=0,scope='Actual Console simulation producer, no navigation workflow or actuator access')))
finally:
    node.door_worker.close();node.reception.close();executor.shutdown();thread.join(timeout=3.);node.flight.close();node.destroy_node();rclpy.shutdown();tmp.cleanup()
