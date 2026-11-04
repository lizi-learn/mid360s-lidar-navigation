#!/usr/bin/env python3
"""Real isolated ROS parameter replies; no navigation action or hardware I/O."""
from pathlib import Path
import argparse,json,os,sys,threading,time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.srv import SetParametersAtomically
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'navigation'))
from planning_requests import RouteReplyTimeout,wait_route_reply

p=argparse.ArgumentParser();p.add_argument('--output',required=True);args=p.parse_args()
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1' and 200<=int(os.environ['ROS_DOMAIN_ID'])<=232
rclpy.init();server=rclpy.create_node('route_reply_fixture_server');client_node=rclpy.create_node('route_reply_fixture_client')
delay=[2.25]
def respond(request,response):
    time.sleep(delay[0]);response.result.successful=True;return response
service=server.create_service(SetParametersAtomically,'/route_reply_fixture/apply',respond)
client=client_node.create_client(SetParametersAtomically,'/route_reply_fixture/apply')
executor=SingleThreadedExecutor();executor.add_node(server);executor.add_node(client_node)
worker=threading.Thread(target=executor.spin,daemon=True);worker.start()
try:
    assert client.wait_for_service(timeout_sec=5.)
    f=client.call_async(SetParametersAtomically.Request());started=time.monotonic()
    reply=wait_route_reply(f,client,lambda:True);elapsed=time.monotonic()-started
    assert reply.result.successful and 2.<elapsed<8.
    current=threading.Event();current.set();f=client.call_async(SetParametersAtomically.Request())
    timer=threading.Timer(.1,current.clear);timer.start();started=time.monotonic()
    try:wait_route_reply(f,client,current.is_set)
    except ValueError:cancel_elapsed=time.monotonic()-started
    else:raise AssertionError('Cancelled reply was consumed')
    timer.join();assert cancel_elapsed<.3
    time.sleep(2.5);assert not f.done(),'Removed late reply unexpectedly completed the caller'
    delay[0]=.4;f=client.call_async(SetParametersAtomically.Request());started=time.monotonic()
    try:wait_route_reply(f,client,lambda:True,timeout=.1)
    except RouteReplyTimeout:timeout_elapsed=time.monotonic()-started
    else:raise AssertionError('Missing bounded timeout')
    time.sleep(.5);assert not f.done()
    row=dict(passed=True,actual_ROS_service=True,delayed_reply_s=elapsed,cancel_response_s=cancel_elapsed,
             bounded_timeout_s=timeout_elapsed,late_replies_removed=True,hardware_commands=0,
             scope='Actual isolated SetParametersAtomically client/server; synthetic server delay, no actual planner workload or physical navigation')
    output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(row,indent=2));print(json.dumps(row))
finally:
    executor.shutdown();worker.join(timeout=3.);server.destroy_node();client_node.destroy_node();rclpy.shutdown()
