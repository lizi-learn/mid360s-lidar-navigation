#!/usr/bin/env python3
"""Bounded real task-worker integration with a synthetic planar navigation adapter."""
import argparse
import copy
import json
from pathlib import Path
import sys
import tempfile
import time
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(Path(__file__).parent));sys.path.insert(0,str(ROOT/'navigation'))
from server import Sandbox
from reception_routes import route, DoorEvidence

def wait(predicate,timeout=8):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.02)
    raise AssertionError('bounded scenario timed out')

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);args=ap.parse_args()
    scenarios=[]
    with tempfile.TemporaryDirectory() as folder:
        node=Sandbox(folder)
        try:
            wait(lambda:node.session.status(time.monotonic())['navigation_permitted'])
            doc=node.reception_catalog.read(node.map_name)
            doc['points'][1]['x']=.6;doc['points'][2]['x']=1.2;doc['points'][2]['yaw']=1.
            doc['points'][2]['guest_wait']=False;doc['points'][2]['product_id']=''
            doc['routes'][0]['steps']=[{'point_id':'hall','speak':False},{'point_id':'robot','speak':False}]
            doc=node.reception_catalog.save(doc,doc['revision'])
            node.reception.command({'command':'start','route_id':'welcome_tour','revision':doc['revision']})
            wait(lambda:node.reception.engine.phase=='completed')
            assert len(node.commands)==1
            assert abs(node.pose[0]-1.2)<.1 and abs(node.pose[2]-1.)<.01
            scenarios.append({'name':'single_start_continuous_transit_and_final_heading','passed':True,'model_arrivals':1,'transit_receipts':1,'hardware_commands':0})
            # New run owns a copy; editing the catalog must not move its target.
            node.pose=[0.,0.,0.]
            node.reception.command({'command':'start','route_id':'welcome_tour','revision':doc['revision']})
            wait(lambda:node.reception.engine.phase=='navigating')
            edited=copy.deepcopy(doc);edited['points'][1]['x']=10.;node.reception_catalog.save(edited,doc['revision'])
            assert node.reception.engine.steps[0]['point']['x']==.6
            node.operator='manual';wait(lambda:node.reception.engine.phase=='manual')
            n=len(node.commands);node.operator='navigation_allowed';time.sleep(.3)
            assert node.reception.engine.phase=='manual'and len(node.commands)==n and node.velocity==[0.,0.,0.]
            scenarios.append({'name':'snapshot_manual_takeover_no_auto_resume','passed':True,'hardware_commands':0})
            doc=node.reception_catalog.read(node.map_name);doc['points'][1]['x']=.6
            doc['routes'][0]['steps']=[{'point_id':'robot','speak':True}];doc['points'][2]['product_id']='p2'
            doc=node.reception_catalog.save(doc,doc['revision']);node.pose=[1.,0.,0.]
            node.reception.command({'command':'start','route_id':'welcome_tour','revision':doc['revision']})
            wait(lambda:node.reception.engine.phase=='guest_wait')
            events=node.reception.engine.events
            assert any(e['kind']=='speech_unavailable'for e in events)
            assert not any(e['kind']=='speech_completed'for e in events)
            node.reception.command({'command':'continue','epoch':node.reception.engine.epoch});assert node.reception.engine.phase=='completed'
            scenarios.append({'name':'missing_voice_text_fallback_not_fake_playback','passed':True})
            # Closed / reopened door graph: real evidence sequence and dwell.
            d=DoorEvidence();d.observe('closed',10.,1,10.);closed=route(doc,'hall','side',{'side_door':d.current(10.)})
            d.observe('open',11.,2,11.);assert d.current(11.)=='unknown'
            d.observe('open',11.5,3,11.5);d.observe('open',12.,4,12.);d.observe('open',12.6,5,12.6);opened=route(doc,'hall','side',{'side_door':d.current(12.6)})
            assert closed is not None and opened is not None and opened['cost']<closed['cost']
            scenarios.append({'name':'door_opening_hysteresis_and_shorter_graph_route','passed':True,'occupancy_cells_modified':0})
            node.reception.command({'command':'start','point_id':'robot','revision':doc['revision']})
            wait(lambda:node.reception.engine.phase=='navigating')
            node.reception.command({'command':'pause'});wait(lambda:node.reception.engine.phase=='paused')
            count=len(node.commands);time.sleep(.2);assert len(node.commands)==count and node.velocity==[0.,0.,0.]
            node.reception.command({'command':'resume','epoch':node.reception.engine.epoch})
            wait(lambda:node.reception.engine.phase=='guest_wait')
            node.reception.command({'command':'continue','epoch':node.reception.engine.epoch})
            scenarios.append({'name':'single_point_pause_explicit_resume','passed':True,'hardware_commands':0})
            node.reception.history.pending.join()
            history=node.reception.history.read(node.map_name)
            assert any(e['kind']=='paused'for e in history['events'])and any(e['kind']=='resumed'for e in history['events'])
            scenarios.append({'name':'durable_task_history_actual_events','passed':True,'hardware_commands':0})
            result={'passed':True,'scenarios':scenarios,'hardware_commands':0,'model_start_requests':len(node.commands),
                    'scope':'Actual MissionService thread and NavigationSession with own synthetic planar/A* adapter. Not Nav2, RL gait, or physical motion validation.',
                    'boundaries':['This adapter is not ROS/Nav2 or biped acceptance; see separate native report for actual action and laser evidence checks.']}
        finally:node.close()
    Path(args.output).parent.mkdir(parents=True,exist_ok=True);Path(args.output).write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False))
if __name__=='__main__':main()
