import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from reception_catalog import Catalog, blank, validate, introduction
from reception_mission import Mission
from reception_routes import DoorEvidence, route, ObstructionPolicy, worthwhile_shortcut
from audio_feedback import save_resource


def fixture():
    d=blank('synthetic_office')
    d['products']=[{'id':'robot','name':'展示机器人','script':'您好，欢迎体验。','tags':'产品'}]
    d['points']=[{'id':i,'name':i,'x':x,'y':y,'yaw':None,'product_id':'robot' if i=='c' else '', 'guest_wait':False,'dwell':0,'script':'','kind':'visit'} for i,x,y in [('a',0,0),('b',3,0),('c',3,3),('d',0,3)]]
    d['routes']=[{'id':'tour','name':'接待路线','mode':'showcase','loops':1,'description':'','steps':[{'point_id':'b','speak':True},{'point_id':'c','speak':True}]}]
    d['doors']=[{'id':'door','name':'展厅门','x':1.5,'y':1.5,'width':1.,'yaw':0}]
    d['edges']=[{'id':a+b,'a':a,'b':b,'door_id':'door' if (a,b)==('a','c') else '', 'penalty':0.,'bidirectional':True} for a,b in [('a','b'),('b','c'),('a','c'),('a','d'),('d','c')]]
    return validate(d)

def permitted(phase='executing',generation=1):
    return {'navigation_permitted':True,'operator_mode':'navigation_allowed','phase':phase,'generation':generation}

class CatalogTest(unittest.TestCase):
    def test_revisions_and_history_and_map_isolation(self):
        with tempfile.TemporaryDirectory() as root:
            store=Catalog(root);d=fixture();one=store.save(d,0);self.assertEqual(one['revision'],1)
            two=store.save(one,1);self.assertEqual(two['revision'],2)
            with self.assertRaises(ValueError):store.save(one,1)
            self.assertEqual(store.read('elsewhere')['points'],[])
            self.assertTrue((Path(root)/'history/synthetic_office/1.json').is_file())
    def test_path_traversal(self):
        with tempfile.TemporaryDirectory() as root:
            for s in ['../other','/tmp/a','x/abc','']:
                with self.assertRaises(ValueError):Catalog(root).read(s)
    def test_nonfinite_bool_and_missing_reference(self):
        for value in [float('nan'),float('inf'),True]:
            d=fixture();d['points'][0]['x']=value
            with self.assertRaises(ValueError):validate(d)
        d=fixture();d['points'].pop(1)
        with self.assertRaises(ValueError):validate(d)
    def test_wrong_map_content_schema_and_revision(self):
        for field,value in [('schema',2),('revision',True),('points',None),('unknown',1)]:
            d=fixture();d[field]=value
            with self.assertRaises(ValueError):validate(d)
    def test_duplicate_id_and_door_reference(self):
        d=fixture();d['points'].append(copy.deepcopy(d['points'][0]))
        with self.assertRaises(ValueError):validate(d)
        d=fixture();d['doors']=[]
        with self.assertRaises(ValueError):validate(d)
    def test_intro_overrides_product(self):
        d=fixture();p=d['points'][2];self.assertEqual(introduction(d,p),'您好，欢迎体验。')
        p['script']='此点介绍';self.assertEqual(introduction(d,p),'此点介绍')
    def test_mixed_resource_does_not_save_or_move(self):
        class Node:
            def reception_save(self,d):raise AssertionError('must not mutate')
            def save_map(self,*a):raise AssertionError('must not save map')
        with self.assertRaises(ValueError):save_resource(Node(),{'reception':{},'name':'map'})
    def test_no_shell_or_actuator_import(self):
        import reception_catalog,reception_routes
        for m in [reception_catalog,reception_routes]:
            s=Path(m.__file__).read_text();self.assertNotIn('import rclpy',s);self.assertNotIn('import socket',s)

class MissionTest(unittest.TestCase):
    def start(self,d=None):
        m=Mission();d=d or fixture();epoch=m.start(d,'tour',0);return m,d,epoch
    def test_full_tour_arrival_speech_and_completed(self):
        m,d,e=self.start();cmd=m.tick(permitted(),0);self.assertEqual(cmd['point']['id'],'b')
        m.dispatched(e,1);m.tick(permitted('arrived'),1);self.assertEqual(m.phase,'dispatching')
        m.dispatched(e,2);m.tick(permitted('arrived',2),2);self.assertEqual(m.phase,'presenting')
        key=m.speech_key;m.tick(permitted('arrived',2),3,{'key':'wrong','result':'completed'});self.assertEqual(m.phase,'presenting')
        m.tick(permitted('arrived',2),4,{'key':key,'result':'completed'});self.assertEqual(m.phase,'completed')
        self.assertEqual([x['kind']for x in m.events].count('arrived'),2)
    def test_snapshot_not_changed_by_admin_edit(self):
        m,d,e=self.start();d['points'][1]['x']=100.;self.assertEqual(m.tick(permitted(),0)['point']['x'],3.)
    def test_manual_cancels_and_left_does_not_resume(self):
        for phase in ['dispatching','navigating','presenting','guest_wait','dwell']:
            m,d,e=self.start();m.phase=phase
            m.tick({'navigation_permitted':False,'operator_mode':'manual'},1)
            self.assertEqual(m.phase,'manual');self.assertFalse(m.valid(e));self.assertIsNone(m.tick(permitted(),2))
    def test_pause_cancel_pending_dispatch_invalidated(self):
        m,d,e=self.start();m.cancel('paused');self.assertFalse(m.dispatched(e,1));self.assertFalse(m.valid(e))
    def test_failure_and_unrelated_goal_never_advance(self):
        m,d,e=self.start();m.dispatched(e,1);m.tick(permitted('executing',2),1);self.assertEqual(m.phase,'failed');self.assertEqual(m.index,0)
    def test_speech_failure_becomes_visible_wait_not_success(self):
        m,d,e=self.start();m.index=1;m.dispatched(e,2);m.tick(permitted('arrived',2),1)
        m.tick(permitted('arrived',2),2,{'key':m.speech_key,'result':'missing'})
        self.assertEqual(m.phase,'guest_wait');self.assertNotIn('speech_completed',[x['kind']for x in m.events])
        m.continue_visit(3);self.assertEqual(m.phase,'completed')
    def test_guest_continue_and_dwell(self):
        d=fixture();d['points'][1]['guest_wait']=True;m,d,e=self.start(d);m.dispatched(e,1);m.tick(permitted('arrived'),1)
        self.assertEqual(m.phase,'guest_wait');m.continue_visit(2);self.assertEqual(m.index,1)
        d=fixture();d['points'][1]['dwell']=3.;m,d,e=self.start(d);m.dispatched(e,1);m.tick(permitted('arrived'),1)
        self.assertEqual(m.phase,'dwell');m.tick(permitted('arrived'),3);self.assertEqual(m.index,0);m.tick(permitted('arrived'),4);self.assertEqual(m.index,1)
    def test_no_duplicate_active_start(self):
        m,d,e=self.start()
        with self.assertRaises(ValueError):m.start(d,'tour',1)
    def test_finite_loops_and_heading(self):
        d=fixture();d['routes'][0]['loops']=2;d['points'][1]['yaw']=math.pi/2
        m,d,e=self.start(d);self.assertEqual(len(m.steps),4);self.assertAlmostEqual(m.tick(permitted(),0)['point']['yaw'],math.pi/2)
    def test_terminal_error_not_masked(self):
        m,d,e=self.start();m.dispatched(e,1);s=permitted('blocked');s['notice']='实际无路';m.tick(s,1);self.assertEqual(m.error,'实际无路')

class RoutePolicyTest(unittest.TestCase):
    def test_closed_unknown_and_open_shortcut(self):
        d=fixture();unknown=route(d,'a','c');closed=route(d,'a','c',{'door':'closed'});opened=route(d,'a','c',{'door':'open'})
        self.assertEqual(unknown,closed);self.assertEqual(len(opened['points']),2);self.assertGreater(closed['cost'],opened['cost'])
        self.assertTrue(worthwhile_shortcut(closed['cost'],opened['cost']))
    def test_closed_edges_and_oneway(self):
        d=fixture();self.assertIsNone(route(d,'a','c',closed_edges=[e['id']for e in d['edges']]))
        d['edges']=[{**d['edges'][0],'bidirectional':False}];self.assertIsNone(route(d,'b','a'))
    def test_door_debounce_immediate_closed_and_expiry(self):
        d=DoorEvidence();self.assertTrue(d.observe('open',10.,1,10.));self.assertEqual(d.current(10.),'unknown')
        d.observe('open',10.5,2,10.5);d.observe('open',11.,3,11.);d.observe('open',11.6,4,11.6);self.assertEqual(d.current(11.6),'open')
        d.observe('closed',11.7,5,11.7);self.assertEqual(d.current(11.7),'closed');self.assertEqual(d.current(16),'unknown')
    def test_gap_does_not_count_as_continuous_clear(self):
        d=DoorEvidence();d.observe('open',10.,1,10.);d.observe('open',12.,2,12.)
        self.assertEqual(d.current(12.),'unknown')
    def test_stale_replay_future_door(self):
        d=DoorEvidence()
        for stamp,seq in [(1,1),(20,2)]:self.assertFalse(d.observe('open',stamp,seq,10))
        d.observe('closed',10,3,10);self.assertFalse(d.observe('open',10,3,10));self.assertEqual(d.state,'closed')
    def test_wait_detour_and_timeout(self):
        p=ObstructionPolicy(fixture()['policy']);self.assertEqual(p.decide(0,blocked=True,alternate=True),'wait')
        self.assertEqual(p.decide(4,blocked=True,alternate=True),'detour');self.assertEqual(p.decide(5,blocked=True),'wait')
        self.assertEqual(p.decide(25,blocked=True),'stop_notify')
    def test_crowd_and_clear_hysteresis(self):
        p=ObstructionPolicy(fixture()['policy']);p.decide(0,blocked=True);self.assertEqual(p.decide(5,blocked=True,crowd=True),'wait_crowd')
        self.assertEqual(p.decide(6,blocked=False),'wait_clear');self.assertEqual(p.decide(7,blocked=False),'wait_clear')
        self.assertEqual(p.decide(7.5,blocked=False),'resume');self.assertEqual(p.decide(8,blocked=False),'continue')
    def test_closed_door_skips_brief_wait(self):
        p=ObstructionPolicy(fixture()['policy']);self.assertEqual(p.decide(0,blocked=True,alternate=True,door_closed=True),'detour')
    def test_small_gain_no_route_flapping(self):
        self.assertFalse(worthwhile_shortcut(100,90));self.assertTrue(worthwhile_shortcut(100,70))

if __name__=='__main__':unittest.main()
