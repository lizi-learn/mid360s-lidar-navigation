import json
import sys
import tempfile
import time
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from reception_history import History
from reception_mission import Mission
from test_reception import fixture,permitted

class HistoryTests(unittest.TestCase):
    def test_persisted_events_and_map_cursor_isolation(self):
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'events.sqlite3';h=History(path)
            for i in range(5):h.append(dict(run_id='test',map_id='a'if i%2==0 else 'b',wall_time=time.time(),kind='arrived',point=str(i)))
            h.pending.join();h.close();h=History(path)
            try:
                r=h.read('a',limit=2);self.assertEqual([e['point']for e in r['events']],['4','2'])
                self.assertEqual([e['point']for e in h.read('a',r['before'])['events']],['0'])
                self.assertEqual(len(h.read('b')['events']),2)
                self.assertEqual(path.stat().st_mode&0o777,0o600)
                with self.assertRaises(ValueError):h.read('a',limit=True)
                with self.assertRaises(ValueError):h.read('a',before=-1)
            finally:h.close()
    def test_catalog_restore_makes_new_revision_and_keeps_history(self):
        from reception_catalog import Catalog
        with tempfile.TemporaryDirectory()as root:
            catalog=Catalog(root);one=catalog.save(fixture(),0);one['points'][0]['name']='edited'
            two=catalog.save(one,1);restored=catalog.restore(one['map_id'],1,2)
            self.assertEqual(restored['revision'],3);self.assertEqual(restored['points'][0]['name'],'a')
            self.assertEqual([r['revision']for r in catalog.versions(one['map_id'])['versions']],[2,1])
            self.assertEqual(catalog.read_revision(one['map_id'],2)['points'][0]['name'],'edited')
            with self.assertRaises(ValueError):catalog.restore(one['map_id'],1,2)
            with self.assertRaises(ValueError):catalog.read_revision('../private',1)
    def test_mission_snapshot_writes_actual_event_not_fake_arrival(self):
        rows=[];m=Mission(rows.append);e=m.start(fixture(),'tour',0);m.dispatched(e,1);m.cancel('manual')
        self.assertEqual([r['kind']for r in rows],['started','leg_started','manual'])
        self.assertEqual(len({r['run_id']for r in rows}),1);self.assertTrue(all(r['wall_time']>0 for r in rows))
        self.assertFalse(any(r['kind']=='arrived'for r in rows))
    def test_pause_requires_explicit_resume_manual_cannot_resume(self):
        m=Mission();e=m.start(fixture(),'tour',0);m.dispatched(e,1);m.cancel('paused')
        self.assertIsNone(m.tick(permitted(),2));new=m.resume(3);self.assertNotEqual(e,new)
        self.assertEqual(m.tick(permitted(),3)['point']['id'],'b')
        m.cancel('manual')
        with self.assertRaises(ValueError):m.resume(4)
    def test_repeated_cancel_is_idempotent_not_extra_user_events(self):
        rows=[];m=Mission(rows.append);m.start(fixture(),'tour',0);m.cancel();epoch=m.epoch;m.cancel()
        self.assertEqual(m.epoch,epoch);self.assertEqual([r['kind']for r in rows],['started','cancelled'])
    def test_guest_pause_does_not_advance_or_duplicate_arrival(self):
        d=fixture();d['points'][1]['guest_wait']=True;m=Mission();e=m.start(d,'tour',0)
        m.dispatched(e,1);m.tick(permitted('arrived'),1);m.cancel('paused');m.resume(2)
        self.assertEqual(m.phase,'guest_wait');self.assertEqual(m.index,0)
        self.assertEqual(sum(r['kind']=='arrived'for r in m.events),1)

if __name__=='__main__':unittest.main()
