"""One Start waits for real native readiness without retrying after takeover."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'navigation'))
from reception_catalog import blank
from reception_mission import Mission, MissionService


class DispatchNativeReadinessTest(unittest.TestCase):
    def setup_dispatch(self, ready_at=.2, change=None):
        clock=SimpleNamespace(now=0.)
        point=dict(id='goal',name='goal',x=3.,y=0.,yaw=None,kind='visit',product_id='',script='',dwell=0.,guest_wait=False)
        doc=blank('synthetic');doc['points']=[point]
        doc['routes']=[dict(id='route',name='route',mode='showcase',loops=1,description='',steps=[dict(point_id='goal',speak=False)])]
        engine=Mission();epoch=engine.start(doc,'route',0.)
        service=MissionService.__new__(MissionService)
        service.engine=engine;service.resume_command=None;service.active_command=None
        session=SimpleNamespace(status=lambda now:dict(navigation_permitted=True,generation=1))
        confirms=[]
        def confirm(data):
            self.assertGreaterEqual(clock.now,ready_at)
            self.assertTrue(engine.valid(epoch))
            confirms.append(data)
        node=SimpleNamespace(session=session,set_goal=lambda data:dict(generation=1),health_ready=lambda:True,
            execution_ready=lambda:clock.now>=ready_at,confirm_goal=confirm,
            gait=SimpleNamespace(status=lambda:dict(blockers='actual native body sample unavailable')))
        service.node=node
        def sleep(dt):
            clock.now+=dt
            if change:change(clock,engine,session)
        command=dict(epoch=epoch,point=point)
        return service,engine,clock,confirms,command,sleep

    def run_dispatch(self,parts):
        service,engine,clock,confirms,command,sleep=parts
        with patch('reception_mission.time.monotonic',side_effect=lambda:clock.now),patch('reception_mission.time.sleep',side_effect=sleep):
            service.dispatch(command)
        return engine,clock,confirms

    def test_one_start_waits_for_native_sample_then_confirms_once(self):
        engine,clock,confirms=self.run_dispatch(self.setup_dispatch())
        self.assertEqual(engine.phase,'navigating');self.assertEqual(len(confirms),1)
        self.assertLess(clock.now,1.)

    def test_cancel_during_wait_does_not_start_later(self):
        def cancel(clock,engine,session):
            if clock.now>=.1:engine.cancel()
        engine,clock,confirms=self.run_dispatch(self.setup_dispatch(change=cancel))
        self.assertEqual(engine.phase,'cancelled');self.assertEqual(confirms,[])

    def test_e_right_during_wait_does_not_start_later(self):
        def manual(clock,engine,session):
            if clock.now>=.1:
                session.status=lambda now:dict(navigation_permitted=False,generation=1,operator_mode='manual')
                engine.cancel('manual')
        engine,clock,confirms=self.run_dispatch(self.setup_dispatch(change=manual))
        self.assertEqual(engine.phase,'manual');self.assertEqual(confirms,[])

    def test_missing_actual_sample_fails_with_native_reason_and_no_motion(self):
        engine,clock,confirms=self.run_dispatch(self.setup_dispatch(ready_at=100.))
        self.assertEqual(engine.phase,'failed');self.assertEqual(confirms,[])
        self.assertIn('actual native body sample unavailable',engine.error)
        self.assertLess(clock.now,8.1)
