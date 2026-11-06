import copy,math,unittest
from types import SimpleNamespace as NS
from relative_intent import forward_goal
import test_independent_rotation as rotations
import test_reception_temporary_target as targets

class RelativeTests(unittest.TestCase):
    def test_forward_uses_body_heading_and_metric_displacement(self):
        for yaw in (0.,math.pi/2,math.pi,-math.pi/2):
            p=forward_goal([3.,4.,yaw],2.)
            self.assertAlmostEqual(p['x'],3+2*math.cos(yaw));self.assertAlmostEqual(p['y'],4+2*math.sin(yaw))
            self.assertEqual(p['heading_mode'],'position')
        for value in (0,-1,True,float('inf'),float('nan')):
            with self.assertRaises(ValueError):forward_goal([0,0,0],value)
        with self.assertRaises(ValueError):forward_goal(None,2)
    def test_relative_endpoint_prepares_navigation_without_starting(self):
        setup=rotations.RotationTests();c=setup.setup_console();self.addCleanup(setup.doCleanups)
        c.reception_pose=lambda:[3.,4.,math.pi/2]
        c.prepare_relative({'meters':2.})
        self.assertAlmostEqual(c.session.goal['x'],3.);self.assertAlmostEqual(c.session.goal['y'],6.)
        self.assertFalse(c.nav.sent);self.assertEqual(c.session.phase,'ready')
    def test_stale_task_cannot_replace_relative_or_rotation_goal(self):
        for method,body in [('prepare_relative',{'meters':2.}),('prepare_rotation',{'radians':7.})]:
            setup=rotations.RotationTests();c=setup.setup_console();self.addCleanup(setup.doCleanups)
            c.reception_pose=lambda:[0.,0.,0.];c.checked_pose=lambda d,**kw:object()
            c.reception=NS(source_recovery_active=False,map_id=lambda:'example',engine=NS(run_id='new',epoch=2,phase='navigating'))
            body['_sdk_expected']=dict(generation=c.session.generation,run_id='old',epoch=1,map_id='example')
            generation=c.session.generation
            with self.assertRaises(ValueError):getattr(c,method)(body)
            self.assertEqual(c.session.generation,generation);self.assertFalse(c.nav.sent)
    def test_product_sequence_uses_actual_mission_snapshot_without_saving(self):
        setup=targets.TemporaryTargetTests();node=setup.fixture();self.addCleanup(setup.doCleanups)
        before=copy.deepcopy(node.reception_catalog.read(node.map_name));ids=[p['id']for p in before['points'][:2]]
        self.assertEqual(len(ids),2)
        node.reception.command(dict(command='start',point_ids=ids,revision=before['revision']))
        steps=node.reception.engine.steps
        self.assertEqual([s['point']['id']for s in steps],ids)
        self.assertTrue(all(s['speak']for s in steps))
        self.assertEqual(node.reception_catalog.read(node.map_name),before)
        self.assertEqual(node.commands,[])
    def test_unknown_second_product_never_starts_first(self):
        setup=targets.TemporaryTargetTests();node=setup.fixture();self.addCleanup(setup.doCleanups)
        before=node.reception_catalog.read(node.map_name)
        with self.assertRaises(ValueError):node.reception.command(dict(command='start',point_ids=[before['points'][0]['id'],'missing'],revision=before['revision']))
        self.assertEqual(node.reception.engine.phase,'idle')
    def test_phase_compare_is_atomic_with_prepare_even_when_generation_did_not_change(self):
        from session import NavigationSession
        import time
        s=NavigationSession();s.phase='arrived';s.generation=3
        now=time.monotonic();s.operator('manual',True,True,False,1,1,now,now)
        s.operator('navigation_allowed',True,True,False,1,2,now+.01,now+.01)
        with self.assertRaises(ValueError):s.prepare(dict(x=1,y=0,yaw=0,_sdk_expected_generation=3,_sdk_expected_phase='arrived'),now+.01)
        self.assertEqual(s.generation,3);self.assertEqual(s.phase,'manual')
    def test_rotation_endpoint_preserves_previous_phase_compare(self):
        setup=rotations.RotationTests();c=setup.setup_console();self.addCleanup(setup.doCleanups)
        c.reception_pose=lambda:[0.,0.,0.];c.checked_pose=lambda d,**kw:object()
        c.session.phase='manual'
        with self.assertRaises(ValueError):c.prepare_rotation(dict(radians=1.,_sdk_expected_phase='arrived'))
        self.assertFalse(c.nav.sent)
