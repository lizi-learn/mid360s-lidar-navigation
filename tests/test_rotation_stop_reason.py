import time,unittest
from types import SimpleNamespace as NS
import test_console_cancellation as fixture

class RotationStopReasonTests(unittest.TestCase):
    def run_stop(self,reason):
        c,g=fixture.fixture();c.session.rotation_ready(g)
        c.session.start(g,time.monotonic(),True,True)
        self.assertTrue(c.session.started(g,time.monotonic()))
        fixture.ns['CollisionMonitorState']=NS(STOP=1)
        c.on_collision(NS(action_type=1,polygon_name=reason))
        self.assertNotEqual(c.session.phase,'executing')
        self.assertEqual(c.emitted[-1],dict(armed=False,paused=True))
        return c
    def test_source_unavailable_is_not_reported_as_an_obstacle(self):
        c=self.run_stop('source_unavailable')
        self.assertEqual(c.session.phase,'cancelled')
        self.assertIn('观测暂时中断',c.session.message)
        self.assertNotIn('受阻',c.session.message)
    def test_actual_obstacle_still_blocks_rotation(self):
        c=self.run_stop('')
        self.assertEqual(c.session.phase,'blocked')
        self.assertIn('受阻',c.session.message)

if __name__=='__main__':unittest.main()
