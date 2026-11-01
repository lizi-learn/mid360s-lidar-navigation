"""Exercise the actual reception adapter with no hardware interfaces."""
import copy,importlib.util,sys,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('reception_sandbox',ROOT/'experiments/reception/server.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class TemporaryTargetTests(unittest.TestCase):
    def fixture(self):
        folder=tempfile.TemporaryDirectory();self.addCleanup(folder.cleanup)
        node=module.Sandbox(folder.name);self.addCleanup(node.close)
        # Stop the automatic model loop so each command's snapshot is deterministic.
        node.closed.set();node.thread.join(timeout=2)
        now=__import__('time').monotonic();node.session.operator('navigation_allowed',True,True,False,1,100000,now,now)
        return node
    def test_map_target_is_single_position_goal_not_a_saved_poi(self):
        node=self.fixture();before=copy.deepcopy(node.reception_catalog.read(node.map_name))
        node.reception.command(dict(command='start',target=dict(x=3.,y=.5),revision=before['revision']))
        point=node.reception.engine.steps[0]['point']
        self.assertEqual((point['x'],point['y'],point['yaw']),(3.,.5,None))
        self.assertEqual(node.reception_catalog.read(node.map_name),before)
        self.assertEqual(node.commands,[],'selection/start snapshot does not fake physical motion')
        with self.assertRaises(ValueError):node.reception.command(dict(command='start',target=dict(x=4.,y=0.),revision=before['revision']))
    def test_invalid_target_and_manual_mode_never_start(self):
        for target in (dict(x=True,y=0),dict(x=float('nan'),y=0),dict(x=0),dict(x=1e6,y=0),[]):
            node=self.fixture();rev=node.reception_catalog.read(node.map_name)['revision']
            with self.assertRaises(ValueError):node.reception.command(dict(command='start',target=target,revision=rev))
            self.assertEqual(node.reception.engine.phase,'idle')
        node=self.fixture();now=__import__('time').monotonic();node.session.operator('manual',True,True,False,1,100001,now,now)
        with self.assertRaises(ValueError):node.reception.command(dict(command='start',target=dict(x=1.,y=0.),revision=node.reception_catalog.read(node.map_name)['revision']))
        self.assertEqual(node.reception.engine.phase,'idle')

if __name__=='__main__':unittest.main()
