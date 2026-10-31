"""Actual console waits for lifecycle activation, not just an advertised action."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

ROOT=Path(__file__).resolve().parents[1]
cls=next(n for n in ast.parse((ROOT/'navigation/console.py').read_text()).body if isinstance(n,ast.ClassDef)and n.name=='Console')
method=next(n for n in cls.body if isinstance(n,ast.FunctionDef)and n.name=='wait_planner_active')


class PlannerStartupTests(unittest.TestCase):
    def fixture(self,states,available=True):
        clock=NS(value=10.)
        def sleep(dt):clock.value+=max(dt,.001)
        scope=dict(time=NS(monotonic=lambda:clock.value,sleep=sleep),GetState=NS(Request=lambda:object()))
        exec(compile(ast.Module(body=[method],type_ignores=[]),'console.py','exec'),scope)
        node=NS(queries=0,destroyed=False)
        def service(timeout_sec):
            if not available:sleep(timeout_sec)
            return available
        def call(request):
            state=states[min(node.queries,len(states)-1)];node.queries+=1
            return NS(current_state=NS(id=state))
        client=NS(wait_for_service=service,call_async=call)
        node.create_client=lambda *a:client
        node.destroy_client=lambda c:setattr(node,'destroyed',True)
        node.wait_future=lambda response,timeout:response
        return node,scope['wait_planner_active']

    def test_advertised_endpoint_is_not_used_before_activation_completes(self):
        node,wait=self.fixture([1,2,2,3])
        wait(node,timeout=1.)
        self.assertEqual(node.queries,4)
        self.assertTrue(node.destroyed)

    def test_active_planner_needs_no_extra_delay(self):
        node,wait=self.fixture([3]);wait(node)
        self.assertEqual(node.queries,1)
        self.assertTrue(node.destroyed)

    def test_missing_or_never_active_service_times_out_and_releases_client(self):
        for available in (False,True):
            with self.subTest(available=available):
                node,wait=self.fixture([2],available)
                with self.assertRaises(TimeoutError):wait(node,timeout=.3)
                self.assertTrue(node.destroyed)


if __name__=='__main__':unittest.main()
