"""Delayed actual Future replies, cancellation races, and bounded RPC cleanup."""
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock
from pathlib import Path
import unittest,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from planning_requests import RouteReplyTimeout, wait_route_reply


class PlanningReplyTests(unittest.TestCase):
    def fixture(self, response_at=None, cancel_at=None):
        f=Future();clock=NS(now=0.);client=NS(remove_pending_request=Mock())
        def sleep(dt):
            clock.now+=dt
            if response_at is not None and clock.now>=response_at and not f.done():f.set_result('real-response')
        def wait():
            return wait_route_reply(f,client,lambda:cancel_at is None or clock.now<cancel_at,
                                    clock=lambda:clock.now,sleep=sleep)
        return f,client,clock,wait

    def test_queued_response_after_old_two_second_limit_is_consumed(self):
        f,c,t,wait=self.fixture(response_at=3.)
        self.assertEqual(wait(),'real-response');self.assertGreaterEqual(t.now,3.)
        c.remove_pending_request.assert_not_called()

    def test_cancel_during_queue_wait_removes_request_without_waiting_for_reply(self):
        f,c,t,wait=self.fixture(response_at=3.,cancel_at=.1)
        with self.assertRaisesRegex(ValueError,'已取消'):wait()
        self.assertLess(t.now,.15);c.remove_pending_request.assert_called_once_with(f)
        f.set_result('late-response')  # Does not re-enter a completed wait or start an action.

    def test_cancel_wins_over_same_tick_completed_reply(self):
        f,c,t,wait=self.fixture(response_at=.1,cancel_at=.1)
        with self.assertRaises(ValueError):wait()
        self.assertTrue(f.done())

    def test_missing_reply_expires_and_cleans_only_parameter_request(self):
        f,c,t,wait=self.fixture()
        with self.assertRaises(RouteReplyTimeout):wait()
        self.assertGreaterEqual(t.now,8.);self.assertLess(t.now,8.05)
        c.remove_pending_request.assert_called_once_with(f)

    def test_service_error_is_not_reclassified_as_retryable_timeout(self):
        f,c,t,wait=self.fixture();f.set_exception(ValueError('rejected parameters'))
        with self.assertRaisesRegex(ValueError,'rejected parameters'):wait()
        c.remove_pending_request.assert_not_called()
