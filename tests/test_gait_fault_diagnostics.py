import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from gait_transport import GaitTransport, LOCAL_READY, readiness_error


def reply(**changes):
    return SimpleNamespace(**{**dict(reason='navigation_disarmed',readiness=LOCAL_READY,
        monotonic_ns=time.monotonic_ns(),operator_verified=True,operator_mode=1,
        manual_activity=False,cancel_pending=False),**changes})


class GaitFaultDiagnosticsTests(unittest.TestCase):
    def test_emergency_reason_reported_before_secondary_missing_readiness(self):
        r=reply(reason='estop',readiness=193)
        text=readiness_error(r)
        self.assertTrue(text.startswith('遥控急停已锁定'))
        self.assertIn('步态尚未进入可行走状态',text)

    def test_emergency_cannot_appear_ready_even_with_old_ready_bits(self):
        t=GaitTransport.__new__(GaitTransport)
        t.lock=threading.RLock();t.feedback=reply(reason='estop');t.phase='idle'
        self.assertFalse(t.ready())
        self.assertTrue(readiness_error(t.feedback))

    def test_healthy_disarmed_reply_keeps_existing_readiness(self):
        t=GaitTransport.__new__(GaitTransport)
        t.lock=threading.RLock();t.feedback=reply();t.phase='idle'
        self.assertTrue(t.ready())
        self.assertEqual(readiness_error(t.feedback),'')

    def test_legacy_reply_without_reason_keeps_conservative_bit_checks(self):
        r=reply(readiness=193);del r.reason
        self.assertIn('硬件状态未就绪',readiness_error(r))


if __name__=='__main__':unittest.main()
