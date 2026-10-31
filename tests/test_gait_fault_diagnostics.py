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


class AcceptanceFileRemovalTests(unittest.TestCase):
    def test_archive_bit_is_not_required_but_new_live_bit_is(self):
        from threading import RLock
        t=GaitTransport.__new__(GaitTransport);t.lock=RLock();t.phase='idle'
        t.feedback=reply(readiness=LOCAL_READY)
        self.assertTrue(t.ready());self.assertFalse(LOCAL_READY&16)
        t.feedback.readiness|=16
        self.assertTrue(t.ready())
        t.feedback.readiness&=~256
        self.assertFalse(t.ready());self.assertIn('实时机身坐标数据未就绪',readiness_error(t.feedback))
        self.assertNotIn('标定未核验',readiness_error(t.feedback))

class ReadinessWireCompatibilityTests(unittest.TestCase):
    def test_live_kinematic_bit_decodes_and_other_unknown_bits_reject(self):
        import dataclasses
        from gait_socket_client import Reply,REPLY,MAGIC,decode_reply
        now=time.monotonic_ns()
        def packet(bits):
            fields={name:0 for name in Reply.__dataclass_fields__}
            fields.update(monotonic_ns=now,readiness=bits,reason='fixture')
            values=list(dataclasses.astuple(Reply(**fields)));values[-1]=values[-1].encode()
            return REPLY.pack(MAGIC,1,*values)
        for bits in (0,255,256,511):
            self.assertEqual(decode_reply(packet(bits),now).readiness,bits)
        for bits in (512,1<<31):
            with self.assertRaises(ValueError):decode_reply(packet(bits),now)

if __name__=='__main__':unittest.main()
