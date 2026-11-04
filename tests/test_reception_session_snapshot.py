"""Task management consumes the same coherent session observation as controls."""
import sys, unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'navigation'))
from reception_mission import Mission, MissionService
from test_reception import fixture

class OneIteration:
    def __init__(self): self.calls = 0
    def wait(self, _):
        self.calls += 1
        return self.calls > 1

class ReceptionSessionSnapshotTests(unittest.TestCase):
    def service(self, state):
        service = MissionService.__new__(MissionService)
        service.engine = Mission()
        epoch = service.engine.start(fixture(), 'tour', 0.)
        service.engine.dispatched(epoch, 7)
        raw = Mock(return_value=dict(state, navigation_permitted=False, operator_mode='unknown'))
        service.node = SimpleNamespace(
            session=SimpleNamespace(status=raw), session_state=Mock(return_value=state),
            reception_speech_result=lambda: None, stop=Mock())
        service.closed = OneIteration()
        service.future = None
        service.capture_resume = Mock()
        service.update_live_route = lambda: None
        return service, raw

    def test_background_task_does_not_pause_a_current_executing_session_from_an_old_view(self):
        state = dict(phase='executing', generation=7, navigation_permitted=True, operator_mode='navigation_allowed')
        service, raw = self.service(state)
        service.run()
        self.assertEqual(service.engine.phase, 'navigating')
        self.assertFalse(service.node.stop.called)
        self.assertFalse(raw.called)

    def test_actual_manual_snapshot_stops_the_attached_navigation(self):
        state = dict(phase='manual', generation=7, navigation_permitted=False, operator_mode='manual')
        service, _ = self.service(state)
        service.run()
        self.assertEqual(service.engine.phase, 'manual')
        service.node.stop.assert_called_once_with()

    def test_actual_pause_snapshot_stops_and_does_not_auto_resume_on_left(self):
        state = dict(phase='paused', generation=7, navigation_permitted=False, operator_mode='paused')
        service, _ = self.service(state)
        service.run()
        self.assertEqual(service.engine.phase, 'paused')
        service.node.stop.assert_called_once_with()
        service.node.session_state.return_value = dict(state, phase='cancelled', navigation_permitted=True, operator_mode='navigation_allowed')
        service.closed = OneIteration()
        service.run()
        self.assertEqual(service.engine.phase, 'paused')
