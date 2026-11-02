import time,unittest
from types import SimpleNamespace as NS
from test_console_cancellation import Console,fixture

def message(stamp=101.,node='ObservedRouteWait',status='RUNNING'):
 sec=int(stamp);return NS(timestamp=NS(sec=sec,nanosec=round((stamp-sec)*1e9)),event_log=[NS(node_name=node,current_status=status)])
def active():
 c,g=fixture();now=time.monotonic();c.session.planned(g,8,now);c.session.start(g,now,True,True);c.session.started(g,now)
 c.path_revision=7;c.obstacle_path_revision=0;c.bt_goal_source_ns=100*10**9;c.bt_recovery_report={};c.get_clock=lambda:NS(now=lambda:NS(nanoseconds=101020000000))
 return c
class ActualBTFeedbackTests(unittest.TestCase):
 def test_real_wait_starts_stopped_replanning_and_remembers_path_revision(self):
  c=active();c.on_bt_log(message());self.assertEqual(c.session.phase,'replanning');self.assertTrue(c.session.status(time.monotonic())['force_navigation_zero']);self.assertEqual(c.obstacle_path_revision,7);self.assertEqual(c.bt_recovery_report['source_ros_ns'],'101000000000')
 def test_old_future_unrelated_and_nonrunning_receipts_do_not_change_execution(self):
  for msg in [message(99),message(102),message(node='NavigationRecovery'),message(status='SUCCESS')]:
   c=active();c.on_bt_log(msg);self.assertEqual(c.session.phase,'executing')
 def test_right_or_cancelled_task_never_restored_by_late_bt_event(self):
  for mode in ['manual','paused']:
   c=active();now=time.monotonic();c.session.operator(mode,True,True,False,1,2,now,now);c.on_bt_log(message());self.assertEqual(c.session.phase,mode)
 def test_repeated_recovery_events_do_not_restart_maximum_wait_timer(self):
  c=active();c.on_bt_log(message());before=c.session.blocked_since;c.on_bt_log(message());self.assertEqual(c.session.blocked_since,before)
