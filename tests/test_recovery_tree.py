import sys,tempfile,unittest,xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from recovery_tree import recovery_tree
class RecoveryTreeTests(unittest.TestCase):
 def test_p2_action_acknowledgement_budget_does_not_change_sensor_expiry(self):
  from configure import config
  self.assertEqual(config(gait_profile='p2-flat-model')['bt_navigator']['ros__parameters']['default_server_timeout'],200)
  self.assertEqual(config()['bt_navigator']['ros__parameters']['default_server_timeout'],20)
 def test_path_refresh_retry_does_not_start_a_robot_wait_action(self):
  nav=Path(__file__).resolve().parents[1]/'navigation'
  with tempfile.TemporaryDirectory()as d:
   for name in ['forward_navigation.xml','forward_position_navigation.xml','forward_through_poses.xml','forward_position_through_poses.xml','forward_directed_route.xml']:
    before=(nav/name).read_bytes();target=recovery_tree(nav/name,d,25.,keep_tracking_on_plan_retry=True);root=ET.parse(target).getroot()
    retry=root.find('.//RecoveryNode[@name="RetryPathRefresh"]');self.assertIsNotNone(retry)
    self.assertEqual(retry.get('number_of_retries'),'1');self.assertIsNotNone(retry.find('./Delay/AlwaysSuccess'))
    self.assertEqual(len(root.findall('.//Wait')),1)
    self.assertEqual(root.find('.//Wait').get('name'),'ObservedRouteWait')
    self.assertEqual(before,(nav/name).read_bytes())
 def test_policy_waits_and_cancel_nodes_preserved(self):
  nav=Path(__file__).resolve().parents[1]/'navigation'
  with tempfile.TemporaryDirectory()as d:
   for name in ['forward_navigation.xml','forward_position_navigation.xml','forward_through_poses.xml','forward_position_through_poses.xml','forward_directed_route.xml']:
    base=nav/name;before=base.read_bytes()
    for wait in [8.,25.,600.]:
     target=recovery_tree(base,d,wait);root=ET.parse(target).getroot();recovery=root.find('.//RecoveryNode');self.assertGreaterEqual(int(recovery.get('number_of_retries'))*2,wait)
     self.assertEqual(root.find('.//Wait').get('name'),'ObservedRouteWait');self.assertIsNotNone(root.find('.//FollowPath'));self.assertEqual(before,base.read_bytes());self.assertEqual(target,recovery_tree(base,d,wait))
 def test_invalid_policy_does_not_create_tree(self):
  with tempfile.TemporaryDirectory()as d:
   for value in [True,float('nan'),0,-1,601]:
    with self.assertRaises(ValueError):recovery_tree('missing',d,value)

class PresentationArrivalTrees(unittest.TestCase):
 def test_only_terminal_checker_changes_and_source_tree_is_preserved(self):
  from configure import config
  import math
  params=config(gait_profile='p2-flat-model')['controller_server']['ros__parameters']
  self.assertEqual(params['presentation_goal_checker']['xy_goal_tolerance'],.30)
  self.assertAlmostEqual(params['presentation_goal_checker']['yaw_goal_tolerance'],math.radians(20))
  self.assertEqual(params['position_goal_checker']['xy_goal_tolerance'],.15)
  self.assertEqual(params['presentation_goal_checker']['plugin'],'nav2_controller::SimpleGoalChecker')
  nav=Path(__file__).resolve().parents[1]/'navigation'
  with tempfile.TemporaryDirectory()as d:
   for name in ['forward_navigation.xml','forward_position_navigation.xml','forward_corridor_directed.xml','forward_corridor_position.xml']:
    base=nav/name;before=base.read_bytes()
    for checker in ['presentation_goal_checker','presentation_position_goal_checker']:
     result=recovery_tree(base,d,25.,checker);tree=ET.parse(result).getroot()
     self.assertEqual(tree.find('.//FollowPath').get('goal_checker_id'),checker)
     self.assertEqual(tree.find('.//Wait').get('name'),'ObservedRouteWait')
     self.assertEqual(before,base.read_bytes())
   with self.assertRaises(ValueError):recovery_tree(nav/'forward_navigation.xml',d,25.,'arbitrary_checker')
