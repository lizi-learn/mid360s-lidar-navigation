import sys,tempfile,unittest,xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from recovery_tree import recovery_tree
class RecoveryTreeTests(unittest.TestCase):
 def test_policy_waits_and_cancel_nodes_preserved(self):
  nav=Path(__file__).resolve().parents[1]/'navigation'
  with tempfile.TemporaryDirectory()as d:
   for name in ['forward_navigation.xml','forward_position_navigation.xml','forward_through_poses.xml','forward_position_through_poses.xml']:
    base=nav/name;before=base.read_bytes()
    for wait in [8.,25.,600.]:
     target=recovery_tree(base,d,wait);root=ET.parse(target).getroot();recovery=root.find('.//RecoveryNode');self.assertGreaterEqual(int(recovery.get('number_of_retries'))*2,wait)
     self.assertEqual(root.find('.//Wait').get('name'),'ObservedRouteWait');self.assertIsNotNone(root.find('.//FollowPath'));self.assertEqual(before,base.read_bytes());self.assertEqual(target,recovery_tree(base,d,wait))
 def test_invalid_policy_does_not_create_tree(self):
  with tempfile.TemporaryDirectory()as d:
   for value in [True,float('nan'),0,-1,601]:
    with self.assertRaises(ValueError):recovery_tree('missing',d,value)
