import unittest,tempfile,sys,uuid,json,shutil,subprocess,os
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_profile import create,validate,check_machine,runtime_path,bind_runtime,check_calibration,identity,workflow_paths,check_simulation_runtime
class RobotProfiles(unittest.TestCase):
 def setUp(self):self.a=create('P2-1003','fixture1003','192.168.1.10','192.168.1.196')
 def test_copied_profile_cannot_be_used_on_another_robot(self):
  check_machine(self.a,'fixture1003')
  with self.assertRaises(ValueError):check_machine(self.a,'fixture1005')
 def test_independent_installations_never_reuse_runtime(self):
  b=create('P2-1003','fixture1003','192.168.1.10','192.168.1.196')
  self.assertNotEqual(runtime_path(self.a,'/tmp/base'),runtime_path(b,'/tmp/base'))
 def test_custom_sensor_runtime_drives_workflow_alignment_location(self):
  sensor,workflow=workflow_paths(self.a,'/tmp/ignored','/tmp/custom-sensor')
  self.assertEqual(sensor,Path('/tmp/custom-sensor'));self.assertEqual(workflow,sensor/'workflow')
  self.assertEqual(workflow_paths(self.a,'/tmp/ignored','/tmp/custom-sensor','/tmp/other-workflow')[1],Path('/tmp/other-workflow'))
 def test_simulation_cannot_write_into_physical_installation(self):
  with tempfile.TemporaryDirectory() as tmp:
   physical=bind_runtime(self.a,Path(tmp)/'physical')
   for path in [physical,physical/'workflow']:
    with self.assertRaises(ValueError):check_simulation_runtime(path)
   check_simulation_runtime(Path(tmp)/'simulation')
 def test_existing_runtime_identity_is_checked_without_overwrite(self):
  with tempfile.TemporaryDirectory() as d:
   bind_runtime(self.a,d);p=Path(d)/'map.pgm';p.write_text('preserve');bind_runtime(self.a,d)
   b=create('P2-1005','fixture1005','192.168.1.11','192.168.1.196')
   with self.assertRaises(ValueError):bind_runtime(b,d)
   self.assertEqual(p.read_text(),'preserve')
 def test_legacy_data_without_identity_is_not_adopted(self):
  with tempfile.TemporaryDirectory() as d:
   (Path(d)/'map_alignment.json').write_text('{}')
   with self.assertRaises(ValueError):bind_runtime(self.a,d)
   self.assertFalse((Path(d)/'robot-identity.json').exists())
 def test_body_calibration_must_match_machine_and_installation(self):
  d={'robot_serial':self.a['serial'],'installation_id':self.a['installation_id']};check_calibration(self.a,d)
  for change in [{'robot_serial':'fixture1005'},{'installation_id':str(uuid.uuid4())}]:
   with self.assertRaises(ValueError):check_calibration(self.a,d|change)
 def test_sensor_launcher_rejects_wrong_robot_before_driver_dispatch(self):
  root=Path(__file__).resolve().parents[1]
  with tempfile.TemporaryDirectory() as tmp:
   d=Path(tmp);(d/'config').mkdir();(d/'scripts').mkdir();(d/'bin').mkdir()
   for name in ['run-stack.sh','robot_profile.py']:shutil.copy2(root/name,d/name)
   shutil.copy2(root/'scripts/configure.py',d/'scripts/configure.py');shutil.copy2(root/'config/MID360S.example.json',d/'config/MID360S.example.json')
   subprocess.run([sys.executable,str(d/'scripts/configure.py'),'--host-ip','192.168.1.10','--lidar-ip','192.168.1.196','--robot-name','P2-1003','--robot-serial','fixture1003'],check=True,stdout=subprocess.DEVNULL)
   p=d/'bin/ros2';p.write_text('#!/bin/sh\ntouch "'+str(d/'driver-dispatched')+'"\n');p.chmod(0o755)
   env=dict(os.environ,PATH=str(d/'bin')+':'+os.environ['PATH']);env.pop('P2_NAV_ROBOT_PROFILE',None);env.pop('P2_NAV_RUNTIME',None)
   result=subprocess.run(['bash',str(d/'run-stack.sh')],env=env,capture_output=True,text=True)
   self.assertNotEqual(result.returncode,0);self.assertRegex(result.stderr,'(identity mismatch|verify this machine serial)');self.assertFalse((d/'driver-dispatched').exists())
 def test_invalid_names_endpoints_and_uuid_rejected(self):
  for key,value in [('name','../1008'),('serial',''),('host_ip','127.0.0.1'),('host_ip','::1'),('lidar_ip',self.a['host_ip']),('installation_id','old')]:
   with self.assertRaises(ValueError):validate(self.a|{key:value})
if __name__=='__main__':unittest.main()
