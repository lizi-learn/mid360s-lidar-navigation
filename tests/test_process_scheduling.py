"""Validate actual launch children and unprivileged priority inheritance."""
import ast
import json
import os
from pathlib import Path
import subprocess
import shlex
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'navigation'))
from process_scheduling import workflow_command
import configure


class SchedulingTests(unittest.TestCase):
    def children(self, physical):
        source = Path(__file__).resolve().parents[1] / 'navigation/stack.launch.py'
        function = next(n for n in ast.parse(source.read_text()).body
                        if isinstance(n, ast.FunctionDef) and n.name == 'setup')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            map_file = path / 'map.yaml'
            map_file.touch()
            values = {'mode': 'navigation', 'simulation': 'true',
                      'params': str(path / 'params.yaml'), 'map': str(map_file),
                      'physical_gait': str(physical).lower(), 'gait_handoff': 'true',
                      'simulation_sensor_fixture': 'true'}
            class Config:
                def __init__(self, key): self.key = key
                def perform(self, _): return values[self.key]
            node = lambda **kw: kw
            namespace = {'__file__': str(source), 'Path': Path, 'os': os,
                         'LaunchConfiguration': Config, 'Node': node,
                         'ExecuteProcess': node, 'TimerAction': node,
                         'validate_map_metadata': lambda _: None}
            exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
            with patch.object(configure, 'apply_runtime_geometry'), patch.dict(os.environ, {'P2_NAV_MOTION_BUFFER': 'true'}):
                return namespace['setup'](None)

    def test_physical_launch_keeps_gate_buffer_collision_and_control_normal(self):
        children = self.children(True)
        by_name = {x['name']: x for x in children if 'name' in x}
        self.assertEqual(by_name['amcl']['prefix'], 'nice -n 10')
        self.assertEqual(by_name['planner_server']['prefix'], 'nice -n 10')
        self.assertEqual(shlex.split(by_name['planner_server']['prefix']), ['nice','-n','10'])
        for name in ('controller_server', 'velocity_smoother', 'collision_monitor'):
            self.assertFalse(by_name[name].get('prefix'))
        commands = [x['cmd'] for x in children if 'cmd' in x]
        self.assertTrue(any('gate_node.py' in str(x) for x in commands))
        self.assertTrue(any('motion_buffer_node.py' in str(x) for x in commands))
        self.assertTrue(all(x[0] == '/usr/bin/python3' for x in commands))

    def test_generic_launch_keeps_existing_normal_scheduling(self):
        self.assertTrue(all(not x.get('prefix') for x in self.children(False)))

    @unittest.skipUnless(hasattr(os, 'getpriority'), 'POSIX nice inheritance')
    def test_normal_launcher_can_lower_only_the_planner_without_privileges(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            # This child prints actual priority; its planner child lowers only
            # itself. No renice, root, realtime policy, or real robot is used.
            probe = root / 'probe.py'
            probe.write_text("import os,json,subprocess,sys\n"
                             "print(json.dumps({'command':os.getpriority(os.PRIO_PROCESS,0),"
                             "'planner':int(subprocess.check_output(['nice','-n','10',sys.executable,'-c',"
                             "'import os;print(os.getpriority(os.PRIO_PROCESS,0))']))}))\n")
            (root / 'run.sh').write_text('exec ' + sys.executable + ' ' + str(probe) + '\n')
            args = workflow_command(root, 'navigation', False, root/'params.yaml', root/'map.yaml', True)
            result = json.loads(subprocess.check_output(args))
            parent = os.getpriority(os.PRIO_PROCESS, 0)
            self.assertEqual(result['command'], parent)
            self.assertEqual(result['planner'], min(19, parent + 10))
            self.assertIn('physical_gait:=true', args)


if __name__ == '__main__': unittest.main()
