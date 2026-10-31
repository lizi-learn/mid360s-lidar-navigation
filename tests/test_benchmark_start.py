"""Connect actual benchmark start messages to actual source-fault arm checks."""
import ast
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from gate import ForwardGate

ROOT=Path(__file__).resolve().parents[1]/'navigation'
class SpeedLimit: pass
def method(file, cls, name):
    c=next(x for x in ast.parse((ROOT/file).read_text()).body if isinstance(x,ast.ClassDef) and x.name==cls)
    f=next(x for x in c.body if isinstance(x,ast.FunctionDef) and x.name==name)
    scope={'json':json,'SpeedLimit':SpeedLimit}
    exec(compile(ast.Module(body=[f],type_ignores=[]),str(ROOT/file),'exec'),scope)
    return scope[name]
arm=method('benchmark.py','Bench','arm')
operator=method('gate_node.py','GateNode','on_operator')


class BenchmarkStartTests(unittest.TestCase):
    def fixture(self):
        gate=NS(gate=ForwardGate(),arm_generation=-1,collision_fault_generation=-1,paused=False)
        b=NS(operator=object(),arm_generation=-1,speed_pub=NS(publish=lambda m:None),messages=[])
        def publish(p,d):
            b.messages.append(d.copy());operator(gate,NS(data=json.dumps(d)))
        b.publish=publish
        return b,gate

    def test_explicit_fixture_start_reaches_real_gate(self):
        b,g=self.fixture();arm(b,.15)
        self.assertTrue(g.gate.armed)
        self.assertEqual(g.arm_generation,0)

    def test_source_recovery_does_not_replay_old_start(self):
        b,g=self.fixture();arm(b,.15);old=b.messages[-1]
        g.collision_fault_generation=g.arm_generation;g.gate.armed=False
        operator(g,NS(data=json.dumps(old)))
        self.assertFalse(g.gate.armed)
        arm(b,.15)
        self.assertTrue(g.gate.armed)
        self.assertEqual(g.arm_generation,1)


if __name__=='__main__':unittest.main()
