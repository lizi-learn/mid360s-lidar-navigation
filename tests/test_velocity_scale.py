"""Halve physical navigation targets, including turns; retain fault priority."""
import ast,json,math,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from navigation.gate import ForwardGate

class VelocityScaleTests(unittest.TestCase):
    def test_half_vector_and_ramp_through_mixed_commands(self):
        full=ForwardGate();half=ForwardGate();half.set_velocity_scale(.5)
        full.armed=half.armed=True;full.set_speed(.15);half.set_speed(.15)
        for command in [(.15,0,.24)]*30+[(.08,0,-.2)]*30+[(0,0,0)]+[(0,0,.24)]*30:
            a=full.step(command,.05,0,0,0);b=half.step(command,.05,0,0,0)
            for x,y in zip(a,b):self.assertAlmostEqual(y,x*.5,places=12)

    def test_half_mode_keeps_immediate_fault_and_stop_zero(self):
        for changes in [{'paused':True},{'ground_ok':False},{'localized':False}]:
            g=ForwardGate();g.armed=True;g.set_velocity_scale(.5)
            for _ in range(30):g.step((.15,0,.24),.05,0,0,0)
            self.assertEqual(g.step((.15,0,.24),.05,0,0,0,**changes),(0.,0.,0.))
        for command in [(0,0,0),(.1,.01,0),(-.01,0,0),(math.nan,0,0)]:
            g=ForwardGate();g.armed=True;g.set_velocity_scale(.5)
            self.assertEqual(g.step(command,.05,0,0,0),(0.,0.,0.))

    def test_lower_scale_caps_immediately_and_raise_ramps(self):
        g=ForwardGate();g.armed=True;g.set_speed(.15)
        for _ in range(100):before=g.step((.15,0,.35),.05,0,0,0)
        g.set_velocity_scale(.5);a=g.step((.15,0,.35),.05,0,0,0)
        self.assertEqual(a,tuple(x*.5 for x in before))
        self.assertLessEqual(a[0],.075);self.assertLessEqual(abs(a[2]),.175)
        g.set_velocity_scale(1.);b=g.step((.15,0,.35),.05,0,0,0)
        self.assertLessEqual(b[0]-a[0],g.limits.accel*.05)
        self.assertLessEqual(abs(b[2]-a[2]),g.limits.angular_accel*.05)

    def test_invalid_factor_cannot_arm_or_change_factor(self):
        g=ForwardGate()
        for v in [0,-.5,2,.75,True,'0.5',math.nan]:
            with self.assertRaises(ValueError):g.set_velocity_scale(v)
            self.assertEqual(g.velocity_scale,1.);self.assertFalse(g.armed)

    def test_actual_console_speed_payload_reaches_actual_gate_callback(self):
        root=Path(__file__).resolve().parents[1]/'navigation'
        def methods(file,cls,names,ns):
            c=next(n for n in ast.parse((root/file).read_text()).body if isinstance(n,ast.ClassDef) and n.name==cls)
            f=[n for n in c.body if isinstance(n,ast.FunctionDef) and n.name in names]
            exec(compile(ast.Module(body=f,type_ignores=[]),str(root/file),'exec'),ns)
        emitted=[];ns={'math':math,'SpeedLimit':lambda:NS()}
        methods('console.py','Console',{'speed','publish_speed'},ns)
        c=NS(minimum_speed=.03,maximum_speed=.15,velocity_scale=1.,op=lambda **kw:emitted.append(kw),limiter=NS(publish=lambda _:None))
        c.publish_speed=lambda:ns['publish_speed'](c)
        reply=ns['speed'](c,.15,.5);self.assertEqual(reply['effective_speed_m_s'],.075)
        gn={'json':json};methods('gate_node.py','GateNode',{'on_operator'},gn)
        n=NS(gate=ForwardGate(),arm_generation=-1,collision_fault_generation=-1)
        gn['on_operator'](n,NS(data=json.dumps({**emitted[-1],'armed':True,'arm_generation':0})))
        self.assertTrue(n.gate.armed);self.assertEqual(n.gate.velocity_scale,.5)
        for _ in range(40):v=n.gate.step((.15,0,.24),.05,0,0,0)
        self.assertEqual(v,(.075,0.,.12))
        before=c.velocity_scale
        with self.assertRaises(ValueError):ns['speed'](c,.10,2.)
        self.assertEqual(c.velocity_scale,before);self.assertEqual(c.speed_value,.15)

if __name__=='__main__':unittest.main()
