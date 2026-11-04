import ctypes,math,subprocess,tempfile,unittest,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from native_input_model import NativeInputModel

class NativeInputModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();cls.library=Path(cls.tmp.name)/'input.so'
        root=Path(__file__).resolve().parents[1]
        subprocess.run(['g++','-std=c++17','-O2','-shared','-fPIC','-I'+str(root/'navigation'),str(root/'navigation/p2_nav_controller/src/input_model.cpp'),'-o',str(cls.library)],check=True)
    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()
    def model(self):
        model=NativeInputModel(self.library);self.addCleanup(model.close);return model
    def test_single_curve_after_scale_uses_same_initial_jerk(self):
        full=self.model();half=self.model();now=10**9
        for i in range(25):
            stamp=now+i*20_000_000;a=full.step((.75,.8),(0.,0.),.02,stamp,stamp);b=half.step((.375,.4),(0.,0.),.02,stamp,stamp)
        self.assertAlmostEqual(a[0],.05);self.assertAlmostEqual(b[0],.05)
        self.assertAlmostEqual(a[1],.075);self.assertAlmostEqual(b[1],.075)
    def test_unclassified_zero_immediately_resets_like_production(self):
        m=self.model()
        for i in range(60):m.step((.75,.8),(0.,0.),.02,10**9,10**9)
        self.assertEqual(m.step((0.,0.),(.4,.3),.02,10**9,10**9),(0.,0.))
        self.assertEqual(m.step((.75,.8),(0.,0.),.02,10**9,10**9),(.00008,.00012))
    def test_original_feedback_expiry_and_takeover_are_not_refreshed(self):
        m=self.model();self.assertEqual(m.step((.75,.8),(0.,0.),.02,10**9,1_600_000_000),(0.,0.));self.assertEqual(m.reason,2)
        self.assertEqual(m.step((.75,.8),(0.,0.),.02,10**9,10**9,owned=False),(0.,0.))
