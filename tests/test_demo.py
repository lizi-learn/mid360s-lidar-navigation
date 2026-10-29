import sys,pathlib,unittest,threading,json,urllib.request,urllib.error
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from demo_server import DemoServer
class DemoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server=DemoServer(('127.0.0.1',0));cls.thread=threading.Thread(target=cls.server.serve_forever,daemon=True);cls.thread.start();cls.base='http://127.0.0.1:'+str(cls.server.server_port)
    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown();cls.server.server_close();cls.thread.join(timeout=2)
    def post(self,path,data):
        request=urllib.request.Request(self.base+path,data=json.dumps(data).encode(),headers={'Content-Type':'application/json'})
        try:
            with urllib.request.urlopen(request) as r:return r.status,json.load(r)
        except urllib.error.HTTPError as e:return e.code,json.load(e)
    def test_demo_identifies_synthetic_data_and_never_enables_execution(self):
        with urllib.request.urlopen(self.base+'/api/status') as r:data=json.load(r)
        self.assertTrue(data['demo']);self.assertFalse(data['execution_enabled']);self.assertFalse(data['survey']['active'])
    def test_route_uses_free_cells_and_unknown_or_obstacle_goals_are_rejected(self):
        code,data=self.post('/api/goal',{'x':3.5,'y':2.5});self.assertEqual(code,200);self.assertFalse(data['execution_enabled'])
        free,_=self.server.floor_map.layers()
        for point in data['path']:self.assertTrue(free[self.server.floor_map.cell(*point)])
        self.assertEqual(self.post('/api/goal',{'x':1.5,'y':1.5})[0],400)
        self.assertEqual(self.post('/api/goal',{'x':50,'y':50})[0],400)
    def test_demo_cannot_start_a_real_recording(self):
        self.assertEqual(self.post('/api/survey/start',{})[0],400)
