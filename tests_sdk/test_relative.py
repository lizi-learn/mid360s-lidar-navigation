import copy,math,unittest
import test_contract as fixtures
from navigation_sdk import NavigationError
from navigation_sdk.validation import task_request

class RelativeContract(unittest.TestCase):
    def setUp(self):fixtures.P2.setUp(self)
    def test_forward_and_rotation_use_conditional_native_actions(self):
        for body,path,key,value in [({'kind':'forward','meters':.5},'/api/relative-goal','meters',.5),
                                   ({'kind':'rotate','radians':-4*math.pi},'/api/rotate','radians',-4*math.pi)]:
            with self.subTest(body=body):
                self.server.state['session']['phase']='idle';self.server.posts.clear()
                ref=self.backend.submit(task_request(body))
                self.assertEqual([p for p,b in self.server.posts],[path,'/api/confirm'])
                self.assertEqual(self.server.posts[0][1][key],value)
                self.assertIn('_sdk_expected',self.server.posts[0][1])
                self.assertEqual(self.backend.observe(ref)['state'],'running')
                self.server.state['session']['phase']='arrived';self.assertEqual(self.backend.observe(ref)['state'],'succeeded')
    def test_map_or_task_replacement_between_read_and_prepare_does_not_start(self):
        original=self.backend._post
        def changed(path,body):
            self.server.state['reception']['map_id']='new-map'
            return original(path,body)
        self.backend._post=changed
        with self.assertRaises(NavigationError):self.backend.submit({'kind':'rotate','radians':2.})
        self.assertNotIn('/api/confirm',[p for p,b in self.server.posts])
    def test_product_order_and_unknown_tail_are_atomic(self):
        before=copy.deepcopy(self.server.document)
        self.server.document['points'].append(dict(before['points'][0],id='b',name='B产品'))
        self.server.document['points'][0]['name']='A产品'
        ref=self.backend.submit({'kind':'tour','markers':['B产品','A产品','B产品']})
        self.assertEqual(self.server.posts[0][1]['reception_command']['point_ids'],['b','p','b'])
        self.assertEqual(ref['type'],'mission')
        self.server.state['reception']['mission']['phase']='completed';self.server.posts.clear()
        with self.assertRaises(NavigationError):self.backend.submit({'kind':'tour','markers':['A产品','不存在']})
        self.assertEqual(self.server.posts,[])
    def test_old_console_does_not_advertise_or_attempt_new_features(self):
        self.backend._extension=lambda:{'conditional_task_control':True}
        self.assertFalse(self.backend.capabilities()['features']['relative_rotation'])
        for body in ({'kind':'forward','meters':1.},{'kind':'rotate','radians':1.},{'kind':'tour','markers':['p']}):
            with self.assertRaises(NavigationError)as exc:self.backend.submit(body)
            self.assertEqual(exc.exception.code,'UNSUPPORTED_CAPABILITY')
        self.assertEqual(self.server.posts,[])
    def test_units_and_exact_fields_reject_invalid_requests(self):
        for data in ({'kind':'forward','meters':0},{'kind':'forward','meters':-1},{'kind':'forward','meters':True},
                     {'kind':'rotate','radians':float('nan')},{'kind':'rotate','degrees':90},
                     {'kind':'tour','markers':[]},{'kind':'tour','markers':['a','']},
                     {'kind':'forward','meters':2,'seconds':2}):
            with self.subTest(data=data),self.assertRaises(NavigationError):task_request(data)
