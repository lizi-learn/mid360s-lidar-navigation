import math,unittest
import test_agent as fixtures
from navigation_agent.request_scope import request_scope

class RelativeLanguage(unittest.TestCase):
    def scope(self,text):return request_scope(text,[dict(id='a',name='A产品'),dict(id='b',name='B产品')],[])
    def test_metric_units_and_body_forward_expressions(self):
        for text,meters in [('往前走2米',2.),('请前进五十厘米',.5),('向前移动1.5米',1.5),('往前走半米',.5),('向前二点五米',2.5)]:
            with self.subTest(text=text):self.assertEqual(self.scope(text)['navigation'],dict(kind='forward',meters=meters))
    def test_signed_angles_and_multiple_turns(self):
        for text,radians in [('左转90度',math.pi/2),('原地向右转两圈',-4*math.pi),('顺时针旋转半圈',-math.pi),('转一圈',math.tau),('左转一百八十度',math.pi)]:
            with self.subTest(text=text):self.assertAlmostEqual(self.scope(text)['navigation']['radians'],radians)
    def test_queries_negation_and_compounds_do_not_move_partially(self):
        for text in ('不要前进2米','前进2米了吗？','为什么左转90度','能不能支持前进2米和转两圈','先前进2米，然后右转90度','前进2','原地转几圈'):
            with self.subTest(text=text):self.assertIsNone(self.scope(text)['navigation'])
    def test_product_names_and_order_are_bound_to_real_points(self):
        self.assertEqual(self.scope('去介绍A产品')['navigation'],dict(kind='visit',point_id='a'))
        for text in ('去介绍A产品，B产品','先去A产品，然后去B产品','去介绍A产品和B产品'):
            with self.subTest(text=text):self.assertEqual(self.scope(text)['navigation'],dict(kind='tour',point_ids=['a','b']))
        for text in ('去介绍A产品，未知产品','去介绍A产品和未知产品'):
            with self.subTest(text=text):self.assertIsNone(self.scope(text)['navigation'])
        for text in ('去A产品，不要去B产品','去A产品还是B产品？','前进2米，或者左转90度'):
            with self.subTest(text=text):self.assertIsNone(self.scope(text)['navigation'])

class RelativeAgent(unittest.TestCase):
    setUp=fixtures.AgentTest.setUp
    tearDown=fixtures.AgentTest.tearDown
    agent=fixtures.AgentTest.agent
    wait=fixtures.AgentTest.wait
    def test_graph_wrong_units_or_direction_cannot_change_bound_command(self):
        a=self.agent([('relative_motion',{'kind':'rotate','value':90.}),('relative_motion',{'kind':'rotate','value':math.pi/2}),'已转完'])
        a.submit('request-1','test','右转90度');job=self.wait(a)
        self.assertEqual(job['task']['request'],dict(kind='rotate',radians=-math.pi/2))
        self.assertEqual(job['task']['state'],'accepted');self.assertEqual(self.backend.submissions,1)
        self.assertNotIn('已转完',job['reply'])
    def test_graph_forward_missing_model_tool_dispatches_once(self):
        a=self.agent(['可以']);a.submit('request-1','test','往前走50厘米');j=self.wait(a)
        self.assertEqual(j['task']['request'],dict(kind='forward',meters=.5))
        self.assertEqual(self.backend.submissions,1)
    def test_product_tour_preserves_requested_order(self):
        self.backend.points[0]['name']='A产品';self.backend.points[1]['name']='B产品'
        self.backend.routes=lambda:[]
        a=self.agent([('visit_products',{'point_ids':['display','welcome']}),'已完成'])
        a.submit('request-1','test','去介绍A产品，B产品');j=self.wait(a)
        self.assertEqual(j['task']['request'],dict(kind='tour',markers=['welcome','display']))
        self.assertEqual(self.backend.submissions,1);self.assertEqual(j['task']['state'],'accepted')
    def test_status_question_cannot_call_motion_tool(self):
        a=self.agent([('relative_motion',{'kind':'forward','value':2.}),'已处理'])
        a.submit('request-1','test','前进2米了吗？');self.wait(a)
        self.assertEqual(self.backend.submissions,0)
