"""Finite language/catalogue binding checks; no model/provider invocation."""
import unittest
from navigation_agent.request_scope import request_scope,requested_controls

class Scope(unittest.TestCase):
    def setUp(self):
        self.points=[{'id':'p'+str(n),'name':'点位 '+str(n)}for n in range(1,21)]
        self.routes=[{'id':'reverse','name':'20点逆序挑战 · 20至1','steps':[{'point_id':'p'+str(n)}for n in range(20,0,-1)]},
                     {'id':'forward','name':'20点顺序挑战 · 1至20','steps':[{'point_id':'p'+str(n)}for n in range(1,21)]}]
    def scope(self,text,history=()):return request_scope(text,self.points,self.routes,history)
    def test_all_twenty_targets_with_common_command_forms(self):
        for n in range(1,21):
            for text in (f'去点位{n}',f'请前往点位 {n}',f'到{n}号点',f'再去点位{n}',f'我想去点位{n}'):
                with self.subTest(text=text):self.assertEqual(self.scope(text)['navigation'],{'kind':'visit','point_id':'p'+str(n)})
    def test_numeric_names_do_not_match_their_prefix(self):
        for n in (0,21,101,1001):
            with self.subTest(n=n):self.assertIsNone(self.scope('去点位'+str(n))['navigation'])
    def test_forward_reverse_routes_match_the_complete_saved_order(self):
        for text,route in [('按20到1的路线参观','reverse'),('按20至1参观','reverse'),('启动20点逆序挑战 · 20至1','reverse'),('按1到20的路线参观','forward')]:
            with self.subTest(text=text):self.assertEqual(self.scope(text)['navigation'],{'kind':'route','route_id':route})
    def test_past_tense_destination_questions_do_not_start_a_new_visit(self):
        for text in ('你去点位1了吗？','已经到点位4了吗？','刚才去点位1完成了吗？','为什么去点位1会失败？','点位1怎么去？'):
            with self.subTest(text=text):self.assertIsNone(self.scope(text)['navigation'])
    def test_negated_and_quoted_commands_are_not_navigation(self):
        for text in ('不要去点位1','先别去点位4','只查询：去点位1是否完成，不启动','他说“去点位1”，这是什么意思？'):
            with self.subTest(text=text):self.assertIsNone(self.scope(text)['navigation'])
        self.points.append({'id':'cause','name':'故障原因介绍'})
        self.assertEqual(self.scope('去故障原因介绍')['navigation'],{'kind':'visit','point_id':'cause'})
    def test_short_go_uses_last_user_target_and_not_old_assistant_suggestion(self):
        h=[{'user':'去点位2','assistant':'建议去点位4'}]
        self.assertEqual(self.scope('去',h)['navigation'],{'kind':'visit','point_id':'p2'})
    def test_short_go_without_target_clarifies(self):self.assertTrue(self.scope('去')['error'])
    def test_multiple_points_are_not_silently_reduced_to_first(self):
        self.assertIsNone(self.scope('去点位1，然后去点位4')['navigation'])
        self.assertIsNone(self.scope('去点位1，然后去不存在的地方')['navigation'])
    def test_saved_matching_multi_point_route_is_resolved_once(self):
        self.routes.append({'id':'two','name':'两点路线','steps':[{'point_id':'p1'},{'point_id':'p4'}]})
        self.assertEqual(self.scope('先去点位1，然后去点位4')['navigation'],{'kind':'route','route_id':'two'})
    def test_ambiguous_same_order_route_requires_identity(self):
        self.routes.append({'id':'reverse2','name':'另一个倒序方案','steps':self.routes[0]['steps']})
        self.assertIsNone(self.scope('按20到1的路线参观')['navigation'])
    def test_unrelated_old_history_is_not_a_control_command(self):
        s=self.scope('去点位1',[{'user':'取消当前任务','assistant':'已取消'}])
        self.assertEqual(s['controls'],[])
    def test_control_questions_and_negations_are_readonly(self):
        for text in ('暂停了吗？','恢复了吗？','继续了吗？','不要暂停','请不要停止','取消了吗？'):
            with self.subTest(text=text):self.assertFalse(requested_controls(text))
