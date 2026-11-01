import copy,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from reception_live_routes import LiveRouter
from test_reception import fixture
class LiveRoutingTests(unittest.TestCase):
    def command(self,d,ids):
        p={p['id']:p for p in d['points']};return {'points':[p[i]for i in ids]}
    def test_open_shortcut_and_configured_minimum_gain(self):
        d=fixture();r=LiveRouter();c=self.command(d,['b','c'])
        self.assertIsNone(r.choose(d,{'door':'closed'},[0,0,0],c,'a',now=10))
        decision=r.choose(d,{'door':'open'},[0,0,0],c,'a',now=12)
        self.assertEqual([p['id']for p in decision['points']],['c']);self.assertLess(decision['new_cost'],decision['old_cost'])
        d['policy']['shortcut_gain']=.5;r=LiveRouter();self.assertIsNone(r.choose(d,{'door':'open'},[0,0,0],c,'a',now=10))
    def test_door_closure_detours_even_if_longer(self):
        d=fixture();r=LiveRouter();decision=r.choose(d,{'door':'closed'},[0,0,0],self.command(d,['c']),'a',now=10)
        self.assertFalse(decision['wait']);self.assertEqual(decision['points'][-1]['id'],'c');self.assertGreater(len(decision['points']),1)
        self.assertTrue(all(p['kind']=='transit'for p in decision['points'][:-1]))
    def test_no_graph_path_waits_then_reopens(self):
        d=fixture();d['edges']=[e for e in d['edges']if e['door_id']];r=LiveRouter();c=self.command(d,['c'])
        self.assertTrue(r.choose(d,{'door':'closed'},[0,0,0],c,'a',now=10)['wait'])
        self.assertIsNone(r.choose(d,{'door':'closed'},[0,0,0],c,'a',now=11,waiting=True))
        self.assertFalse(r.choose(d,{'door':'open'},[0,0,0],c,'a',now=12,waiting=True)['wait'])
    def test_passed_node_not_reintroduced_by_shortcut(self):
        d=fixture();r=LiveRouter();c=self.command(d,['b','c'])
        self.assertIsNone(r.choose(d,{'door':'open'},[3,0,0],c,'a',remaining=1,now=10))
    def test_repeated_state_does_not_flap(self):
        d=fixture();r=LiveRouter();c=self.command(d,['b','c'])
        self.assertIsNotNone(r.choose(d,{'door':'open'},[0,0,0],c,'a',now=10))
        self.assertIsNone(r.choose(d,{'door':'open'},[0,0,0],c,'a',now=11))
    def test_partial_view_does_not_reverse_current_opening_route(self):
        d=fixture();r=LiveRouter();c=self.command(d,['c'])
        self.assertIsNone(r.choose(d,{'door':'open'},[0,0,0],c,'a',now=10))
        self.assertIsNone(r.choose(d,{'door':'unknown'},[.2,0,0],c,'a',now=14))
        self.assertIsNotNone(r.choose(d,{'door':'closed'},[.2,0,0],c,'a',now=15))
if __name__=='__main__':unittest.main()
