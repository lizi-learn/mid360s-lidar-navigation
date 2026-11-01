import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from reception_doors import DoorRays,DoorOverlay
from reception_routes import DoorEvidence
from reception_decisions import RouteDecisions
from reception_door_worker import accept_rows

D={'map_id':'fixture','doors':[{'id':'d','x':0.,'y':2.,'width':1.,'yaw':0.}]}
class DoorEvidenceTests(unittest.TestCase):
    def rays(self):return [[x,4.]for x in [i*.1 for i in range(-10,11)]]
    def test_finite_rays_cover_aperture_and_duplicate_never_refreshes(self):
        e=DoorRays();rows=e.observe(D,[0,0],self.rays(),10.,10.)
        self.assertEqual(rows[0]['state'],'open');self.assertEqual(rows[0]['coverage_bins'],rows[0]['total_bins'])
        self.assertEqual(e.observe(D,[0,0],self.rays(),10.,10.1),[])
        self.assertEqual(e.observe(D,[0,0],self.rays(),8.,10.2),[])
    def test_no_return_or_occluded_partial_view_is_unknown(self):
        for rays in [[],[[0.,1.]],[[0.,4.]]]:
            e=DoorRays();rows=e.observe(D,[0,0],rays,10.,10.)
            self.assertFalse(rows and rows[0]['state']=='open')
    def test_door_leaf_returns_close_immediately_then_reopen_needs_hold(self):
        e=DoorRays();door=DoorEvidence()
        result=e.observe(D,[0,0],[[0.,2.],[.2,2.]],10.,10.)[0];self.assertEqual(result['state'],'closed')
        door.observe(result['state'],10.,1,10.);self.assertEqual(door.current(10.),'closed')
        for seq,stamp in enumerate([10.5,11.,11.5,12.],2):
            r=e.observe(D,[0,0],self.rays(),stamp,stamp)[0];door.observe(r['state'],stamp,seq,stamp)
            if stamp<12:self.assertNotEqual(door.current(stamp),'open')
        self.assertEqual(door.current(12.),'open')
    def test_window_combines_partial_real_scans_then_expires(self):
        e=DoorRays();rays=self.rays();a=e.observe(D,[0,0],rays[:10],10.,10.)[0];self.assertEqual(a['state'],'unknown')
        b=e.observe(D,[0,0],rays[10:],10.4,10.4)[0];self.assertEqual(b['state'],'open')
        c=e.observe(D,[0,0],[[0,4]],11.5,11.5)[0];self.assertEqual(c['state'],'unknown')
    def test_partial_scan_preserves_but_never_refreshes_full_opening_age(self):
        d=dict(D,policy={'clear_hold':1.5});decisions=RouteDecisions();decisions.bind(d)
        for sequence,t in enumerate([10.,10.5,11.,11.5],1):
            accept_rows(decisions,d,[dict(door_id='d',state='open',source_monotonic=t,source_sequence=sequence)],t)
        self.assertEqual(decisions.states(d,11.5)['d'],'open')
        for sequence,t in enumerate([12.,13.,14.,14.6],5):
            accept_rows(decisions,d,[dict(door_id='d',state='unknown',source_monotonic=t,source_sequence=sequence)],t)
        self.assertEqual(decisions.states(d,14.6)['d'],'unknown')
        self.assertEqual(decisions.doors['d'].stamp,11.5)
        accept_rows(decisions,d,[dict(door_id='d',state='closed',source_monotonic=14.7,source_sequence=9)],14.7)
        self.assertEqual(decisions.states(d,14.7)['d'],'closed')
    def test_overlay_retains_measured_opening_on_partial_view_not_fresh_state(self):
        grid={'width':40,'height':40,'resolution':.1,'origin':[-2.,0.],'cells':[0]*1600}
        overlay=DoorOverlay();closed,meta=overlay.apply(grid,D,{'d':'closed'});self.assertGreater(meta['changed_cells'],0)
        grid['cells']=closed;grid['cells'][0]=100;grid['cells'][1]=-1;before=grid['cells'][:]
        opened,report=overlay.apply(grid,D,{'d':'open'});self.assertEqual(opened[0],100);self.assertEqual(opened[1],-1)
        self.assertEqual(report['unknown_cells_promoted'],0);self.assertEqual(grid['cells'],before)
        remembered,meta=overlay.apply(grid,D,{'d':'unknown'});self.assertEqual(remembered,opened)
        self.assertEqual(meta['door_states']['d'],'unknown');self.assertIn('d',meta['last_full_open_observation'])
        restored,_=overlay.apply(grid,D,{'d':'closed'});self.assertEqual(restored,before)
        unknown,_=overlay.apply(grid,D,{'d':'unknown'});self.assertEqual(unknown,before)
    def test_clear_outside_stripe_does_not_erase_nearby_wall(self):
        grid={'width':40,'height':40,'resolution':.1,'origin':[-2.,0.],'cells':[100]*1600}
        overlay=DoorOverlay();opened,_=overlay.apply(grid,D,{'d':'open'})
        self.assertGreater(opened.count(100),1550);self.assertGreater(opened.count(0),0)
    def test_memory_keeps_original_source_and_resets_on_map_reload(self):
        grid={'width':40,'height':40,'resolution':.1,'origin':[-2.,0.],'cells':[100]*1600,'revision':1}
        overlay=DoorOverlay();e={'d':{'state':'open','source_monotonic':11.5,'source_sequence':7}}
        opened,_=overlay.apply(grid,D,{'d':'open'},e)
        remembered,meta=overlay.apply(grid,D,{'d':'unknown'},{'d':{'state':'unknown','source_monotonic':11.5,'source_sequence':7}})
        self.assertEqual(opened,remembered);self.assertEqual(meta['last_full_open_observation']['d']['source_monotonic'],11.5)
        grid['revision']=2;reset,meta=overlay.apply(grid,D,{'d':'unknown'})
        self.assertEqual(reset,grid['cells']);self.assertFalse(meta['last_full_open_observation'])
if __name__=='__main__':unittest.main()
