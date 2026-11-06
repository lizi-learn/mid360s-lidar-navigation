"""Exercise the actual Console publication and profile without ROS or motors."""
from pathlib import Path
from types import SimpleNamespace as NS
import threading, unittest
from unittest.mock import Mock
from test_console_cancellation import Console
from reception_annotations import AnnotationLayer
from configure import config
from map_boundary import exterior_unknown_mask
from observed_map import ObservedMap


class MapBoundaryTests(unittest.TestCase):
    def test_reset_snapshot_includes_pending_evidence_before_clearing(self):
        c=Console.__new__(Console);c.map_name='m';c.grid=object();events=[]
        c.map_observations=NS(flush=lambda **_:events.append('flush'),set_enabled=lambda _:None,
            reset=lambda:events.append('reset'),status=lambda:{})
        c.map_library=NS(snapshot=lambda *_:events.append('snapshot'))
        c.reception=NS(observation_document=lambda:{})
        c.publish_reception_map=Mock()
        c.reception_save(dict(operation='map_observations_reset',map_id='m'))
        self.assertEqual(events,['flush','snapshot','reset'])

    def test_unknown_boundary_and_authored_line_survive_dynamic_free_space(self):
        c=Console.__new__(Console)
        cells=[-1 if x<10 else 0 for y in range(40) for x in range(40)]
        cells[20*40+27]=-1
        c.grid=NS(info=NS(width=40,height=40,resolution=.1,origin=NS(position=NS(x=-2.,y=-2.),orientation=NS(x=0.,y=0.,z=0.))),data=cells,header=NS(frame_id='map'))
        c.map_revision=1;c.overlay_revision=0;c._overlay_key=None;c._overlay_lock=threading.Lock()
        c.reception=NS(map_id=lambda:'m',decisions=NS(states=lambda *_:{},observations=lambda *_:{}))
        c.door_overlay=NS(apply=lambda grid,*_: (list(grid['cells']),{}))
        c.map_observations=ObservedMap()
        c.annotation_layer=AnnotationLayer();c.reception_map=Mock();c.keepout_mask=Mock();c.keepout_info=Mock()
        Console.publish_reception_map.__globals__['CostmapFilterInfo']=NS
        line=dict(id='line',kind='keepout',vertices=[[0.,-.5],[0.,.5]],width=.1)
        document=dict(map_id='m',doors=[],annotations=[line])
        c.publish_reception_map(document)
        mask=c.keepout_mask.publish.call_args.args[0]
        self.assertEqual(c.grid.data,cells)
        self.assertEqual(mask.data[20*40+5],100)
        self.assertEqual(mask.data[20*40+30],-1)
        self.assertEqual(mask.data[20*40+27],-1)
        self.assertEqual(c.navigation_grid.data[20*40+27],-1)
        self.assertEqual(mask.data[20*40+20],100)
        self.assertEqual(c.overlay_report['map_boundary_cells'],400)
        self.assertEqual(c.overlay_report['interior_unknown_cells'],1)
        # Collecting pending evidence does not change/re-publish the fixed map.
        published=c.reception_map.publish.call_count
        c.map_observations.pending={1:{'index':1}}
        c.publish_reception_map(document)
        self.assertEqual(c.reception_map.publish.call_count,published)
        # A map reload/expanded mapped area produces a new boundary, with no
        # alteration of the underlying sensor map or any point/route catalog.
        c.grid.data=[0]*1600;c.map_revision+=1;c.publish_reception_map(document)
        self.assertEqual(c.keepout_mask.publish.call_args.args[0].data[20*40+5],-1)

    def test_edge_connected_unknown_is_protected_but_enclosed_holes_stay_unknown(self):
        cells=[0]*81
        for x,y in [(0,4),(1,4),(2,4),(3,4),(4,4),(5,5),(6,6),(6,7)]:
            cells[y*9+x]=-1
        before=list(cells);mask,report=exterior_unknown_mask(cells,9,9)
        self.assertEqual(cells,before)
        self.assertEqual(report,dict(map_boundary_cells=5,interior_unknown_cells=3))
        self.assertEqual(mask[4*9+4],100)
        self.assertEqual(mask[5*9+5],-1) # diagonal contact is not a floor boundary
        self.assertEqual(mask[7*9+6],-1)
        self.assertEqual(cells[7*9+6],-1)

    def test_thin_empty_and_invalid_maps(self):
        for width,height in [(1,3),(3,1),(1,1)]:
            mask,report=exterior_unknown_mask([-1]*(width*height),width,height)
            self.assertEqual(mask,[100]*(width*height))
            self.assertEqual(report['interior_unknown_cells'],0)
        self.assertEqual(exterior_unknown_mask([0]*9,3,3)[0],[-1]*9)
        with self.assertRaises(ValueError):exterior_unknown_mask([],0,0)
        with self.assertRaises(ValueError):exterior_unknown_mask([0],2,2)

    def test_temporal_memory_matches_global_and_local_p2_only(self):
        p=config(gait_profile='p2-flat-model')
        self.assertEqual(p['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['plugin'],'p2_nav_controller::TemporalObstacleLayer')
        self.assertEqual(p['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['plugin'],'p2_nav_controller::TemporalObstacleLayer')
        self.assertEqual(config()['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['plugin'],'nav2_costmap_2d::ObstacleLayer')
        self.assertEqual(config()['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['plugin'],'nav2_costmap_2d::ObstacleLayer')

if __name__=='__main__':unittest.main()
