"""Exercise the actual Console publication and profile without ROS or motors."""
from pathlib import Path
from types import SimpleNamespace as NS
import threading, unittest
from unittest.mock import Mock
from test_console_cancellation import Console
from reception_annotations import AnnotationLayer
from configure import config


class MapBoundaryTests(unittest.TestCase):
    def test_unknown_boundary_and_authored_line_survive_dynamic_free_space(self):
        c=Console.__new__(Console)
        cells=[-1 if x<10 else 0 for y in range(40) for x in range(40)]
        c.grid=NS(info=NS(width=40,height=40,resolution=.1,origin=NS(position=NS(x=-2.,y=-2.),orientation=NS(x=0.,y=0.,z=0.))),data=cells,header=NS(frame_id='map'))
        c.map_revision=1;c.overlay_revision=0;c._overlay_key=None;c._overlay_lock=threading.Lock()
        c.reception=NS(map_id=lambda:'m',decisions=NS(states=lambda *_:{},observations=lambda *_:{}))
        c.door_overlay=NS(apply=lambda grid,*_: (list(grid['cells']),{}))
        c.annotation_layer=AnnotationLayer();c.reception_map=Mock();c.keepout_mask=Mock();c.keepout_info=Mock()
        Console.publish_reception_map.__globals__['CostmapFilterInfo']=NS
        line=dict(id='line',kind='keepout',vertices=[[0.,-.5],[0.,.5]],width=.1)
        document=dict(map_id='m',doors=[],annotations=[line])
        c.publish_reception_map(document)
        mask=c.keepout_mask.publish.call_args.args[0]
        self.assertEqual(c.grid.data,cells)
        self.assertEqual(mask.data[20*40+5],100)
        self.assertEqual(mask.data[20*40+30],0)
        self.assertEqual(mask.data[20*40+20],100)
        self.assertEqual(c.overlay_report['map_boundary_cells'],400)
        # A map reload/expanded mapped area produces a new boundary, with no
        # alteration of the underlying sensor map or any point/route catalog.
        c.grid.data=[0]*1600;c.map_revision+=1;c.publish_reception_map(document)
        self.assertEqual(c.keepout_mask.publish.call_args.args[0].data[20*40+5],0)

    def test_temporal_memory_is_global_p2_only(self):
        p=config(gait_profile='p2-flat-model')
        self.assertEqual(p['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['plugin'],'p2_nav_controller::TemporalObstacleLayer')
        self.assertEqual(p['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['plugin'],'nav2_costmap_2d::ObstacleLayer')
        self.assertEqual(config()['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['plugin'],'nav2_costmap_2d::ObstacleLayer')

if __name__=='__main__':unittest.main()
