import unittest
from configure import config


class LocalStaticContextTests(unittest.TestCase):
    def test_p2_local_map_keeps_mapped_floor_under_dynamic_layer(self):
        p=config(gait_profile='p2-flat-model')['local_costmap']['local_costmap']['ros__parameters']
        self.assertEqual(p['plugins'],['static_layer','obstacle_layer','inflation_layer'])
        self.assertEqual(p['static_layer']['map_topic'],'/p2_nav/reception_map')
        self.assertTrue(p['static_layer']['map_subscribe_transient_local'])
        self.assertTrue(p['track_unknown_space'])
        self.assertEqual(p['obstacle_layer']['plugin'],'nav2_costmap_2d::ObstacleLayer')
        self.assertTrue(p['obstacle_layer']['scan']['marking'])
        self.assertTrue(p['obstacle_layer']['scan']['clearing'])

    def test_generic_profile_unchanged(self):
        p=config()['local_costmap']['local_costmap']['ros__parameters']
        self.assertEqual(p['plugins'],['obstacle_layer','inflation_layer'])


if __name__=='__main__':unittest.main()
