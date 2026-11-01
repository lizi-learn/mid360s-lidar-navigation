#!/usr/bin/env python3
"""Generate explicit Nav2 Humble configuration for forward walking and turning."""
from pathlib import Path
import argparse, yaml, json, math
import numpy as np
from footprint import checked,DEFAULT,load as load_footprint

def config(controller='rpp', tree=None, minimum_speed=.03, footprint_vertices=None, odometry_model='differential',gait_profile='generic',stop_side_margin=.20,planning_side_padding=.05,planning_envelope='stop_clearance',footprint_padding=.05):
    if not math.isfinite(stop_side_margin) or not .05<=stop_side_margin<=.20:
        raise ValueError('Stop side margin must be between 0.05 and 0.20 m; body envelope is independent')
    if not math.isfinite(planning_side_padding) or not 0.<=planning_side_padding<=.05:
        raise ValueError('Additional planning side padding must be between 0 and 0.05 m')
    if planning_envelope not in ('body','stop_clearance'):
        raise ValueError('Unknown planning envelope')
    if not math.isfinite(footprint_padding) or not 0.<=footprint_padding<=.05:
        raise ValueError('Footprint padding must be between 0 and 0.05 m')
    if gait_profile not in ('generic','p2-flat-model'):raise ValueError('Unknown gait profile')
    if gait_profile=='p2-flat-model':
        if controller!='rpp':raise ValueError('The experimental P2 model profile requires RPP')
        if minimum_speed>.8:raise ValueError('P2 profile minimum input exceeds the D-policy manual command range of 0.8')
        if footprint_vertices is None:footprint_vertices=load_footprint(Path(__file__).parent/'profiles/p2-model-envelope.json')
        odometry_model='omni'
    if odometry_model not in ('differential','omni'):raise ValueError('Unknown localization odometry model')
    footprint=json.dumps(checked(DEFAULT if footprint_vertices is None else footprint_vertices))
    result={}
    def add(name, params):result[name]={'ros__parameters':params}
    add('amcl',dict(use_sim_time=False,alpha1=.05,alpha2=.05,alpha3=.05,alpha4=.05,alpha5=.05,
        base_frame_id='base_link',global_frame_id='map',odom_frame_id='odom',scan_topic='scan',
        robot_model_type='nav2_amcl::OmniMotionModel' if odometry_model=='omni' else 'nav2_amcl::DifferentialMotionModel',min_particles=500,max_particles=2000,
        max_beams=90,laser_min_range=.12,laser_max_range=14.,laser_model_type='likelihood_field',
        laser_likelihood_max_dist=2.,sigma_hit=.15,z_hit=.8,z_rand=.2,z_short=0.,z_max=0.,
        update_min_d=.05,update_min_a=.05,transform_tolerance=.25,pf_err=.03,pf_z=.99,
        set_initial_pose=False,initial_pose=dict(x=2.,y=2.5,z=0.,yaw=0.),resample_interval=1,
        recovery_alpha_fast=0.,recovery_alpha_slow=0.,tf_broadcast=True))
    add('map_server',dict(use_sim_time=False,yaml_filename=''))
    # RPP bounds rotation about measured body-rate; stepping ripple must not lock that request.
    # The final smoother and command gate independently retain the 0.5 rad/s^2 command ramp.
    rpp=dict(plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        desired_linear_vel=.30,lookahead_dist=.55,min_lookahead_dist=.35,max_lookahead_dist=.8,
        lookahead_time=1.5,rotate_to_heading_angular_vel=.30,transform_tolerance=.15,
        use_velocity_scaled_lookahead_dist=True,min_approach_linear_velocity=max(.03,minimum_speed),
        approach_velocity_scaling_dist=.65,use_collision_detection=True,
        max_allowed_time_to_collision_up_to_carrot=1.5,use_regulated_linear_velocity_scaling=True,
        use_cost_regulated_linear_velocity_scaling=True,cost_scaling_dist=.65,cost_scaling_gain=.7,
        inflation_cost_scaling_factor=3.,regulated_linear_scaling_min_radius=.65,
        regulated_linear_scaling_min_speed=max(.08,minimum_speed),use_rotate_to_heading=True,allow_reversing=False,
        rotate_to_heading_min_angle=.5,max_angular_accel=.5,max_robot_pose_search_dist=3.)
    if gait_profile=='p2-flat-model':
        # Keep useful anticipation while starting/restarting as well as at speed.
        # A larger maximum alone still chases nearby errors at the old floor.
        # Native curvature and collision checks remain.
        rpp.update(plugin='p2_nav_controller::SettledGoalRPP',desired_linear_vel=.8,min_lookahead_dist=1.2,max_lookahead_dist=1.6,
            lookahead_time=2.0,rotate_to_heading_angular_vel=1.0,rotate_to_heading_min_angle=1.2,cost_scaling_dist=1.0,
            max_angular_accel=20.,regulated_linear_scaling_min_radius=1.25,
            regulated_linear_scaling_min_speed=max(.6,minimum_speed),
            use_cost_regulated_linear_velocity_scaling=False,minimum_pivot_input=.5,collision_lookahead_floor=.55)
    mppi=dict(plugin='nav2_mppi_controller::MPPIController',time_steps=48,model_dt=.05,batch_size=1000,
        iteration_count=1,vx_std=.10,vy_std=0.,wz_std=.20,vx_max=.30,vx_min=0.,vy_max=0.,wz_max=.35,
        ax_max=.15,ax_min=-.30,az_max=.5,prune_distance=1.8,transform_tolerance=.15,
        temperature=.3,gamma=.015,motion_model='DiffDrive',visualize=False,reset_period=1.,
        regenerate_noises=False,critics=['ConstraintCritic','ObstaclesCritic','GoalCritic','GoalAngleCritic',
        'PathAlignCritic','PathFollowCritic','PathAngleCritic','PreferForwardCritic'],
        ConstraintCritic=dict(enabled=True,cost_power=1,cost_weight=4.),
        ObstaclesCritic=dict(enabled=True,cost_power=1,repulsion_weight=1.5,critical_weight=20.,
            consider_footprint=True,collision_cost=100000.,collision_margin_distance=.15,
            near_goal_distance=.5,inflation_radius=.75,cost_scaling_factor=3.),
        GoalCritic=dict(enabled=True,cost_power=1,cost_weight=5.,threshold_to_consider=1.5),
        GoalAngleCritic=dict(enabled=True,cost_power=1,cost_weight=3.,threshold_to_consider=.5),
        PathAlignCritic=dict(enabled=True,cost_power=1,cost_weight=10.,max_path_occupancy_ratio=.05,
            trajectory_point_step=4,threshold_to_consider=.5,offset_from_furthest=10,use_path_orientations=True),
        PathFollowCritic=dict(enabled=True,cost_power=1,cost_weight=6.,offset_from_furthest=5,threshold_to_consider=1.5),
        PathAngleCritic=dict(enabled=True,cost_power=1,cost_weight=3.,offset_from_furthest=4,
            threshold_to_consider=.5,max_angle_to_furthest=1.,mode=0),
        PreferForwardCritic=dict(enabled=True,cost_power=1,cost_weight=8.,threshold_to_consider=.5))
    add('controller_server',dict(use_sim_time=False,controller_frequency=20.,min_x_velocity_threshold=.001,
        min_y_velocity_threshold=.001,min_theta_velocity_threshold=.001,failure_tolerance=.3,
        progress_checker_plugin='progress_checker',goal_checker_plugins=['goal_checker'],controller_plugins=['FollowPath'],
        progress_checker=dict(plugin='nav2_controller::SimpleProgressChecker',required_movement_radius=.1,movement_time_allowance=20.),
        goal_checker=dict(plugin='nav2_controller::SimpleGoalChecker',xy_goal_tolerance=.15,yaw_goal_tolerance=.15,stateful=True),
        FollowPath=rpp if controller=='rpp' else mppi))
    for name,frame,rolling in [('local_costmap','odom',True),('global_costmap','map',False)]:
        common=dict(use_sim_time=False,update_frequency=5. if rolling else 1.,publish_frequency=2. if rolling else 1.,
            global_frame=frame,robot_base_frame='base_link',resolution=.05,footprint=footprint,
            footprint_padding=footprint_padding,transform_tolerance=.25,track_unknown_space=True,
            plugins=(['obstacle_layer','inflation_layer'] if rolling else ['static_layer','obstacle_layer','inflation_layer']),
            static_layer=dict(plugin='nav2_costmap_2d::StaticLayer',map_subscribe_transient_local=True),
            obstacle_layer=dict(plugin='nav2_costmap_2d::ObstacleLayer',enabled=True,observation_sources='scan',
                scan=dict(topic='/scan',data_type='LaserScan',clearing=True,marking=True,inf_is_valid=True,
                    raytrace_max_range=12.,raytrace_min_range=.12,obstacle_max_range=11.,obstacle_min_range=.12,
                    expected_update_rate=.3)),
            inflation_layer=dict(plugin='nav2_costmap_2d::InflationLayer',inflation_radius=.75,cost_scaling_factor=3.),
            always_send_full_costmap=True)
        if rolling:common.update(rolling_window=True,width=5,height=5)
        result[name]={name:{'ros__parameters':common}}
    add('planner_server',dict(use_sim_time=False,expected_planner_frequency=1.,planner_plugins=['GridBased'],
        GridBased=dict(plugin='nav2_smac_planner/SmacPlannerLattice',tolerance=.05,allow_unknown=False,
            max_iterations=1000000,max_on_approach_iterations=1000,max_planning_time=3.,
            lattice_filepath='/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json',
            allow_reverse_expansion=False,reverse_penalty=5.,change_penalty=.1,non_straight_penalty=1.2,
            cost_penalty=2.,rotation_penalty=5.,retrospective_penalty=.015,
            lookup_table_size=20.,cache_obstacle_heuristic=False,analytic_expansion_ratio=3.5,
            analytic_expansion_max_length=2.,smooth_path=True,
            smoother=dict(max_iterations=200,w_smooth=.3,w_data=.2,tolerance=1e-10,do_refinement=True))))
    if gait_profile=='p2-flat-model':
        import copy
        planner=result['planner_server']['ros__parameters']
        planner['planner_plugins'].append('PositionOnly')
        planner['PositionOnly']=copy.deepcopy(planner['GridBased'])
        planner['PositionOnly']['plugin']='p2_nav_controller::PositionOnlyLattice'
        planner['PositionOnly']['smooth_path']=False
        planner['PositionOnly']['cache_obstacle_heuristic']=True
        control=result['controller_server']['ros__parameters']
        control['goal_checker_plugins'].append('position_goal_checker')
        control['position_goal_checker']=dict(plugin='p2_nav_controller::PositionGoalChecker',xy_goal_tolerance=.15)
    add('behavior_server',dict(use_sim_time=False,costmap_topic='local_costmap/costmap_raw',
        footprint_topic='local_costmap/published_footprint',cycle_frequency=10.,behavior_plugins=['wait'],
        wait=dict(plugin='nav2_behaviors/Wait'),global_frame='odom',robot_base_frame='base_link',transform_tolerance=.2))
    add('bt_navigator',dict(use_sim_time=False,global_frame='map',robot_base_frame='base_link',odom_topic='/odom',
        bt_loop_duration=10,default_server_timeout=20,wait_for_service_timeout=1000,
        default_nav_to_pose_bt_xml=str(tree or Path(__file__).with_name('forward_navigation.xml')),
        default_nav_through_poses_bt_xml=str(Path(tree).with_name('forward_through_poses.xml') if tree else Path(__file__).with_name('forward_through_poses.xml')),
        plugin_lib_names=['nav2_compute_path_to_pose_action_bt_node','nav2_follow_path_action_bt_node',
            'nav2_wait_action_bt_node','nav2_clear_costmap_service_bt_node','nav2_rate_controller_bt_node',
            'nav2_recovery_node_bt_node','nav2_pipeline_sequence_bt_node',
            'nav2_compute_path_through_poses_action_bt_node','nav2_remove_passed_goals_action_bt_node',
            'nav2_goal_updated_condition_bt_node','nav2_globally_updated_goal_condition_bt_node',
            'nav2_is_path_valid_condition_bt_node','nav2_round_robin_node_bt_node',
            'nav2_spin_action_bt_node','nav2_back_up_action_bt_node']))
    add('velocity_smoother',dict(use_sim_time=False,smoothing_frequency=20.,scale_velocities=False,
        feedback='OPEN_LOOP',max_velocity=[.30,0.,.35],min_velocity=[0.,0.,-.35],
        max_accel=[.15,0.,.5],max_decel=[-.30,0.,-.5],odom_topic='odom',odom_duration=.1,
        deadband_velocity=[0.,0.,0.],velocity_timeout=.3))
    bounds=np.asarray(checked(DEFAULT if footprint_vertices is None else footprint_vertices))
    lo,hi=bounds.min(axis=0),bounds.max(axis=0)
    stop_front=float(hi[0]+(.53 if gait_profile=='p2-flat-model' else .25))
    slow_front=float(hi[0]+(.95 if gait_profile=='p2-flat-model' else .75))
    stop_points=[stop_front,float(hi[1]+stop_side_margin),stop_front,float(lo[1]-stop_side_margin),float(lo[0]-.15),float(lo[1]-stop_side_margin),float(lo[0]-.15),float(hi[1]+stop_side_margin)]
    slow_points=[slow_front,float(hi[1]+.32),slow_front,float(lo[1]-.32),float(lo[0]-.25),float(lo[1]-.32),float(lo[0]-.25),float(hi[1]+.32)]
    add('collision_monitor',dict(use_sim_time=False,base_frame_id='base_link',odom_frame_id='odom',
        cmd_vel_in_topic='/p2_nav/cmd_smooth',cmd_vel_out_topic='/p2_nav/cmd_collision',
        state_topic='/p2_nav/collision_state',transform_tolerance=.15,source_timeout=.5,
        stop_pub_timeout=2.,polygons=['StopZone','SlowZone'],
        StopZone=dict(type='polygon',points=stop_points,
            action_type='stop',max_points=2,visualize=True,polygon_pub_topic='stop_zone'),
        SlowZone=dict(type='polygon',points=slow_points,
            action_type='slowdown',slowdown_ratio=.4,max_points=2,visualize=True,polygon_pub_topic='slow_zone'),
        observation_sources=['scan'],scan=dict(type='scan',topic='/scan',enabled=True)))
    if gait_profile=='p2-flat-model':
        # Use the selected manual-equivalent input, one operator multiplier,
        # and the actual collision STOP. The old cost multiplier and wide
        # SlowZone compounded into sub-actuation inputs (0.39*0.4*0.5).
        result['collision_monitor']['ros__parameters']['polygons']=['StopZone']
        result['planner_server']['ros__parameters']['GridBased']['plugin']='p2_nav_controller::ExactHeadingLattice'
        for costmap in ('local_costmap','global_costmap'):result[costmap][costmap]['ros__parameters']['inflation_layer']['inflation_radius']=1.2
        result['controller_server']['ros__parameters']['progress_checker']['movement_time_allowance']=40.
        result['velocity_smoother']['ros__parameters']['max_velocity'][0]=.8
        result['velocity_smoother']['ros__parameters']['max_accel'][0]=.4
        result['velocity_smoother']['ros__parameters']['max_decel'][0]=-.8
        result['velocity_smoother']['ros__parameters']['max_velocity'][2]=1.
        result['velocity_smoother']['ros__parameters']['min_velocity'][2]=-1.
        # Routes must fit the monitor's stop clearance, rather than repeatedly
        # selecting a physical-footprint gap that the monitor will reject.
        clearance=np.asarray(stop_points).reshape(-1,2).tolist()
        # Global Nav2 padding is applied after this polygon. Keep front/rear
        # padding and reserve at least the monitor/body-padded lateral bounds,
        # without requiring duplicate lateral padding in an explicit override.
        for point in clearance:
            point[1]-=math.copysign(1.,point[1])*(footprint_padding-planning_side_padding)
        if planning_envelope=='stop_clearance':
            result['global_costmap']['global_costmap']['ros__parameters']['footprint']=json.dumps(checked(clearance))
    return result

def apply_runtime_geometry(params_file):
    """Apply a runtime's explicit footprint on every workflow launch.

    Older long-running consoles can regenerate parameters using their cached
    module. Loading this ordinary configuration in the new launch process keeps
    the requested geometry across map/mode changes without restarting the gait.
    An absent override retains the supplied parameters; it is not an access gate.
    """
    path=Path(params_file);override=path.with_name('planning_geometry.json')
    if not override.is_file():return False
    value=json.loads(override.read_text());vertices=checked(value['footprint'])
    padding=value.get('footprint_padding',.05)
    if not isinstance(padding,(int,float)) or not math.isfinite(padding) or not 0<=padding<=.05:
        raise ValueError('Invalid configured footprint padding')
    params=yaml.safe_load(path.read_text())
    # The same configured body must be used by costmaps AND collision polygons.
    # Preserve existing front/rear/side clearances while replacing the body;
    # otherwise an old wide model silently remains a navigation stop condition.
    old_shape=checked(json.loads(params['local_costmap']['local_costmap']['ros__parameters']['footprint']))
    old_lo,old_hi=np.asarray(old_shape).min(axis=0),np.asarray(old_shape).max(axis=0)
    new_lo,new_hi=np.asarray(vertices).min(axis=0),np.asarray(vertices).max(axis=0)
    monitor=params.get('collision_monitor',{}).get('ros__parameters',{})
    for name in ('StopZone','SlowZone'):
        zone=monitor.get(name)
        if not zone or zone.get('type')!='polygon':continue
        points=np.asarray(zone['points'],dtype=float).reshape(-1,2)
        zlo,zhi=points.min(axis=0),points.max(axis=0)
        # Configured zones are axis-aligned rectangles around the body bounds.
        if len(points)!=4 or not all(x in (zlo[0],zhi[0]) and y in (zlo[1],zhi[1]) for x,y in points):
            raise ValueError('Configured collision zone is not a rectangular body-clearance polygon')
        lower=new_lo-(old_lo-zlo);upper=new_hi+(zhi-old_hi)
        zone['points']=[float(upper[0]),float(upper[1]),float(upper[0]),float(lower[1]),
                        float(lower[0]),float(lower[1]),float(lower[0]),float(upper[1])]
    for name in ('local_costmap','global_costmap'):
        grid=params[name][name]['ros__parameters']
        grid['footprint']=json.dumps(vertices);grid['footprint_padding']=padding
    if 'global_obstacle_combination' in value:
        method=value['global_obstacle_combination']
        if type(method) is not int or method not in (0,1):raise ValueError('Invalid obstacle layer combination')
        params['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['combination_method']=method
    path.write_text(yaml.safe_dump(params,sort_keys=False))
    return True

def apply_directional_stop(params_file):
    """Use the actual costmap body and native Nav2 trajectory prediction.

    This is an explicit optional P2 runtime profile. A predicted collision
    produces STOP, rather than a second hidden policy-input slowdown.
    """
    path=Path(params_file);params=yaml.safe_load(path.read_text())
    grid=params['local_costmap']['local_costmap']['ros__parameters']
    shape=checked(json.loads(grid['footprint']));padding=float(grid.get('footprint_padding',0.))
    body=[[x+math.copysign(padding,x),y+math.copysign(padding,y)]for x,y in shape]
    checked(body)
    monitor=params['collision_monitor']['ros__parameters']
    threshold=monitor['StopZone']['max_points']
    monitor['StopZone'].update(points=[v for p in body for v in p],action_type='stop')
    monitor['SweptZone']=dict(type='polygon',action_type='approach',max_points=threshold,
        time_before_collision=1.0,simulation_time_step=.02,
        footprint_topic='/local_costmap/published_footprint',visualize=True,polygon_pub_topic='swept_zone')
    monitor['polygons']=['StopZone','SweptZone']
    monitor['stop_on_predicted_collision']=True
    path.write_text(yaml.safe_dump(params,sort_keys=False))

def main():
    p=argparse.ArgumentParser();p.add_argument('--controller',choices=['rpp','mppi'],default='rpp');p.add_argument('--output',required=True)
    p.add_argument('--minimum-speed',type=float,default=.03)
    p.add_argument('--footprint-profile',default='')
    p.add_argument('--odometry-model',choices=['differential','omni'],default='differential')
    p.add_argument('--gait-profile',choices=['generic','p2-flat-model'],default='generic')
    args=p.parse_args();path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
    if not .03<=args.minimum_speed<=.25:raise ValueError('Minimum speed outside profile range')
    path.write_text(yaml.safe_dump(config(args.controller,minimum_speed=args.minimum_speed,footprint_vertices=load_footprint(args.footprint_profile) if args.footprint_profile else None,odometry_model=args.odometry_model,gait_profile=args.gait_profile),sort_keys=False))
if __name__=='__main__':main()
