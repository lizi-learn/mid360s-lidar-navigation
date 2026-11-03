"""Isolated Nav2 + SLAM/AMCL launch. Live mode publishes preview commands only."""
from pathlib import Path
import os,sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parent))
from map_persistence import FREE_THRESH,OCCUPIED_THRESH,validate_map_metadata

def setup(context):
    root=Path(__file__).resolve().parent
    get=lambda k:LaunchConfiguration(k).perform(context)
    mode=get('mode');simulation=get('simulation')=='true';params=get('params');map_file=get('map')
    from process_scheduling import background_prefix
    background=background_prefix(get('physical_gait')=='true')
    from configure import apply_runtime_geometry,apply_directional_stop,apply_input_ramp_routing
    apply_runtime_geometry(params)
    if os.getenv('P2_NAV_DIRECTIONAL_STOP')=='true':apply_directional_stop(params)
    assert mode in ('mapping','navigation')
    common={'use_sim_time':False}
    nodes=[]
    emulated_ramp=simulation and get('gait_handoff')=='true' and get('simulation_sensor_fixture')!='true'
    buffered=os.getenv('P2_NAV_MOTION_BUFFER')=='true' or emulated_ramp
    native_ramp=get('physical_gait')=='true' or emulated_ramp
    apply_input_ramp_routing(params,native_ramp,buffered)
    if get('physical_gait')=='true':nodes.append(ExecuteProcess(cmd=['/usr/bin/python3','-S',str(root/'path_view_node.py')],output='screen'))
    if buffered:nodes.append(ExecuteProcess(cmd=['/usr/bin/python3','-S',str(root/'motion_buffer_node.py'),
        '--ros-args','-p','native_input_ramp:='+str(native_ramp).lower()],output='screen'))
    if get('simulation_sensor_fixture')=='true' and not simulation:raise ValueError('External simulation sensors require simulation mode')
    if simulation and get('simulation_sensor_fixture')!='true':
        shape=yaml.safe_load(Path(params).read_text())['local_costmap']['local_costmap']['ros__parameters']['footprint']
        sim_env={'P2_NAV_FOOTPRINT':shape}
        if native_ramp:sim_env['P2_NAV_NATIVE_INPUT_RAMP']='true'
        if get('gait_handoff')=='true':sim_env['P2_NAV_VELOCITY_WINDOW']=os.getenv('P2_NAV_VELOCITY_WINDOW','.4')
        nodes.append(ExecuteProcess(cmd=['/usr/bin/python3','-S',str(root/'sim_world.py')],additional_env=sim_env,output='screen'))
    elif not simulation:
        alignment=get('alignment')
        assert Path(alignment).is_file(),'Live preview needs the running sensor gravity-alignment file'
        nodes.append(ExecuteProcess(cmd=['/usr/bin/python3','-S',str(root/'live_adapter.py'),
            '--alignment',alignment,'--robot-profile',get('robot_profile'),'--body-reference',get('body_reference'),
            '--navigation-reference',get('navigation_reference'),'--ground-policy',get('ground_policy'),'--source-domain',get('source_domain'),'--target-domain',os.getenv('ROS_DOMAIN_ID','194')]+
            (['--support-history'] if get('support_history')=='true' else []),output='screen'))
    nodes.append(ExecuteProcess(cmd=['/usr/bin/python3','-S',str(root/'gate_node.py'),'--ros-args',
        '-p','simulation:='+str(simulation).lower(),'-p','gait_handoff:='+get('gait_handoff'),'-p','require_localization:='+str(mode=='navigation').lower()],output='screen'))
    if mode=='mapping':
        slam={'use_sim_time':False,'solver_plugin':'solver_plugins::CeresSolver','ceres_linear_solver':'SPARSE_NORMAL_CHOLESKY',
            'ceres_preconditioner':'SCHUR_JACOBI','ceres_trust_strategy':'LEVENBERG_MARQUARDT','ceres_dogleg_type':'TRADITIONAL_DOGLEG',
            'ceres_loss_function':'None','odom_frame':'odom','map_frame':'map','base_frame':'base_link','scan_topic':'/scan',
            'mode':'mapping','resolution':.05,'max_laser_range':14.,'minimum_time_interval':.05,
            'transform_publish_period':.05,'map_update_interval':1.,'transform_timeout':.2,'tf_buffer_duration':30.,
            'stack_size_to_use':40000000,'enable_interactive_mode':False,'use_scan_matching':True,'use_scan_barycenter':True,
            'minimum_travel_distance':.08,'minimum_travel_heading':.08,'scan_buffer_size':20,
            'scan_buffer_maximum_scan_distance':12.,'link_match_minimum_response_fine':.1,
            'link_scan_maximum_distance':1.5,'loop_search_maximum_distance':3.,'do_loop_closing':True,
            'loop_match_minimum_chain_size':10,'loop_match_maximum_variance_coarse':3.,
            'loop_match_minimum_response_coarse':.35,'loop_match_minimum_response_fine':.45,
            'correlation_search_space_dimension':.5,'correlation_search_space_resolution':.01,
            'correlation_search_space_smear_deviation':.1,'loop_search_space_dimension':6.,
            'loop_search_space_resolution':.05,'loop_search_space_smear_deviation':.03,
            'distance_variance_penalty':.5,'angle_variance_penalty':1.,'fine_search_angle_offset':.00349,
            'coarse_search_angle_offset':.349,'coarse_angle_resolution':.0349,'minimum_angle_penalty':.9,
            'minimum_distance_penalty':.5,'use_response_expansion':True}
        if get('resume_file'):
            import json,math
            stem=Path(get('resume_file'));pose=json.loads(get('resume_pose'))
            if not all(Path(str(stem)+suffix).is_file()for suffix in ('.data','.posegraph')):
                raise ValueError('续扫缺少序列化位姿图和扫描数据')
            if not isinstance(pose,list)or len(pose)!=3 or not all(type(v)in(int,float)and math.isfinite(v)for v in pose):
                raise ValueError('续扫起点坐标无效')
            slam.update(map_file_name=str(stem),map_start_pose=[float(v)for v in pose],map_start_at_dock=False)
        nodes.append(Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',parameters=[slam],prefix=background,output='screen'))
        nodes.append(Node(package='nav2_map_server',executable='map_saver_server',name='map_saver',parameters=[dict(use_sim_time=False,save_map_timeout=5.,map_subscribe_transient_local=True,free_thresh_default=FREE_THRESH,occupied_thresh_default=OCCUPIED_THRESH)],output='screen'))
        navigation=['map_saver','velocity_smoother','collision_monitor']
    else:
        assert map_file and Path(map_file).is_file(),'Navigation needs a saved map'
        validate_map_metadata(map_file)
        nodes += [Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[params,{'yaml_filename':map_file}],output='screen'),
                  Node(package='nav2_amcl',executable='amcl',name='amcl',parameters=[params,{'set_initial_pose':simulation}],prefix=background,output='screen'),
                  TimerAction(period=3.,actions=[Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_localization',
                       parameters=[{'use_sim_time':False,'autostart':True,'node_names':['map_server','amcl'],'bond_timeout':4.}],output='screen')])]
        for package,executable in [('nav2_planner','planner_server'),('nav2_controller','controller_server'),
                                   ('nav2_behaviors','behavior_server'),('nav2_bt_navigator','bt_navigator')]:
            nodes.append(Node(package=package,executable=executable,name=executable,parameters=[params],prefix=background if executable=='planner_server' else [],
                remappings=[('cmd_vel','/p2_nav/cmd_raw')],output='screen'))
        navigation=['planner_server','controller_server','behavior_server','bt_navigator','velocity_smoother','collision_monitor']
    if native_ramp:
        navigation.remove('velocity_smoother')
    else:
        nodes.append(Node(package='nav2_velocity_smoother',executable='velocity_smoother',name='velocity_smoother',parameters=[params],
            remappings=[('cmd_vel','/p2_nav/cmd_buffered' if buffered else '/p2_nav/cmd_raw'),('cmd_vel_smoothed','/p2_nav/cmd_smooth')],output='screen'))
    nodes += [Node(package='p2_nav_controller',executable='observed_collision_monitor',name='collision_monitor',parameters=[params],output='screen'),
              TimerAction(period=6.,actions=[Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',
                   parameters=[{'use_sim_time':False,'autostart':True,'node_names':navigation,'bond_timeout':4.}],output='screen')])]
    return nodes

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mode',default_value='navigation'),
        DeclareLaunchArgument('simulation_sensor_fixture',default_value='false'),
        DeclareLaunchArgument('physical_gait',default_value='false'),
        DeclareLaunchArgument('gait_handoff',default_value=os.getenv('P2_NAV_GAIT_HANDOFF','false')),
        DeclareLaunchArgument('simulation',default_value='false'),DeclareLaunchArgument('params'),
        DeclareLaunchArgument('robot_profile',default_value=os.getenv('P2_NAV_ROBOT_PROFILE','')),
        DeclareLaunchArgument('body_reference',default_value=os.getenv('P2_NAV_BODY_CALIBRATION','')),
        DeclareLaunchArgument('navigation_reference',default_value=os.getenv('P2_NAV_REFERENCE','auto')),
        DeclareLaunchArgument('ground_policy',default_value=os.getenv('P2_NAV_GROUND_POLICY','observed_support')),
        DeclareLaunchArgument('support_history',default_value=os.getenv('P2_NAV_SUPPORT_HISTORY','false')),
        DeclareLaunchArgument('map',default_value=''),DeclareLaunchArgument('source_domain',default_value='188'),
        DeclareLaunchArgument('resume_file',default_value=''),DeclareLaunchArgument('resume_pose',default_value='[]'),
        DeclareLaunchArgument('alignment',default_value=os.getenv('P2_LIDAR_ALIGNMENT',str(Path(__file__).resolve().parents[1]/'runtime/map_alignment.json'))),
        OpaqueFunction(function=setup)])
