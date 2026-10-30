"""Read-only frozen-map ComputePath replay. Never arms or requests navigation."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from rcl_interfaces.srv import GetParameters, DescribeParameters
from replay_contract import nominal_stop_footprint


def wait(node,future,timeout):
    deadline=time.monotonic()+timeout
    while not future.done() and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
    if not future.done():raise TimeoutError('Bounded planner replay request timed out')
    return future.result()


def parameter_value(value):
    return {1:value.bool_value,2:value.integer_value,3:value.double_value,
            4:value.string_value,8:list(value.double_array_value)}.get(value.type)


def parameters(node,name,keys):
    client=node.create_client(GetParameters,'/'+name+'/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=10.):raise TimeoutError(name+' parameters unavailable')
        request=GetParameters.Request();request.names=keys
        values=wait(node,client.call_async(request),10.).values
        return {key:parameter_value(value) for key,value in zip(keys,values)}
    finally:node.destroy_client(client)


def pose(node,values):
    result=PoseStamped();result.header.frame_id='map';result.header.stamp=node.get_clock().now().to_msg()
    result.pose.position.x,result.pose.position.y=map(float,values[:2])
    result.pose.orientation.z=math.sin(values[2]/2);result.pose.orientation.w=math.cos(values[2]/2)
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True,type=Path);args=parser.parse_args()
    output=args.output;output.mkdir(parents=True,exist_ok=False)
    proof=json.loads((Path(__file__).resolve().parent/'validation-2026-10-30.json').read_text())
    expected=proof['synthetic_map_fixture']
    rclpy.init();node=Node('gait_space_frozen_replay');commands=[]
    subscription=node.create_subscription(Twist,'/p2_nav/cmd_safe',lambda value:commands.append([value.linear.x,value.linear.y,value.angular.z]),10)
    result={'scope':'Read-only ComputePath queries; never arm, NavigateToPose or physical acceptance','queries':[],'navigation_accepted':False}
    def save_map(label):
        client=node.create_client(GetCostmap,'/global_costmap/get_costmap')
        try:
            if not client.wait_for_service(timeout_sec=10.):raise TimeoutError('Global costmap unavailable')
            message=wait(node,client.call_async(GetCostmap.Request()),10.).map
        finally:node.destroy_client(client)
        metadata=message.metadata
        grid=np.asarray(message.data,dtype=np.uint8).reshape(metadata.size_y,metadata.size_x)
        if hashlib.sha256((grid>=254).tobytes()).hexdigest()!=expected['lethal_mask_sha256']:
            raise RuntimeError('The published frozen lethal/unknown mask was changed')
        np.savez_compressed(output/(label+'.npz'),data=grid,resolution=float(metadata.resolution),origin=[metadata.origin.position.x,metadata.origin.position.y],size=[metadata.size_x,metadata.size_y])
        return {'lethal_mask_exact':True,'resolution':float(metadata.resolution)}
    try:
        action=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        if not action.wait_for_server(timeout_sec=30.):raise TimeoutError('Planner action unavailable')
        planner=parameters(node,'planner_server',['GridBased.plugin','GridBased.lattice_filepath','GridBased.lookup_table_size','GridBased.smooth_path','GridBased.allow_reverse_expansion'])
        grid=parameters(node,'global_costmap/global_costmap',['footprint','footprint_padding','inflation_layer.inflation_radius','obstacle_layer.enabled'])
        result['effective']={'planner_server':planner,'global_costmap/global_costmap':grid}
        if planner['GridBased.plugin']!='p2_nav_controller::GaitFeasibleLattice' or planner['GridBased.lookup_table_size']!=.05 or planner['GridBased.smooth_path'] or planner['GridBased.allow_reverse_expansion']:
            raise RuntimeError('Unexpected experimental planner parameters')
        if grid['footprint_padding']!=.05 or grid['inflation_layer.inflation_radius']<1.4 or grid['obstacle_layer.enabled']:
            raise RuntimeError('Frozen replay requires original full padding, sufficient inflation and no dynamic overlay')
        if not nominal_stop_footprint(json.loads(grid['footprint'])):
            raise RuntimeError('Unexpected stopping footprint')
        heading_count=32 if planner['GridBased.lattice_filepath'].endswith('pivot32.json') else 8 if planner['GridBased.lattice_filepath'].endswith('pivot8.json') else None
        cases=next(x for x in proof['frozen_synthetic_map_queries'] if x['headings']==heading_count)
        if len(cases['queries'])!=2 or {case['name'] for case in cases['queries']}!={'detour_start','settled_failure'}:
            raise RuntimeError('Expected both frozen failure queries')
        names=['GridBased.gait_max_midroute_rotation','GridBased.gait_min_forward_leg','GridBased.gait_terminal_region']
        actual=parameters(node,'planner_server',names);result['effective']['planner_server'].update(actual)
        if abs(actual[names[0]]-math.pi/2)>1e-9 or actual[names[1]]!=.5 or actual[names[2]]!=.3:raise RuntimeError('Unexpected gait bounds')
        client=node.create_client(DescribeParameters,'/planner_server/describe_parameters')
        try:
            if not client.wait_for_service(timeout_sec=5.):raise TimeoutError('Parameter descriptors unavailable')
            request=DescribeParameters.Request();request.names=names;descriptors=wait(node,client.call_async(request),10.).descriptors
            result['gait_parameters_read_only']={k:v.read_only for k,v in zip(names,descriptors)}
            if len(descriptors)!=3 or not all(v.read_only for v in descriptors):raise RuntimeError('Gait bounds must be read-only')
        finally:node.destroy_client(client)
        result['pre_map']=save_map('before')
        for case in cases['queries']:
            request=ComputePathToPose.Goal();request.start=pose(node,case['start']);request.goal=pose(node,case['goal']);request.use_start=True;request.planner_id='GridBased'
            started=time.monotonic();handle=wait(node,action.send_goal_async(request),10.)
            if not handle.accepted:raise RuntimeError('Planner query rejected')
            answer=wait(node,handle.get_result_async(),30.)
            path=[]
            for item in answer.result.path.poses:
                q=item.pose.orientation
                path.append([item.pose.position.x,item.pose.position.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z)])
            (output/(case['name']+'-path.json')).write_text(json.dumps(path))
            result['queries'].append({'name':case['name'],'start':case['start'],'goal':case['goal'],'status':answer.status,'elapsed_s':time.monotonic()-started,'poses':len(path)})
            if answer.status!=case['status']:raise RuntimeError('Replayed planning outcome changed')
            if answer.status==4 and (not path or math.hypot(path[-1][0]-case['goal'][0],path[-1][1]-case['goal'][1])>1e-9):
                raise RuntimeError('Returned path does not end at the requested goal')
        result['post_map']=save_map('after')
        deadline=time.monotonic()+3.
        while not commands and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        result['observed_safe_command_count']=len(commands)
        result['all_safe_commands_zero']=bool(commands) and all(all(value==0. for value in command) for command in commands)
        if not result['all_safe_commands_zero']:raise RuntimeError('A safe command was nonzero or no command stream was observed')
        result['planning_replay_matched']=True
    except Exception as error:
        result['error']=str(error);raise
    finally:
        (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        node.destroy_subscription(subscription);node.destroy_node();rclpy.shutdown()
    print(json.dumps({'statuses':[x['status'] for x in result['queries']],'planning_replay_matched':True,'all_safe_commands_zero':True}))


if __name__=='__main__':main()
