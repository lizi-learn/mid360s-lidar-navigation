"""Reception heading stages use native Spin, with ordinary authority receipts.

No velocities, motor interface, map edits or source timestamps are synthesized.
The directed planner still checks the requested complete approach and final yaw.
"""
import math
import time


def enabled(node):
    return (getattr(node, 'gait_profile', None) == 'p2-flat-model' and
            getattr(node, 'native_reception_spins', False))


def heading_error(node, point):
    pose = node.reception_pose()
    if pose is None:
        raise ValueError('原地转向缺少当前实测位置姿态')
    return math.remainder(point['yaw'] - pose[2], math.tau)


def initial_error(node):
    """Only replace an actual large initial pivot; forward curves stay curves."""
    pose = node.reception_pose()
    if pose is None:
        raise ValueError('原地转向缺少当前实测位置姿态')
    for x, y in getattr(node, 'path', []):
        if math.hypot(x-pose[0], y-pose[1]) >= .25:
            error = math.remainder(math.atan2(y-pose[1], x-pose[0])-pose[2], math.tau)
            return error if abs(error) > 1.2 else 0.
    return 0.


def permitted(node, engine, epoch):
    state = node.session_state()
    with engine.lock:
        if not engine.valid(epoch):
            return False
        if (not state['navigation_permitted'] or state.get('manual_activity') or
                state['phase'] in ('manual', 'paused')):
            engine.cancel('manual' if state.get('operator_mode') == 'manual' or
                          state.get('manual_activity') or state['phase'] == 'manual' else 'paused')
            return False
    return True


def available(node, engine, epoch, *, now=time.monotonic, sleep=time.sleep):
    deadline = now()+8.
    while permitted(node, engine, epoch):
        if node.health_ready() and node.execution_ready():
            return True
        if now() >= deadline:
            raise ValueError('原地转向等待实际定位、避障或步态回执超时')
        sleep(.05)
    return False


def prepare(node, engine, epoch, radians):
    if not permitted(node, engine, epoch) or not node.cancel_reception_leg(epoch):
        return None
    if not available(node, engine, epoch):
        return None
    result = node.prepare_rotation({'radians': radians})
    if not permitted(node, engine, epoch):
        return None
    node.confirm_goal({'generation': result['generation'], '_mission_epoch': epoch})
    return result


def wait_after_interruption(node, engine, epoch, deadline, *, now=time.monotonic, sleep=time.sleep):
    """A stopped wait, followed by a new live-costmap-checked action.

    Clear monitor samples select when to retry, not geometric permission to
    rotate. Native Spin still checks its full sweep and may reject the retry.
    The original route policy bounds waiting; takeover invalidates its epoch.
    """
    policy=engine.document['policy'];earliest=now()+policy['replan_interval'];clear_since=None
    while permitted(node,engine,epoch):
        stamp=now()
        if stamp>=deadline:raise ValueError('原地转向持续受阻或执行回执未恢复，已结束本次等候')
        clear=node.health_ready()and node.execution_ready()and not getattr(node,'collision_stop',False)
        if not clear:clear_since=None
        elif clear_since is None:clear_since=stamp
        elif stamp>=earliest and stamp-clear_since>=policy['clear_hold']:return True
        sleep(.05)
    return False


def turn_initial(node, engine, epoch, radians, *, now=time.monotonic, sleep=time.sleep):
    pose=node.reception_pose()
    if pose is None:raise ValueError('原地转向缺少当前实测位置姿态')
    target=math.remainder(pose[2]+radians,math.tau);waiting_deadline=None
    result = prepare(node, engine, epoch, radians)
    if result is None:
        return False
    generation = result['generation']
    engine.event('initial_spin_started', generation=generation, radians=radians)
    deadline = now()+45.
    while permitted(node, engine, epoch):
        state = node.session_state()
        if state['generation'] != generation:
            raise ValueError('原地转向已被其他目标替换')
        if state['phase'] in ('arrived', 'blocked', 'failed', 'cancelled', 'stopped'):
            engine.event('initial_spin_finished', generation=generation, phase=state['phase'])
            receipt=state.get('terminal_receipt')or{}
            if(state['phase']=='blocked'and receipt.get('status')==6 and
                    receipt.get('task_kind')=='rotation'and receipt.get('generation')==generation and
                    not state.get('last_stop')):
                if waiting_deadline is None:waiting_deadline=now()+engine.document['policy']['max_wait']
                engine.event('initial_spin_wait',generation=generation,reason=state.get('notice',''))
                if not node.cancel_reception_leg(epoch):return False
                if not wait_after_interruption(node,engine,epoch,waiting_deadline,now=now,sleep=sleep):return False
                current=node.reception_pose()
                if current is None:raise ValueError('原地转向恢复缺少当前实测位置姿态')
                radians=math.remainder(target-current[2],math.tau)
                if abs(radians)<=getattr(node,'presentation_yaw_tolerance',math.radians(10)):
                    # The next original translation still needs its own native
                    # success. Do not synthesize an arrived Spin receipt.
                    engine.event('initial_heading_observed_after_wait',remaining_radians=radians)
                    return True
                result=prepare(node,engine,epoch,radians)
                if result is None:return False
                generation=result['generation'];deadline=now()+45.
                engine.event('initial_spin_retried',generation=generation,radians=radians)
                continue
            if state['phase'] != 'arrived':
                raise ValueError(state.get('notice') or '原地转向未完成')
            return node.cancel_reception_leg(epoch)
        if now() >= deadline:
            node.cancel_reception_leg(epoch)
            raise ValueError('原地转向动作超时，已取消本次动作')
        sleep(.05)
    return False
