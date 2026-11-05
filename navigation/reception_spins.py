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


def turn_initial(node, engine, epoch, radians, *, now=time.monotonic, sleep=time.sleep):
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
            if state['phase'] != 'arrived':
                raise ValueError(state.get('notice') or '原地转向未完成')
            return node.cancel_reception_leg(epoch)
        if now() >= deadline:
            node.cancel_reception_leg(epoch)
            raise ValueError('原地转向动作超时，已取消本次动作')
        sleep(.05)
    return False
