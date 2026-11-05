"""Bounded in-place observation recovery inside an already started mission.

Uses the existing live-costmap Spin action and its real completion/cancel
receipts. It neither publishes a motor command nor modifies a map or stamp.
"""
import math
import time


def no_route(error):
    return isinstance(error, ValueError) and '未找到可通行路径' in str(error)


def preferred_side(node, pose, goal):
    bearing = math.atan2(goal['y']-pose[1], goal['x']-pose[0])
    side = 1. if math.remainder(bearing-pose[2], math.tau) >= 0. else -1.
    scan = getattr(node, 'scan_preview', None)
    if not scan or scan.get('frame_id') != 'map':
        return side
    age = (node.get_clock().now().nanoseconds-scan['source_ns'])/1e9
    if not -.05 <= age <= .5:
        return side
    distance = {1.: 3., -1.: 3.}
    c, s = math.cos(pose[2]), math.sin(pose[2])
    for x, y in scan['points']:
        dx, dy = x-pose[0], y-pose[1]
        forward, left = c*dx+s*dy, -s*dx+c*dy
        if -.4 <= forward <= 1.2 and abs(left) > .05:
            k = 1. if left > 0. else -1.
            distance[k] = min(distance[k], math.hypot(dx, dy))
    # This selects a preference, never a clearance authorization. Missing
    # returns do not mark unknown space free; Spin checks the live costmap.
    if abs(distance[1.]-distance[-1.]) > .15:
        return 1. if distance[1.] > distance[-1.] else -1.
    return side


def plan_after_turn(node, engine, epoch, data, original_error, *, now=time.monotonic, sleep=time.sleep):
    """Return a new plan, or None after takeover; never replace the target."""
    def permitted():
        s = node.session_state()
        with engine.lock:
            if not engine.valid(epoch):
                return False
            if (not s['navigation_permitted'] or s.get('manual_activity', False) or
                    s['phase'] in ('manual', 'paused')):
                # Latch the observed intervention before the dispatch worker
                # returns. A quick right->left or middle->left transition must
                # not make the polling thread redispatch this old mission.
                engine.cancel('manual' if s.get('operator_mode') == 'manual' or s.get('manual_activity') or s['phase'] == 'manual' else 'paused')
                return False
            return True

    def available():
        # Cancellation and the next native readiness reply are asynchronous.
        # Use the same bounded stopped wait as ordinary mission dispatch;
        # never relabel an old reply as ready or retry across intervention.
        deadline = now()+8.
        while True:
            if not permitted():
                return False
            healthy, ready = node.health_ready(), node.execution_ready()
            if healthy and ready:
                return True
            if now() >= deadline:
                raise ValueError('原地转向恢复等待真实定位、避障或步态回执超时')
            sleep(.05)

    if not permitted():
        return None
    if not available():
        return None
    pose = node.reception_pose()
    if pose is None:
        raise ValueError('原地转向恢复时没有新鲜位置姿态')
    direction = preferred_side(node, pose, data)
    # Two small, opposite observation turns, each checked by the existing Spin
    # behavior. A live collision abort is a real result, not a completed turn.
    error = original_error
    for side in (direction, -direction):
        if not permitted():
            return None
        if not available():
            return None
        if not node.cancel_reception_leg(epoch):
            return None
        before = node.reception_pose()
        prepared = node.prepare_rotation({'radians': side*math.pi/6})
        generation = prepared['generation']
        if not permitted():
            return None
        node.confirm_goal({'generation': generation, '_mission_epoch': epoch})
        engine.event('heading_recovery_started', generation=generation, requested_radians=side*math.pi/6)
        deadline = now()+45.
        while True:
            s = node.session_state()
            if not permitted():
                return None
            if s['generation'] != generation:
                raise ValueError('原地转向恢复已被其他目标替换')
            if s['phase'] in ('arrived', 'blocked', 'failed', 'cancelled', 'paused', 'stopped'):
                break
            if now() >= deadline:
                if not node.cancel_reception_leg(epoch):
                    return None
                raise ValueError('原地转向未在动作期限内结束，已取消本次恢复')
            sleep(.05)
        after = node.reception_pose()
        moved = None if before is None or after is None else math.remainder(after[2]-before[2], math.tau)
        engine.event('heading_recovery_finished', generation=generation, phase=s['phase'], measured_radians=moved)
        if s['phase'] in ('cancelled', 'paused', 'stopped'):
            raise ValueError(s.get('notice') or '原地转向恢复已取消')
        if not node.cancel_reception_leg(epoch):
            return None
        if not permitted():
            return None
        # A stopped Spin may have genuinely changed the observed direction.
        # Replan against the current data, including after a collision abort.
        # The original goal, yaw, route, and mission epoch stay identical.
        if not available():
            return None
        try:
            result = node.set_goal(data)
            engine.event('heading_recovery_planned', generation=result['generation'])
            return result
        except ValueError as candidate_error:
            if not no_route(candidate_error):
                raise
            engine.event('heading_recovery_replan_failed', error=str(candidate_error))
            error = candidate_error
    raise error
