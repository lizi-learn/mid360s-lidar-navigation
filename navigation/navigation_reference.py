"""Explicit navigation origins; choosing a LiDAR origin is not body calibration."""


def select_reference(requested, has_body_transform):
    if requested == 'auto':
        return 'body_kinematics' if has_body_transform else 'preview'
    if requested not in ('lidar_projection', 'body_kinematics', 'preview'):
        raise ValueError('Unknown navigation reference')
    if (requested == 'body_kinematics') != bool(has_body_transform):
        raise ValueError('Body transform and navigation reference disagree')
    return requested


def reference_status(mode, *, body_verified=False, body_feedback=False):
    """Called only after the adapter has processed a valid fresh source pose.

    lidar_projection defines base_link at the actual LiDAR's XY position, with
    forward yaw from its configured mounting orientation. It does not measure
    the pelvis position or replace joint feedback with a synthetic zero.
    """
    ready = (mode == 'lidar_projection' or
             mode == 'body_kinematics' and body_verified and body_feedback)
    return dict(navigation_reference=mode, navigation_reference_ready=bool(ready),
                body_reference_ready=bool(body_verified),
                kinematic_feedback_ready=bool(body_feedback))


def reference_ready(report):
    if not isinstance(report, dict):
        return False
    mode = report.get('navigation_reference')
    if mode == 'lidar_projection':
        return (report.get('navigation_reference_ready') is True and
                report.get('velocity_ready') is True)
    if mode not in (None, 'body_kinematics'):
        return False
    return (report.get('body_reference_ready') is True and
            report.get('kinematic_feedback_ready') is True and
            (mode is None or report.get('navigation_reference_ready') is True))
