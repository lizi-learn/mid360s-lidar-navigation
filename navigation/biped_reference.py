"""Timestamped head-to-body kinematics for navigation, independent of ROS.

URDF and mount transforms are inputs, never evidence of physical calibration.
T_parent_child maps coordinates in child into parent. Quaternions use xyzw.
"""
from bisect import bisect_left
from pathlib import Path
import math
import json
import hashlib
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def rigid(rotation=None, translation=(0., 0., 0.)):
    t = np.eye(4)
    if rotation is not None:t[:3, :3] = rotation
    t[:3, 3] = translation
    return validate_transform(t)


def validate_transform(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1., atol=1e-6)):
        raise ValueError('Invalid proper rigid transform')
    return t.copy()


def inverse(value):
    t = validate_transform(value)
    return rigid(t[:3, :3].T, -t[:3, :3].T @ t[:3, 3])


def ground_pose(world_body):
    t = validate_transform(world_body)
    if t[2, 2] < math.cos(math.radians(25)):raise ValueError('Body tilt exceeds flat-ground navigation envelope')
    if np.linalg.norm(t[:2, 0]) < .25:raise ValueError('Body forward axis is nearly vertical')
    return np.array([t[0, 3], t[1, 3], math.atan2(t[1, 0], t[0, 0])])


class KinematicChain:
    def __init__(self, urdf, base, tip, fixed_overrides=()):
        root = ET.parse(Path(urdf)).getroot()
        parents = {}
        overrides = set(fixed_overrides)
        for j in root.findall('joint'):
            child, parent = j.find('child'), j.find('parent')
            if child is not None and parent is not None:
                if child.attrib['link'] in parents:raise ValueError('URDF child has multiple parents')
                parents[child.attrib['link']] = (parent.attrib['link'], j)
        self.joints = []
        seen = set()
        while tip != base:
            if tip in seen or tip not in parents:raise ValueError('No acyclic base-to-tip chain')
            seen.add(tip)
            parent, j = parents[tip]
            kind, name = j.attrib['type'], j.attrib['name']
            if j.find('mimic') is not None:raise ValueError('Mimic joints need an explicit adapter')
            if name in overrides:
                if kind != 'fixed':raise ValueError('Override is only for an explicitly documented fixed joint')
                kind = 'revolute';overrides.remove(name)
            if kind not in ('fixed', 'revolute', 'continuous', 'prismatic'):raise ValueError('Unsupported joint type')
            origin, axis, limit = j.find('origin'), j.find('axis'), j.find('limit')
            xyz = np.fromstring(origin.get('xyz', '0 0 0') if origin is not None else '0 0 0', sep=' ')
            rpy = np.fromstring(origin.get('rpy', '0 0 0') if origin is not None else '0 0 0', sep=' ')
            vector = np.fromstring(axis.get('xyz', '1 0 0') if axis is not None else '1 0 0', sep=' ')
            if xyz.shape != (3,) or rpy.shape != (3,) or vector.shape != (3,) or not np.isfinite(np.r_[xyz,rpy,vector]).all():raise ValueError('Invalid joint geometry')
            if np.linalg.norm(vector) < 1e-9:raise ValueError('Zero joint axis')
            bounds = None
            if kind in ('revolute', 'prismatic'):
                if limit is None:raise ValueError('Bounded joint needs limits')
                bounds = (float(limit.attrib['lower']), float(limit.attrib['upper']))
                if not all(math.isfinite(v) for v in bounds) or bounds[0] > bounds[1]:raise ValueError('Invalid joint limits')
            self.joints.append((name,kind,rigid(Rotation.from_euler('xyz',rpy).as_matrix(),xyz),vector/np.linalg.norm(vector),bounds))
            tip = parent
        self.joints.reverse()
        if overrides:raise ValueError('Override is not in the selected chain')
        self.required = tuple(j[0] for j in self.joints if j[1] != 'fixed')

    def transform(self, positions):
        result = np.eye(4)
        for name,kind,origin,axis,bounds in self.joints:
            result = result @ origin
            if kind == 'fixed':continue
            if name not in positions:raise ValueError('Missing joint: '+name)
            value = float(positions[name])
            if not math.isfinite(value) or (bounds and not bounds[0]-1e-6 <= value <= bounds[1]+1e-6):raise ValueError('Joint outside valid range: '+name)
            result = result @ (rigid(translation=axis*value) if kind == 'prismatic' else rigid(Rotation.from_rotvec(axis*value).as_matrix()))
        return result


class JointHistory:
    """Interpolate measured joints at the LiDAR odometry stamp; no stale fallback."""
    def __init__(self, required, max_gap=.10):
        self.required = tuple(required);self.max_gap = max_gap;self.rows = []

    def add(self, stamp, positions):
        stamp = float(stamp)
        values = np.array([positions[k] for k in self.required], dtype=float)
        if not math.isfinite(stamp) or not np.isfinite(values).all():raise ValueError('Invalid joint sample')
        if self.rows and stamp <= self.rows[-1][0]:raise ValueError('Joint source time did not advance')
        self.rows.append((stamp,values));self.rows = self.rows[-1000:]

    def at(self, stamp):
        rows = self.rows[:]
        if not rows:raise ValueError('Joint feedback unavailable')
        times = [r[0] for r in rows];i = bisect_left(times, stamp)
        if i < len(times) and abs(times[i]-stamp) < 1e-6:return dict(zip(self.required,rows[i][1]))
        if i == 0 or i == len(times):raise ValueError('Joint feedback does not bracket odometry time')
        a,b = rows[i-1],rows[i]
        if b[0]-a[0] > self.max_gap:raise ValueError('Joint feedback gap exceeds limit')
        values = a[1]+(b[1]-a[1])*(stamp-a[0])/(b[0]-a[0])
        return dict(zip(self.required, values))


def body_from_lidar(world_lidar, base_tip, tip_lidar):
    return validate_transform(world_lidar) @ inverse(validate_transform(base_tip) @ validate_transform(tip_lidar))


class BodyReference:
    """Explicit model/mount profile. Verification metadata never enables motors."""
    def __init__(self, profile):
        profile = Path(profile).resolve();data = json.loads(profile.read_text())
        if data.get('schema_version') != 1:raise ValueError('Unsupported body reference profile')
        urdf = Path(data['urdf'])
        if not urdf.is_absolute():urdf = profile.parent / urdf
        expected = data.get('urdf_sha256')
        if not isinstance(expected,str) or hashlib.sha256(urdf.read_bytes()).hexdigest() != expected:raise ValueError('URDF identity differs from calibrated profile')
        self.chain = KinematicChain(urdf,data['base_link'],data['tip_link'],data.get('fixed_joint_overrides',[]))
        self.mount = validate_transform(data['tip_to_lidar'])
        self.history = JointHistory(self.chain.required)
        self.verified = data.get('hardware_verified') is True
        self.topic = data.get('joint_topic','/joint_states')
        self.profile_path = profile;self.profile_hash = hashlib.sha256(profile.read_bytes()).hexdigest()

    def transform(self, world_lidar, stamp):
        if hashlib.sha256(self.profile_path.read_bytes()).hexdigest() != self.profile_hash:raise ValueError('Body reference changed; start a new session')
        joints = self.history.at(stamp) if self.chain.required else {}
        body = body_from_lidar(world_lidar,self.chain.transform(joints),self.mount)
        ground_pose(body)
        return body
