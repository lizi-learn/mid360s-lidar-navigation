"""Short source-timed observation window for nonrepeating LiDAR ground returns.

    Points stay in the registered world frame. No interpolation, ray filling,
    timestamp renewal, or expansion of the six support patches is performed.
"""
from collections import deque
import math
import numpy as np
from terrain import classify


class SupportHistory:
    def __init__(self, duration=.25):
        if not math.isfinite(duration) or not 0 < duration <= .25:
            raise ValueError('Invalid support observation duration')
        self.duration = duration
        self.frames = deque()
        self.last_stamp = None

    def clear(self):
        self.frames.clear()
        self.last_stamp = None

    def update(self, points, center, yaw, plane, source_stamp, now):
        points = np.asarray(points, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 3 or
                not np.isfinite([source_stamp, now, yaw, *center, *plane]).all()):
            self.clear()
            raise ValueError('Invalid support observations')
        if not -.05 <= now - source_stamp <= .5:
            self.clear()
            raise ValueError('Support source expired or in future')
        if self.last_stamp is not None and source_stamp <= self.last_stamp:
            self.clear()
            raise ValueError('Support source time did not advance')
        self.last_stamp = source_stamp
        near = points[np.isfinite(points).all(axis=1) &
                      (np.linalg.norm(points[:, :2] - center, axis=1) < 1.6)]
        # Bound memory independently of the point rate. Missing samples remain
        # missing if this cap is reached; they are never duplicated to vote.
        self.frames.append((source_stamp, near[:50000].copy()))
        while self.frames and (now - self.frames[0][0] > self.duration or
                               len(self.frames) > 32):
            self.frames.popleft()
        points = (np.concatenate([p for _, p in self.frames]) if self.frames
                  else np.empty((0, 3)))
        ground, _ = classify(points, center, plane)
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                             [math.sin(yaw), math.cos(yaw)]])
        local = (points[:, :2] - center) @ rotation
        current_local = (near[:, :2] - center) @ rotation
        below = near[:, 2] - (near[:, :2] - center) @ np.asarray(plane[:2]) - plane[2] < -.12
        counts, contradicted = [], []
        for x in (.7, 1.05):
            for y in (-.25, 0., .25):
                counts.append(int(np.count_nonzero(ground &
                    (abs(local[:, 0] - x) < .18) & (abs(local[:, 1] - y) < .18))))
                contradicted.append(bool(np.any(below &
                    (abs(current_local[:, 0] - x) < .18) &
                    (abs(current_local[:, 1] - y) < .18))))
        # A newly observed lower surface invalidates older support at that
        # location immediately. Do not let old floor points hide a drop.
        if any(contradicted):
            self.frames.clear()
        observed = all(n >= 2 for n in counts) and not any(contradicted)
        return observed, dict(support_patch_counts=counts,
                              support_patch_lower_returns=contradicted,
                              support_window_s=self.duration,
                              support_window_frames=len(self.frames),
                              support_oldest_source_stamp_s=(self.frames[0][0] if self.frames else None))
