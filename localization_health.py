"""Source-time freshness and pose continuity guards for a preview-only mapper."""
import math
INVALID_AGE=1_000_000.
def source_age(now,stamp,receive_age=0.,future_tolerance=.2):
    if not all(math.isfinite(float(x)) for x in (now,stamp,receive_age)) or stamp<=0:
        return INVALID_AGE
    age=now-stamp
    if age < -future_tolerance:return INVALID_AGE
    return max(0.,age,receive_age)
def pose_discontinuous(before,after,source_dt):
    if source_dt < -1e-4:return True
    distance=math.dist(before,after)
    if source_dt<=1e-4:return distance>.02
    return distance>.35 and distance/source_dt>3.
