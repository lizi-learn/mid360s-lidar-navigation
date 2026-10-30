"""Geometry requirements for the published frozen synthetic replay."""
import math


def nominal_stop_footprint(points):
    required=[[-.58,-.72],[1.,-.72],[1.,.72],[-.58,.72]]
    if len(points)!=4 or any(len(point)!=2 or not all(math.isfinite(x) for x in point) for point in points):return False
    # Cyclic shifts and winding reversal describe the same rectangle. A crossed
    # ordering of the same vertices does not describe the required polygon.
    for order in [required,list(reversed(required))]:
        for shift in range(4):
            candidate=order[shift:]+order[:shift]
            if all(abs(a-b)<1e-12 for p,q in zip(points,candidate) for a,b in zip(p,q)):return True
    return False
