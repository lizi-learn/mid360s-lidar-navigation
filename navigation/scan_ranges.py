"""Encode observed height-slice distances within the declared LaserScan range."""
import numpy as np

def encode_ranges(nearest,range_min=.12,range_max=14.):
    values=np.asarray(nearest,dtype=float).copy()
    observed=np.isfinite(values)
    # Empty height-slice bins are unknown. A finite measured return beyond the
    # advertised range proves free space up to that range and is encoded +inf;
    # Nav2's inf_is_valid clearing path projects it at range_max-epsilon.
    values[~observed]=np.nan
    values[observed & (values<range_min)]=np.nan
    values[observed & (values>range_max)]=np.inf
    return values


def polar_ranges(delta, yaw, bins=720, range_min=.12, range_max=14.):
    """Nearest circular angular bin, preserving unknown versus measured free rays."""
    if not isinstance(bins, int) or bins < 4:
        raise ValueError('Invalid scan bin count')
    delta = np.asarray(delta, dtype=float)
    if delta.ndim != 2 or delta.shape[1] != 2 or not np.isfinite(delta).all() or not np.isfinite(yaw):
        raise ValueError('Finite XY obstacle vectors required')
    angles = (np.arctan2(delta[:,1], delta[:,0])-yaw+np.pi) % (2*np.pi)
    indices = np.rint(angles*bins/(2*np.pi)).astype(int) % bins
    ranges = np.full(bins, np.inf)
    np.minimum.at(ranges, indices, np.linalg.norm(delta, axis=1))
    return encode_ranges(ranges, range_min, range_max)
