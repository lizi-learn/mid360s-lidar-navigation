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
