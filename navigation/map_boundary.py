"""Protect map-exterior unknown space without turning interior holes into walls."""
import numpy as np
from scipy.ndimage import binary_propagation


def exterior_unknown_mask(cells, width, height):
    """Return a keepout mask; leave the input occupancy grid untouched.

    Unknown (-1) in this filter is transparent, not observed free space.
    Interior unknowns remain unknown in the source/static map and require real
    laser clearing before an unknown-disallowing planner can traverse them.
    Four-connected exterior unknowns retain the original boundary protection.
    """
    if width <= 0 or height <= 0 or len(cells) != width * height:
        raise ValueError('invalid occupancy grid dimensions')
    unknown = np.asarray(cells).reshape(height, width) < 0
    seeds = np.zeros_like(unknown)
    seeds[0, :] = unknown[0, :]
    seeds[-1, :] = unknown[-1, :]
    seeds[:, 0] = unknown[:, 0]
    seeds[:, -1] = unknown[:, -1]
    exterior = binary_propagation(seeds, mask=unknown)
    exterior_count = int(np.count_nonzero(exterior))
    return np.where(exterior, 100, -1).ravel().tolist(), {
        'map_boundary_cells': exterior_count,
        'interior_unknown_cells': int(np.count_nonzero(unknown)) - exterior_count,
    }
