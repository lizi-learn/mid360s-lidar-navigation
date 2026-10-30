"""Keep Nav2 Humble's trinary PGM unknown pixel unknown on reload."""
import math
from pathlib import Path
import yaml

# Humble writes unknown=205, free=254 and occupied=0, regardless of the
# requested save threshold. 205 has occupancy 50/255, below the .25 default.
FREE_THRESH = .196
OCCUPIED_THRESH = .65


def configure_save_request(request, stem):
    request.map_topic = '/map'
    request.map_url = str(stem)
    request.image_format = 'pgm'
    request.map_mode = 'trinary'
    request.free_thresh = FREE_THRESH
    request.occupied_thresh = OCCUPIED_THRESH
    return request


def validate_map_metadata(path):
    """Validate this workflow's standard, non-negated trinary map convention.

    Do not silently repair third-party images or change map pixels. A legacy
    file must be inspected and copied with corrected metadata before loading.
    """
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict):
        raise ValueError('地图元数据无效')
    if data.get('mode', 'trinary') != 'trinary' or data.get('negate', 0) != 0:
        raise ValueError('工作台要求标准非反色 trinary 地图')
    try:
        free = float(data['free_thresh'])
        occupied = float(data['occupied_thresh'])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('地图占用阈值无效') from exc
    if not (math.isfinite(free) and math.isfinite(occupied)
            and 0 < free <= FREE_THRESH < 50 / 255 <= occupied <= 1):
        raise ValueError('地图阈值会破坏未知区域：请检查原图并修正副本；拒绝载入')
    return data
