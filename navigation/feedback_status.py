"""Read installation-bound diagnostic feedback; never grants motion authority."""
import json
import math
from pathlib import Path
import time


def read_neck_feedback(path, identity, *, now_ns=None, boot_id=None):
    unavailable = {'valid': False, 'state': '等待真实颈部反馈',
                   'physical_zero_verified': False, 'kinematics_ready': False}
    if not path or not identity:
        return unavailable
    try:
        p = Path(path)
        if p.stat().st_size > 8192:
            raise ValueError('Oversize feedback')
        d = json.loads(p.read_text())
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip() if boot_id is None else boot_id
        age = (now_ns - d['received_monotonic_ns']) / 1e9
        if (d['schema_version'] != 1 or d['robot_identity'] != identity
                or d['boot_id'] != boot_id or not 0 <= age <= .3
                or d.get('valid') is not True
                or type(d['position_counts']) is not int
                or d['error_code'] != 0 or d['working_counter'] != d['expected_working_counter']):
            raise ValueError('Invalid, stale or differently bound feedback')
        relative = d['fixture_relative_deg']
        if type(relative) not in (int, float) or not math.isfinite(relative):
            raise ValueError('Invalid relative angle')
        # A relative fixture anchor must never become an accepted physical zero.
        return {'valid': True, 'state': f'实测相对前倾 {relative:+.2f}° · 物理零位待标定',
                'position_counts': d['position_counts'], 'fixture_relative_deg': relative,
                'age_s': age, 'statusword': d['statusword'], 'mode_display': d['mode_display'],
                'error_code': 0, 'source': 'received_process_data',
                'physical_zero_verified': False, 'kinematics_ready': False}
    except (OSError, ValueError, KeyError, TypeError, OverflowError):
        return {**unavailable, 'state': '颈部反馈不可用或已过期'}
