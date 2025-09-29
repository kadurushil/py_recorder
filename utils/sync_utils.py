"""
utils/sync_utils.py

Small utilities to map radar timestamps to closest camera frame timestamps.
"""

import bisect

def find_closest_index(ts_pairs, target_ts):
    """
    ts_pairs: list of (frame_index, ts) sorted by ts ascending
    target_ts: float timestamp to match
    Returns: (best_frame_index, best_ts, time_diff) or (None, None, None) if no timestamps
    """
    if not ts_pairs:
        return (None, None, None)

    # extract ts list
    ts_list = [t for (_, t) in ts_pairs]
    pos = bisect.bisect_left(ts_list, target_ts)

    candidates = []
    if pos > 0:
        candidates.append(pos - 1)
    if pos < len(ts_list):
        candidates.append(pos)

    best_i = min(candidates, key=lambda i: abs(ts_list[i] - target_ts))
    best_frame_index, best_ts = ts_pairs[best_i]
    time_diff = best_ts - target_ts
    return (best_frame_index, best_ts, time_diff)
