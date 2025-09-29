# radar/parse_wrapper.py
import time
from .parseFrame import parseStandardFrame

def parse_frame(frame_bytes: bytes) -> dict:
    """
    Parse raw frame bytes into structured data.
    Adds a host-side timestamp.
    """
    parsed = parseStandardFrame(frame_bytes)
    parsed['timestamp'] = time.time()
    return parsed
