# radar/serial_reader.py
import time
import struct
from collections import deque

UART_MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'

def read_n_bytes(ser, n, timeout_s=2.0):
    """
    Read exactly n bytes from serial.
    Returns bytes or None on timeout.
    """
    buf = bytearray()
    start = time.time()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            if (time.time() - start) >= timeout_s:
                return None
            continue
        buf.extend(chunk)
    return bytes(buf)

def wait_for_magic(ser, timeout_s=5.0):
    """
    Wait for the magic word in the serial stream.
    Returns True if found, False on timeout.
    """
    dq = deque(maxlen=len(UART_MAGIC_WORD))
    start = time.time()
    while True:
        b = ser.read(1)
        if not b:
            if (time.time() - start) >= timeout_s:
                return False
            continue
        dq.append(b[0])
        if len(dq) == len(UART_MAGIC_WORD) and bytes(dq) == UART_MAGIC_WORD:
            return True

def read_one_frame(ser):
    """
    Reads one radar frame (header + payload).
    Returns full frame bytes or None if error.
    """
    if not wait_for_magic(ser):
        return None

    hdr_len = 8 + 8 * 4  # Q + 8I = 40 bytes
    rest = read_n_bytes(ser, hdr_len - len(UART_MAGIC_WORD), timeout_s=2.0)
    if rest is None:
        return None
    full_hdr = UART_MAGIC_WORD + rest

    try:
        magic, version, totalPacketLen, platform, frameNum, timeCPUCycles, \
        numDetectedObj, numTLVs, subFrameNum = struct.unpack('<Q8I', full_hdr)
    except Exception as e:
        print("[ERROR] Could not unpack frame header:", e)
        return None

    payload_len = int(totalPacketLen) - hdr_len
    if payload_len < 0:
        print("[ERROR] Invalid payload length:", payload_len)
        return None

    payload = read_n_bytes(ser, payload_len, timeout_s=5.0)
    if payload is None:
        return None

    return full_hdr + payload
