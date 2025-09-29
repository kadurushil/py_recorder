"""
camera/camera_recorder.py

Threaded webcam recorder with enforced MJPG format and real-time FPS handling.

Features:
- Requests 1920x1080 @ 30fps by default (configurable).
- Forces MJPG compression to achieve higher fps over USB.
- Prints requested, driver-reported, and measured fps for debugging.
- Writes MP4 video at measured fps (so playback matches real-time).
- Optionally overlays timestamp text on each frame.
- Keeps true capture timestamps in a rolling buffer for sync with radar.
"""

import threading
import time
import cv2
from collections import deque
import os
from typing import Optional, Tuple, List


class CameraRecorder(threading.Thread):
    def __init__(
        self,
        device_index: int = 0,
        out_path: str = "cam.mp4",
        requested_fps: int = 30,
        frame_size: Optional[Tuple[int, int]] = (1920, 1080),
        buffer_len: int = 1000,
        overlay_timestamp: bool = True,
        fourcc_str: str = "mp4v",
        low_power_mode: bool = False,
        warmup_frames: int = 100
    ):
        super().__init__(daemon=True)
        self.device_index = device_index
        self.out_path = out_path
        self.requested_fps = requested_fps
        self.frame_size = frame_size
        self.buffer_len = buffer_len
        self.overlay_timestamp = overlay_timestamp and (not low_power_mode)
        self.fourcc_str = fourcc_str
        self.low_power_mode = low_power_mode
        self.warmup_frames = max(5, int(warmup_frames))

        # state
        self._stop_event = threading.Event()
        self._timestamps_lock = threading.Lock()
        self._timestamps = deque(maxlen=buffer_len)
        self._frame_counter = 0
        self._writer: Optional[cv2.VideoWriter] = None
        self._cap: Optional[cv2.VideoCapture] = None
        self.last_width: Optional[int] = None
        self.last_height: Optional[int] = None
        self.measured_fps: Optional[float] = None
        self.started = False
        self._warmup_buffer: List[Tuple] = []

    def _open_capture(self) -> bool:
        """Open the camera and request settings."""
        self._cap = (
            cv2.VideoCapture(self.device_index)
            if os.name == "nt"
            else cv2.VideoCapture(self.device_index)
        )
        if not self._cap or not self._cap.isOpened():
            print(f"[CameraRecorder] ERROR: Could not open camera {self.device_index}")
            return False

        # Force MJPG format for higher fps
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        # Desired resolution
        if self.frame_size:
            target_w, target_h = self.frame_size
        else:
            target_w, target_h = (1920, 1080)

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_w)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_h)
        self._cap.set(cv2.CAP_PROP_FPS, float(self.requested_fps))

        # Report actual settings
        width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(self._cap.get(cv2.CAP_PROP_FPS) or 0.0)

        print(f"[CameraRecorder] Requested {target_w}x{target_h} @ {self.requested_fps} fps (MJPG forced)")
        print(f"[CameraRecorder] Camera opened (driver reported) at {width}x{height} @ {actual_fps:.2f} fps")

        self.last_width, self.last_height = width, height
        return True

    def _measure_capture_fps(self, max_frames: int) -> float:
        """Warm-up capture to measure real fps and store frames."""
        buf = []
        start = None
        for i in range(max_frames):
            ret, frame = self._cap.read()
            ts = time.time()
            if not ret:
                time.sleep(0.005)
                continue
            if start is None:
                start = ts
            buf.append((frame, ts))
        if len(buf) >= 2:
            measured = len(buf) / (buf[-1][1] - buf[0][1])
        else:
            measured = float(self.requested_fps)
        self._warmup_buffer = buf
        return measured

    def run(self):
        try:
            if not self._open_capture():
                return

            measured_fps = self._measure_capture_fps(self.warmup_frames)
            self.measured_fps = measured_fps
            print(f"[CameraRecorder] Measured capture FPS: {measured_fps:.2f}")
            writer_fps = float(round(measured_fps, 2))
            print(f"[CameraRecorder] Using measured FPS for writer: {writer_fps:.2f}")
            """ # Use measured fps for writer
            driver_fps = float(self._cap.get(cv2.CAP_PROP_FPS) or 0.0)
            if driver_fps >= 10:   # trust driver if sane
                writer_fps = driver_fps
                print(f"[CameraRecorder] Using driver FPS: {writer_fps:.2f}")
            else:                  # fallback to measured
                writer_fps = float(round(measured_fps, 2))
                print(f"[CameraRecorder] Using measured FPS: {writer_fps:.2f}") """
            width = int(self.last_width or 1920)
            height = int(self.last_height or 1080)
            fourcc = cv2.VideoWriter_fourcc(*self.fourcc_str)  # type: ignore[attr-defined]
            writer = cv2.VideoWriter(self.out_path, fourcc, writer_fps, (width, height))  # type: ignore[arg-type]

            if not writer or not writer.isOpened():
                print(f"[CameraRecorder] WARNING: Could not open VideoWriter for {self.out_path}")
                self._writer = None
            else:
                self._writer = writer
                print(f"[CameraRecorder] VideoWriter opened with fps={writer_fps:.2f}")

            self.started = True

            # Write warmup frames
            for (frame, ts) in self._warmup_buffer:
                self._process_frame(frame, ts)
            self._warmup_buffer = []

            # Main loop
            target_dt = 1.0 / writer_fps if writer_fps > 0 else 0
            next_frame_time = time.time()

            while not self._stop_event.is_set():
                now = time.time()
                if now < next_frame_time:
                    time.sleep(next_frame_time - now)
                    continue

                ret, frame = self._cap.read()
                ts = time.time()
                if not ret:
                    time.sleep(0.001)
                    next_frame_time += target_dt
                    continue

                self._process_frame(frame, ts)
                next_frame_time += target_dt

        finally:
            if self._writer is not None:
                self._writer.release()
            if self._cap is not None:
                self._cap.release()
            self.started = False

    def _process_frame(self, frame, ts: float):
        """Overlay timestamp, write to disk, and store timestamp."""
        if self.overlay_timestamp:
            text = f"{ts:.6f}"
            cv2.putText(frame, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 1, cv2.LINE_AA)

        if self._writer is not None:
            self._writer.write(frame)

        with self._timestamps_lock:
            self._timestamps.append((self._frame_counter, ts))
        self._frame_counter += 1

    def stop(self, wait: bool = True):
        self._stop_event.set()
        if wait and self.is_alive():
            self.join(timeout=5.0)

    def get_timestamps(self):
        with self._timestamps_lock:
            return list(self._timestamps)

    def get_last_index_ts(self):
        with self._timestamps_lock:
            if not self._timestamps:
                return (None, None)
            return self._timestamps[-1]

    def get_frame_count(self) -> int:
        return self._frame_counter

    def is_recording(self) -> bool:
        return self.started
