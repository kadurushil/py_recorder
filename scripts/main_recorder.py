# scripts/main_recorder.py
"""
Main orchestrator: start camera (optional), configure board, read frames, parse, save JSONL.
"""

import signal
import serial
import sys
import time
import os
import datetime

from radar.board_cli import open_cli, send_cfg
from radar.serial_reader import read_one_frame
from radar.parse_wrapper import parse_frame
from storage.jsonl_saver import JSONLSaver
from storage.mat_saver import MATSaver # Import the new MATSaver

# Camera & sync utilities
# camera package must exist: camera/__init__.py
CAMERA_ENABLED = True           # set False to disable video recording (useful on low-power devices)
CAMERA_DEVICE_INDEX = 0
CAMERA_OUT_PATH = "cam.mp4"
CAMERA_FPS = 30
CAMERA_LOW_POWER = False        # set True for Raspberry Pi or low-power mode (less overlay, lower res)
CAMERA_BUFFER_LEN = 2000        # how many timestamps to retain for lookup

if CAMERA_ENABLED:
    from camera.camera_recorder import CameraRecorder
    from utils.sync_utils import find_closest_index

running = True
def handle_sigint(sig, frame):
    global running
    print("\n[INFO] Stopping...")
    running = False
signal.signal(signal.SIGINT, handle_sigint)

def safe_to_list(val):
    """Convert numpy arrays and lists to plain lists for JSON. Accept None."""
    import numpy as np
    if val is None:
        return []
    if isinstance(val, np.ndarray):
        return val.tolist()
    if isinstance(val, (list, tuple)):
        return list(val)
    return [val]

def main():
    cli_port = "COM5"     # TODO: update to your CLI port
    cfg_file = "cfg/MotionDetect.cfg"  # TODO: path to your config
    log_dir = "LOGS" # define the main LOG directory
    # --- Create a unique, timestamped sub-directory for this run ---
    timestamp_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    session_log_dir = os.path.join(log_dir, timestamp_str)
    os.makedirs(session_log_dir, exist_ok=True)
    # --- Update output file paths to use the new session directory ---
    camera_out_path = os.path.join(session_log_dir, "cam.mp4")
    out_jsonl = os.path.join(session_log_dir, "radar_log.jsonl")
    out_mat = os.path.join(session_log_dir, "radar_log.mat")
    # Start camera (optional)
    camera = None
    if CAMERA_ENABLED:
        print("[Main] Starting camera recorder...")
        camera = CameraRecorder(
            device_index=CAMERA_DEVICE_INDEX,
            out_path=CAMERA_OUT_PATH,
            requested_fps=CAMERA_FPS,
            buffer_len=CAMERA_BUFFER_LEN,
            overlay_timestamp=True,
            low_power_mode=CAMERA_LOW_POWER
        )
        camera.start()

        # Wait for the camera to confirm it's recording
        start_time = time.time()
        while not camera.is_recording():
            if time.time() - start_time > 20:  # 20-second timeout
                print("[Main] FATAL: Camera failed to start within the timeout period.")
                camera.stop()
                camera = None
                break
            time.sleep(0.1)

        if camera:
            print("[Main] Camera started successfully.")


    # Open CLI port, send config, and prepare serial port for data read
    ser = open_cli(cli_port)
    final_baud = send_cfg(ser, cfg_file)
    ser.baudrate = final_baud
    ser.reset_input_buffer()

    jsonl_saver = JSONLSaver(out_jsonl)
    mat_saver = MATSaver(out_mat) # Instantiate the MATSaver

    print(f"Listening on {ser.port} at {final_baud} baud...")
    frame_count = 0

    # main loop
    try:
        while running:
            frame_bytes = read_one_frame(ser)
            if frame_bytes is None:
                print("[WARN] Could not read frame, resyncing...")
                continue

            parsed = parse_frame(frame_bytes)
            if parsed.get("error", 1) != 0:
                print("[WARN] Parser error")
                continue

            # build slim record
            record = {
                "frameNum": int(parsed.get("frameNum", -1)),
                "timestamp": float(parsed.get("timestamp", 0)),
                "numPoints": int(parsed.get("numDetectedPoints", 0)),
                "pointCloud": safe_to_list(parsed.get("pointCloud")),
                "rangeProfile": safe_to_list(parsed.get("rangeProfile"))
            }

            # if camera enabled, find closest video frame timestamp
            if CAMERA_ENABLED and camera is not None and camera.is_recording():
                ts_pairs = camera.get_timestamps()  # list of (frame_index, ts)
                vf_index, vf_ts, td = find_closest_index(ts_pairs, record["timestamp"])
                # Add matching info if found and within reasonable tolerance
                if vf_index is not None:
                    record["video_frame_index"] = int(vf_index)
                    record["video_frame_ts"] = float(vf_ts) # type: ignore
                    record["video_time_delta"] = float(td)  # type: ignore # positive means camera ts is after radar ts
                else:
                    record["video_frame_index"] = None
                    record["video_frame_ts"] = None
                    record["video_time_delta"] = None

            jsonl_saver.write_frame(record) # write to JSON buffer
            mat_saver.write_frame(record) # Write to MAT buffer
            frame_count += 1
            print(f"[Frame {frame_count}] NumPoints={record['numPoints']}")

    except KeyboardInterrupt:
        print("KeyboardInterrupt received, shutting down...")

    # cleanup
    print("[Main] Stopping components...")
    jsonl_saver.close()
    mat_saver.close() # Save the .mat file
    if camera is not None:
        camera.stop(wait=True)
    if ser.is_open:
        ser.close()
    print("Recorder stopped cleanly.")

if __name__ == "__main__":
    main()