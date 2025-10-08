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
import struct

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from radar.board_cli import open_cli, send_cfg
from radar.serial_reader import read_one_frame
from radar.parse_wrapper import parse_frame
from storage.jsonl_saver import JSONLSaver
from storage.mat_saver import MATSaver
from radar.port_finder import find_cli_port # Correctly imported

# Camera & sync utilities
CAMERA_ENABLED = True
CAMERA_DEVICE_INDEX = 0
CAMERA_FPS = 30
CAMERA_LOW_POWER = False
CAMERA_BUFFER_LEN = 2000

if CAMERA_ENABLED:
    from camera.camera_recorder import CameraRecorder
    from utils.sync_utils import find_closest_index

running = True

def handle_sigint(sig, frame):
    global running
    print("\n[INFO] Stopping...")
    running = False

signal.signal(signal.SIGINT, handle_sigint)

def choose_cfg_file():
    """Lists .cfg files in the 'cfg' directory and prompts the user to choose one."""
    cfg_dir = "cfg"
    if not os.path.isdir(cfg_dir):
        print(f"[ERROR] '{cfg_dir}' directory not found.")
        return None

    cfg_files = [f for f in os.listdir(cfg_dir) if f.endswith(".cfg")]

    if not cfg_files:
        print(f"[ERROR] No .cfg files found in '{cfg_dir}' directory.")
        return None

    print("\nPlease choose a configuration file:")
    for i, f in enumerate(cfg_files):
        print(f"  [{i+1}] {f}")

    while True:
        try:
            choice = int(input(f"Enter number (1-{len(cfg_files)}): ")) - 1
            if 0 <= choice < len(cfg_files):
                return os.path.join(cfg_dir, cfg_files[choice])
            else:
                print("Invalid number, please try again.")
        except ValueError:
            print("Invalid input, please enter a number.")

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
    # --- Change 1: Automatically find the CLI port ---
    cli_port = find_cli_port()
    if not cli_port:
        sys.exit("Exiting: Could not find radar's CLI port. Is it connected?")

    # --- Change 2: Use the function to choose the config file ---
    cfg_file = choose_cfg_file()
    if not cfg_file:
        sys.exit("Exiting: No configuration file selected.")

    log_dir = "LOGS"
    timestamp_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    session_log_dir = os.path.join(log_dir, timestamp_str)
    os.makedirs(session_log_dir, exist_ok=True)
    camera_out_path = os.path.join(session_log_dir, "cam.mp4")
    out_jsonl = os.path.join(session_log_dir, "radar_log.jsonl")
    out_mat = os.path.join(session_log_dir, "radar_log.mat")
    camera = None
    if CAMERA_ENABLED:
        print("[Main] Starting camera recorder...")
        camera = CameraRecorder(
            device_index=CAMERA_DEVICE_INDEX,
            out_path=camera_out_path,
            requested_fps=CAMERA_FPS,
            buffer_len=CAMERA_BUFFER_LEN,
            overlay_timestamp=True,
            low_power_mode=CAMERA_LOW_POWER,
        )
        camera.start()

        # Wait for the camera to confirm it's recording
        start_time = time.time()
        while not camera.is_recording():
            if time.time() - start_time > 20:
                print("[Main] FATAL: Camera failed to start within the timeout period.")
                camera.stop()
                camera = None
                break
            time.sleep(0.1)

        if camera:
            print("[Main] Camera started successfully.")

    ser = open_cli(cli_port)
    final_baud = send_cfg(ser, cfg_file)
    ser.baudrate = final_baud
    ser.reset_input_buffer()

    jsonl_saver = JSONLSaver(out_jsonl)
    mat_saver = MATSaver(out_mat)

    print(f"Listening on {ser.port} at {final_baud} baud...")
    frame_count = 0

    # main loop
    try:
        while running:
            frame_bytes = read_one_frame(ser)
            if frame_bytes is None:
                print("[WARN] Could not read frame, resyncing...")
                continue

            # =================================================================
            # ADD THIS CODE FOR THE TEST
            # This will print the raw hex data for every frame received.
            print("\n================ PYTHON RAW FRAME ================", flush=True)
            try:
                # We do a quick unpack here just to get the header info for the log
                _, _, totalPacketLen, _, frameNum, _, _, numTLVs, _ = struct.unpack('<Q8I', frame_bytes[:40])
                print(f"Frame: {frameNum}, Total Length: {totalPacketLen}, TLVs: {numTLVs}", flush=True)
            except Exception as e:
                print(f"Could not parse header for logging: {e}", flush=True)

            hex_string = ' '.join(f'{b:02X}' for b in frame_bytes)
            print(f"Raw Hex ({len(frame_bytes)} bytes): {hex_string}", flush=True)
            print("====================================================\n", flush=True)
            # END OF TEST CODE
            # =================================================================
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
                "rangeProfile": safe_to_list(parsed.get("rangeProfile")),
            }

            # if camera enabled, find closest video frame timestamp
            if CAMERA_ENABLED and camera is not None and camera.is_recording():
                ts_pairs = camera.get_timestamps()  # list of (frame_index, ts)
                vf_index, vf_ts, td = find_closest_index(ts_pairs, record["timestamp"])
                # Add matching info if found and within reasonable tolerance
                if vf_index is not None:
                    record["video_frame_index"] = int(vf_index)
                    record["video_frame_ts"] = float(vf_ts)  # type: ignore
                    record["video_time_delta"] = float(td)  # type: ignore # positive means camera ts is after radar ts
                else:
                    record["video_frame_index"] = None
                    record["video_frame_ts"] = None
                    record["video_time_delta"] = None

            jsonl_saver.write_frame(record)  # write to JSON buffer
            mat_saver.write_frame(record)  # Write to MAT buffer
            frame_count += 1
            print(f"[Frame {frame_count}] NumPoints={record['numPoints']}")

    except KeyboardInterrupt:
        print("KeyboardInterrupt received, shutting down...")

    print("[Main] Stopping components...")
    jsonl_saver.close()
    mat_saver.close()
    if camera is not None:
        camera.stop(wait=True)
    if ser.is_open:
        ser.close()
    print("Recorder stopped cleanly.")

if __name__ == "__main__":
    main()