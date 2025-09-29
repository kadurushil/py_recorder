# radar/board_cli.py
import time
import serial


def open_cli(port_name: str, baud: int = 115200, timeout: float = 1.0):
    """
    Open the CLI serial port.
    """
    return serial.Serial(port_name, baud, timeout=timeout)


def send_cfg(cli_ser: serial.Serial, cfg_path: str) -> int:
    """
    Send configuration commands from .cfg file to radar CLI port.
    Returns the final data baud rate.
    """
    data_port_baud_rate = 921600  # default

    with open(cfg_path, "r") as f:
        cfg_lines = f.readlines()

    cli_ser.reset_input_buffer()
    print(f"\nSending configuration from {cfg_path}...")

    for line in cfg_lines:
        if line.strip().startswith("%") or not line.strip():
            continue  # skip comments and empty lines

        cmd = line.strip()
        print(f"  Sending: {cmd}")
        cli_ser.write((cmd + "\n").encode())
        time.sleep(0.05)

        # Handle baudRate change
        if "baudRate" in cmd:
            try:
                new_baud = int(cmd.split()[1])
                data_port_baud_rate = new_baud
                cli_ser.baudrate = new_baud
                print(f"  -> Changed baud rate to {new_baud}")
                time.sleep(0.1)
            except Exception as e:
                print("  [WARN] Could not parse baudRate:", e)

    print("Configuration finished.\n")
    cli_ser.reset_input_buffer()
    return data_port_baud_rate
