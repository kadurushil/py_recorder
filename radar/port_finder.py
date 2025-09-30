# radar/port_finder.py
import serial.tools.list_ports

# Description strings for TI radar boards
CLICOM_SDK5 = "XDS110 Class Application/User UART"
DATACOM_SDK5 = "XDS110 Class Auxiliary Data Port"

def find_cli_port():
    """
    Scans COM ports and returns the device path for the TI Radar's CLI port.
    """
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if CLICOM_SDK5 in (p.description or ""):
            print(f"Found CLI Port: {p.device}")
            return p.device
    return None

def find_data_port():
    """
    Scans COM ports and returns the device path for the TI Radar's Data port.
    (Note: Not used in the current single-port configuration but good to have)
    """
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if DATACOM_SDK5 in (p.description or ""):
            print(f"Found Data Port: {p.device}")
            return p.device
    return None