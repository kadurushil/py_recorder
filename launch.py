# launch.py
import subprocess
import sys
import os

# This ensures the script looks for modules in the correct project structure
script_path = os.path.join('scripts', 'main_recorder.py')

print("--- Launching Radar Recorder ---")
try:
    # We use sys.executable to ensure we run the script with the same Python
    # that was used to launch this file.
    subprocess.run([sys.executable, script_path], check=True)
except FileNotFoundError:
    print(f"\n[ERROR] Could not find the main script at: {script_path}")
    print("Please make sure you are running this from the root of the 'py_recorder' directory.")
except subprocess.CalledProcessError as e:
    print(f"\n[ERROR] The recorder script exited with an error (code {e.returncode}).")
except KeyboardInterrupt:
    print("\n--- Closing... ---")

print("\n--- Recording session finished ---")
input("Press Enter to exit.")