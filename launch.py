# launch.py
import os
import sys

# Add the 'scripts' directory to the Python path
sys.path.append(os.path.abspath('scripts'))

# Now you can import the main_recorder module
from main_recorder import main

if __name__ == "__main__":
    print("--- Launching Radar Recorder ---")
    try:
        main() # Call the main function directly
    except Exception as e:
        print(f"\n[ERROR] An unexpected error occurred: {e}")
    except KeyboardInterrupt:
        print("\n--- Closing... ---")

    print("\n--- Recording session finished ---")
    input("Press Enter to exit.")