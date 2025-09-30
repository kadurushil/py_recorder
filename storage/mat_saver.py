import scipy.io as sio
import numpy as np

class MATSaver:
    def __init__(self, path: str):
        self.path = path
        self._frames = []

    def write_frame(self, frame_dict: dict):
        """Append one frame record to the internal buffer."""
        self._frames.append(frame_dict)

    def close(self):
        """
        Restructure the buffered frames and save to a .mat file.
        """
        if not self._frames:
            print("[MATSaver] No frames to save.")
            return

        # Restructure the list of dicts into a dict of lists (more MATLAB-friendly)
        mat_dict = {key: [] for key in self._frames[0].keys()}
        for frame in self._frames:
            for key, val in frame.items():
                # Ensure all data is NumPy-friendly for saving
                mat_dict[key].append(np.array(val, ndmin=1))

        # Convert lists to object arrays to handle variable lengths
        for key, val_list in mat_dict.items():
            # Create an object array to accommodate potentially ragged arrays
            mat_dict[key] = np.array(val_list, dtype=object)

        print(f"[MATSaver] Saving {len(self._frames)} frames to {self.path}...")
        sio.savemat(self.path, mat_dict)
        print(f"[MATSaver] Saved successfully.")