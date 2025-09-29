# storage/jsonl_saver.py
import json
import numpy as np

class NumpyEncoder(json.JSONEncoder):
    """Custom JSON Encoder for NumPy data types"""
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, (np.floating,)):
            return float(obj)
        return super().default(obj)

class JSONLSaver:
    def __init__(self, path: str):
        self.f = open(path, 'w', buffering=1)  # line-buffered

    def write_frame(self, frame_dict: dict):
        """Write one frame record as a JSON line"""
        self.f.write(json.dumps(frame_dict, cls=NumpyEncoder) + '\n')

    def close(self):
        self.f.close()
