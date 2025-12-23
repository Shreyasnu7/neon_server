
import numpy as np

class ACESInputTransform:
    """
    Handles Input Device Transform (IDT) for ACES pipeline.
    Connects raw camera space -> AP0/AP1 ACES color space.
    """
    def __init__(self):
        pass

    def process(self, frame: np.ndarray, profile="sRGB", ei=800) -> np.ndarray:
        """
        Passthrough placeholder for missing IDT logic.
        Real implementation would use color matrix multiplication.
        """
        # For now, just normalize and return float32
        if frame is None:
            return None
        return frame.astype(np.float32) / 255.0
