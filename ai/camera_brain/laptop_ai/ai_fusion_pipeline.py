# File: laptop_ai/ai_fusion_pipeline.py
from .ai_hdr_engine import AIHDREngine
from .ai_noise_reduction import AINoiseReduction
from .ai_super_resolution import AISuperResolution
from .ai_depth_estimator import AIDepthEstimator

class AIFusionPipeline:

    def __init__(self):
        self.hdr = AIHDREngine()
        self.nr = AINoiseReduction()
        self.sr = AISuperResolution(scale=2)
        self.depth = AIDepthEstimator()

    def process(self, frame):
        frame = self.nr.denoise(frame)
        depth, mask = self.depth.estimate(frame)
        frame = self.sr.upscale(frame)
        return frame, depth, mask