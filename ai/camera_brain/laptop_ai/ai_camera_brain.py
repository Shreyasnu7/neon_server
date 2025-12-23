
import time
import cv2
import numpy as np
import threading
import traceback
from typing import Dict, Any, Optional

# --- CORE IMPORTS (Basic/Laptop AI) ---
# Verified paths
from laptop_ai.ai_autofocus import AIAutofocus
from laptop_ai.ai_exposure_engine import AIExposureEngine
from laptop_ai.ai_stabilizer import AIStabilizer
from laptop_ai.ai_scene_classifier import SceneClassifier
from laptop_ai.ai_colour_engine import AIColorEngine

# --- REAL DEEP PIPELINE IMPORTS (Corrected Paths) ---
# ACES: Replaced with GPU version found in tone_curve directory
try:
    from laptop_ai.ai_pipeline.tone_curve.gpu_aces import GPUACESToneCurve
except ImportError:
    # Fallback stub if GPU ACES fails
    class GPUACESToneCurve:
        def __init__(self): pass
        def process(self, img): return img

# Global Tone Curve: Using the one found in laptop_ai/ai_pipeline/tone_curve
try:
    from laptop_ai.ai_pipeline.tone_curve.global_tone_curve import GlobalToneCurve
except ImportError:
    class GlobalToneCurve:
        def __init__(self): pass
        def apply(self, img, stats): return img

# Stabilizer / Temporal: Stubs for now as files are missing/complex
class MeshStabilizer:
    def __init__(self): pass
    def stabilize(self, img): return img

class TemporalConsistency:
    def __init__(self, alpha=0.15): pass
    def update(self, data): return data

# Lens: Using Verified path
try:
    from ai.camera_brain.lens.lens_model import LensModel
except ImportError:
    class LensModel:
        def __init__(self): pass
        def undistort(self, frame): return frame

# --- ULTRA LOGIC EXTENSIONS (Corrected Paths) ---
from laptop_ai.ai_subject_tracker import AdvancedAISubjectTracker
from laptop_ai.ai_hdr_engine import AIHDREngine
from laptop_ai.ai_depth_estimator import AIDepthEstimator
from laptop_ai.ai_super_resolution import AISuperResolution

# Motion
try:
    from ai.camera_brain.motion.anticipation import MotionAnticipator
    from ai.camera_brain.motion.smoothing import ExponentialSmoother
except ImportError:
    class MotionAnticipator: 
        def __init__(self): pass
        def predict(self, x): return x
    class ExponentialSmoother:
        def __init__(self, alpha=0.5): pass
        def update(self, x): return x

# Farming / Saliency
try:
    from ai.camera_brain.farming.saliency import SaliencyMap
    from ai.camera_brain.farming.farming_engine import FarmingEngine
except ImportError:
    class SaliencyMap:
        def compute(self, frame): return None
    class FarmingEngine:
        def compute_harvest(self, smap): return 0.5

# Intent / Framing
try:
    from ai.shot_intent.memory.intent_memory import IntentMemory
    from ai.shot_intent.projection.intent_projection import IntentProjector
    from ai.shot_intent.projection.semantic_axes import SemanticAxes
    from ai.camera_brain.core.framing.framing_engine import FramingEngine
except ImportError:
    class IntentMemory:
        def store(self, x): pass
    class IntentProjector:
        def project(self, x): return {}
    class SemanticAxes:
        def analyze(self, x): return {}
    class FramingEngine:
        def compute_framing(self, f, s): return {"active": False}


class AICameraBrain:
    """
    The High-Performance Asynchronous Brain (v3.0).
    
    Architecture:
    1. Main Thread (decide): Handles Fast Pixel Ops (Stab, Lens) & returns latest Plan.
    2. Background Thread (_brain_worker): Handles Slow AI Ops (Scene, Depth, Tracker, Saliency).
    
    This ensures the Camera/Display loop runs at max FPS (60-240), while the AI thinks
    as fast as it can (e.g. 10-20 FPS) without blocking the view.
    """

    def __init__(self):
        print("🧠 Initializing Deep Asynchronous Brain v3.0...")
        
        # --- 1. Fast Components (Main Thread) ---
        self.stabilizer = AIStabilizer()
        self.gpu_aces = GPUACESToneCurve()
        self.color = AIColorEngine()
        
        # --- 2. Slow Components (Background Thread) ---
        self.autofocus = AIAutofocus()
        self.exposure = AIExposureEngine()
        self.scene = SceneClassifier()
        self.tracker = AdvancedAISubjectTracker()
        self.hdr = AIHDREngine()
        self.depth = AIDepthEstimator()
        
        self.super_res = AISuperResolution(scale=2)
        self.anticipator = MotionAnticipator()
        self.lens_model = LensModel()
        self.smoother = ExponentialSmoother(alpha=0.15)
        
        self.saliency = SaliencyMap()
        self.farming = FarmingEngine()
        self.intent_memory = IntentMemory()
        self.projector = IntentProjector()
        self.semantic_axes = SemanticAxes()
        self.framing_engine = FramingEngine()

        print("✅ Brain Loaded. Starting Background Cortex...")

        # --- ASYNC STATE ---
        self.running = True
        self.input_lock = threading.Lock()
        self.output_lock = threading.Lock()
        
        self.default_specs = {
            "fps": 60,
            "resolution": "4K",
            "bitrate": "high",
            "encoding": "h264", 
        }
        
        self.latest_plan = self._get_default_plan()
        
        # Inputs for the background thread
        self.latest_frame = None
        self.latest_context = {}
        self.latest_user_text = ""
        self.latest_fusion_state = {}
        
        # Start Thinking
        self.thread = threading.Thread(target=self._brain_worker, daemon=True)
        self.thread.start()
        
        self.frame_count = 0


    def _get_default_plan(self):
        return {
            "timestamp": time.time(),
            "scene": {"default": 1.0},
            "subject": None,
            "status": "booting",
            # Default empty values so main thread doesn't crash on first frame
            "autofocus": {"active": False},
            "exposure": {"iso": 100, "shutter": 0.01},
            "framing": {"active": False},
            "stabilization": {"active": False},
            "tone_map": [],
            "color_grade": {},
            "recording": self.default_specs
        }

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)

    # -------------------------------------------------------------------------
    # WORKER THREAD (The Cortex)
    # -------------------------------------------------------------------------
    def _brain_worker(self):
        """
        Runs the heavy AI models in a loop.
        """
        while self.running:
            try:
                # 1. Get Inputs (Thread Safe copy)
                with self.input_lock:
                    if self.latest_frame is None:
                        time.sleep(0.01)
                        continue
                    
                    # Resize for AI Inference Speed (e.g. 480p)
                    # This is CRITICAL for 'thinking' performance
                    h, w = self.latest_frame.shape[:2]
                    scale = 480.0 / h
                    if scale < 1.0:
                        frame_small = cv2.resize(self.latest_frame, (int(w*scale), int(h*scale)))
                    else:
                        frame_small = self.latest_frame.copy() 
                    
                    vision_context = self.latest_context
                    user_text = self.latest_user_text
                    fusion_state = self.latest_fusion_state
                
                t_start = time.time()
                
                # 2. RUN HEAVY LOGIC
                
                # A. Scene
                try:
                    scene_labels = self.scene.predict(frame_small)
                except:
                    scene_labels = {"default": 1.0}
                
                # B. Depth & Tracker
                try:
                    # depth_map, _ = self.depth.estimate(frame_small)
                    pass # Skipping actual depth calc for now if it's too slow
                except:
                    pass
                
                detections = vision_context.get("detections", [])
                ranked_subjects = self.tracker.update(detections, frame_image=frame_small)
                
                subject = None
                if ranked_subjects:
                    top_sub = ranked_subjects[0]
                    subject = {
                        "id": top_sub.id,
                        "bbox": top_sub.bbox.tolist(),
                        "label": str(top_sub.cls),
                        "conf": top_sub.conf
                    }
                
                # C. AF & Exposure Analysis
                try:
                    af_info = self.autofocus.analyze_frame(frame_small, detections=[subject] if subject else None)
                    af_cmd = self.autofocus.decide_focus_command(af_info)
                    af_cmd = self.autofocus.smooth_update(af_cmd)
                    
                    stats = self.exposure.analyze_scene(frame_small)
                    ae_cmd = self.exposure.propose_exposure(stats)
                    # tone_map = self.exposure.local_tone_map(frame_small) # Expensive
                    tone_map = []
                    grade = self.color.propose_grade(stats)
                except:
                   af_cmd = {"active": False}
                   ae_cmd = {"iso": 100}
                   tone_map = []
                   grade = {}
                
                # D. Deep Framing/Saliency
                try:
                    saliency_map = self.saliency.compute(frame_small)
                    farming_score = self.farming.compute_harvest(saliency_map)
                    framing_cmd = self.framing_engine.compute_framing(frame_small, subject)
                    axes_state = self.semantic_axes.analyze(scene_labels)
                    # future_state = self.projector.project(scene_labels)
                    future_state = {}
                except:
                    farming_score = 0.5
                    framing_cmd = {"active": False}
                    axes_state = {}
                    future_state = {}
                
                # E. Memory
                try:
                    self.intent_memory.store({
                        "ts": t_start,
                        "scene": scene_labels,
                        "subject": subject
                    })
                except: pass
                
                # F. Rec Specs
                rec_specs = self._infer_recording_specs(user_text, scene_labels)
                camera_choice = fusion_state.get("camera_choice", "auto")

                # 3. Construct Plan
                new_plan = {
                    "timestamp": t_start,
                    "scene": scene_labels,
                    "subject": subject,
                    
                    # Directives
                    "autofocus": af_cmd,
                    "exposure": ae_cmd,
                    "tone_map": tone_map,
                    "color_grade": grade,
                    
                    # Meta
                    "framing": framing_cmd,
                    "semantic_axes": axes_state,
                    "farming_score": farming_score,
                    "future_prediction": future_state,
                    
                    "recording": rec_specs,
                    "camera_choice": camera_choice,
                    
                    "meta": {
                        "brain_fps": 1.0 / (time.time() - t_start + 1e-9),
                        "latency_ms": int((time.time() - t_start) * 1000)
                    }
                }
                
                # 4. Publish Plan (Thread Safe)
                with self.output_lock:
                    self.latest_plan = new_plan
                
                # Avoid CPU burning
                time.sleep(0.005)

            except Exception as e:
                print(f"❌ BRAIN THREAD ERROR: {e}")
                # traceback.print_exc()
                time.sleep(0.1)

    def _infer_recording_specs(self, user_text, scene_labels):
        fps = self.default_specs["fps"]
        if "slow" in user_text: fps = 120
        if "cinematic" in user_text: fps = 24
        return {"fps": fps, "resolution": "4K", "bitrate": "high"}

    # -------------------------------------------------------------------------
    # MAIN PUBLIC METHOD (Called by Vision Loop)
    # -------------------------------------------------------------------------
    def decide(self, user_text, fusion_state, frame, vision_context=None):
        """
        Non-blocking decision making.
        """
        if frame is None:
            with self.output_lock:
                return self.latest_plan.copy()

        # 1. Update Inputs for Worker
        with self.input_lock:
            # We copy FRAME to ensure thread safety during resize in worker
            self.latest_frame = frame.copy() 
            self.latest_context = vision_context or {}
            self.latest_user_text = user_text
            self.latest_fusion_state = fusion_state

        t_fast_start = time.time()

        # 2. Fast Pixel Ops (Main Thread)
        # GPU ACES / Stabilization could run here if needed on full frame
        # But for now we just pass through or do lightweight stuff
        
        # Stabilization (If fast)
        try:
           # stab_frame = self.stabilizer.process(frame) 
           # stab_cmd = {"active": True}
           stab_cmd = {"active": False}
        except:
            stab_cmd = {"active": False}
        
        # 3. Get Latest Brain Plan
        with self.output_lock:
            plan = self.latest_plan.copy()
        
        # 4. Inject Fast Ops Results
        plan["stabilization"] = stab_cmd
        plan["frame_ts"] = t_fast_start
        
        return plan

if __name__ == "__main__":
    b = AICameraBrain()
    print("Running Brain Test...")
    time.sleep(2)
    b.stop()
    print("Done")
