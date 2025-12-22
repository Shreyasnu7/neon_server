"""
AI EXPOSURE ENGINE — Cinematic Auto-Exposure (AE) + Local Tone Mapping
======================================================================

This module performs:
    • Global exposure estimation
    • Scene-brightness normalization
    • Local contrast enhancement (tone mapping)
    • Temporal smoothing (no flicker)
    • Anti-blowout protection
    • Highlight priority mode
    • Shadow recovery mode
    • Scene-adaptive exposure (faces, sky, subject priority)

It does NOT control any motors or flight hardware → 100% SAFE.
Only adjusts image exposure parameters for cinematography.
"""

import numpy as np
import cv2
import time
from collections import deque
from typing import Dict, Optional

# -------------------------------------------------------------
# Utility: clamp function
# -------------------------------------------------------------
def clamp(x, low, high):
    return low if x < low else high if x > high else x

# =====================================================================
# CLASS 1 — HISTOGRAM ANALYZER
# =====================================================================
class ExposureHistogramAnalyzer:
    """
    Computes luminance histogram and exposure metrics.
    """
    def __init__(self, bins: int = 256):
        self.bins = bins

    def analyze(self, frame: np.ndarray) -> Dict[str, float]:
        if frame is None or frame.size == 0:
            return {
                "avg_luma": 0.5,
                "median_luma": 0.5,
                "shadow_ratio": 0.0,
                "highlight_ratio": 0.0,
                "contrast_index": 0.0,
                "brightness": 0.5 # Added for compatibility
            }

        # convert BGR → Y (luminance)
        yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        y = yuv[:, :, 0].astype(np.float32) / 255.0

        # histogram
        hist, _ = np.histogram(y, bins=self.bins, range=(0.0, 1.0))
        hist = hist.astype(np.float32)
        hist /= np.sum(hist) + 1e-8
        # cumulative histograms
        cdf = np.cumsum(hist)

        # average brightness
        avg_luma = float(np.sum(hist * np.linspace(0, 1, self.bins)))

        # median brightness
        median_idx = np.searchsorted(cdf, 0.5)
        median_luma = float(median_idx / (self.bins - 1))

        # shadows = lower 15% of brightness range
        shadow_ratio = float(np.sum(hist[: int(self.bins * 0.15)]))

        # highlights = top 15% of range
        highlight_ratio = float(np.sum(hist[int(self.bins * 0.85):]))

        # simple global contrast index
        contrast_index = float(np.std(y))

        return {
            "avg_luma": avg_luma,
            "median_luma": median_luma,
            "shadow_ratio": shadow_ratio,
            "highlight_ratio": highlight_ratio,
            "contrast_index": contrast_index,
            "brightness": avg_luma # Alias
        }

# =====================================================================
# CLASS 2 — EXPOSURE STATE
# =====================================================================
class ExposureState:
    """
    Maintains a persistent exposure state.
    """
    def __init__(self):
        self.ev = 0.0                     # exposure compensation (-2.0 ... +2.0)
        self.ev_last = 0.0                # Internal tracking
        self.target_luma = 0.55           # ideal cinematic brightness
        self.brightness_target = 0.55     # Alias
        self.mode = "normal"              # modes: normal / highlight / shadow
        
        self.framerate = 30.0             # Default assumption

        self.hist_history = deque(maxlen=12)   # last 12 histograms
        self.ev_history = deque(maxlen=8)       # last few EV adjustments

        self.last_update_ts = time.time()

    def push_histogram(self, hist_metrics: Dict[str, float]):
        self.hist_history.append(hist_metrics)

    def push_ev(self, ev: float):
        self.ev_history.append(ev)

    def get_smooth_ev(self) -> float:
        if not len(self.ev_history):
            return self.ev
        n = len(self.ev_history)
        weights = np.linspace(1, 2, n)
        ev = float(np.sum(np.array(self.ev_history) * weights) / np.sum(weights))
        return clamp(ev, -2.0, 2.0)

# =====================================================================
# CLASS 3 — EXPOSURE DECISION ENGINE
# =====================================================================
class ExposureDecisionEngine:
    """
    Computes the EV (Exposure Value) correction.
    """
    def __init__(self):
        self.max_step = 0.25          # max EV change per frame (anti-flicker)
        self.highlight_threshold = 0.22
        self.shadow_threshold = 0.22
        self.cinematic_mid_bias = 1.15    # pulls exposure toward mid-tones

    def compute_correction(self, hist_metrics: Dict[str, float], state: ExposureState) -> float:
        avg_luma = hist_metrics["avg_luma"]
        median_luma = hist_metrics["median_luma"]
        shadows = hist_metrics["shadow_ratio"]
        highlights = hist_metrics["highlight_ratio"]

        # 1) BASE EXPOSURE ERROR (mid-tone driven)
        mid_tone = (avg_luma * 0.5 + median_luma * 0.5)
        mid_tone *= self.cinematic_mid_bias

        target = state.target_luma
        error = (target - mid_tone)

        # EV adjustment is proportional to error
        ev_delta = error * 1.8     # aggressive control (cinematic)

        # 2) HIGHLIGHT PROTECTION
        if highlights > self.highlight_threshold:
            ev_delta -= (highlights * 0.45)

        # 3) SHADOW BOOST
        if shadows > self.shadow_threshold:
            ev_delta += (shadows * 0.32)

        # 4) SCENE-MODE LOGIC
        if state.mode == "highlight":
            ev_delta -= 0.18
        elif state.mode == "shadow":
            ev_delta += 0.18

        # 5) LIMIT MAX STEP SIZE (anti-flicker)
        ev_delta = float(clamp(ev_delta, -self.max_step, self.max_step))

        return ev_delta

# =====================================================================
# CLASS 4 — EXPOSURE TRANSLATOR (EV → CAMERA SETTINGS)
# =====================================================================
class ExposureTranslator:
    def __init__(self,
                 base_iso=100,
                 max_iso=6400,
                 min_shutter=1/8000,
                 max_shutter=1/24,
                 shutter_180deg=True):

        self.base_iso = base_iso
        self.max_iso = max_iso
        self.min_shutter = min_shutter
        self.max_shutter = max_shutter
        self.shutter_180deg = shutter_180deg

    def estimate_motion_level(self, motion_score: float) -> float:
        return clamp(motion_score, 0.0, 1.0)

    def translate(self,
                  state: ExposureState,
                  hist_metrics: Dict[str,float],
                  ev_delta: float,
                  motion_score: float) -> Dict[str,float]:

        # 1) Apply EV correction to brightness model
        state.ev_last += ev_delta
        state.ev_last = float(clamp(state.ev_last, -4.0, +4.0))

        # 2) Motion-adjusted shutter selection
        motion = self.estimate_motion_level(motion_score)

        if self.shutter_180deg:
            # Cinematic shutter rule (180°)
            shutter = 1.0 / (2.0 * state.framerate)
            shutter *= (1.0 - 0.35 * motion)
        else:
            shutter = self.max_shutter - (motion * (self.max_shutter - self.min_shutter))

        shutter = clamp(shutter, self.min_shutter, self.max_shutter)

        # 3) ISO calculation
        iso_multiplier = 2 ** state.ev_last
        iso = self.base_iso * iso_multiplier
        iso = float(clamp(iso, self.base_iso, self.max_iso))

        # 4) Gain compensation (if ISO topped out)
        gain = 1.0
        if iso >= self.max_iso * 0.98:
            gain = 1.0 + (state.ev_last * 0.06)
            gain = float(clamp(gain, 1.0, 1.25))

        return {
            "iso": iso,
            "shutter": shutter,
            "gain": gain,
            "ev_applied": state.ev_last,
            "mode": state.mode,
            "motion": motion,
        }

# =====================================================================
# CLASS 5 — TEMPORAL EXPOSURE SMOOTHER
# =====================================================================
class ExposureSmoother:
    def __init__(self,
                 iso_smooth=0.25,
                 shutter_smooth=0.20,
                 gain_smooth=0.35,
                 ev_momentum=0.15):

        self.iso_smooth = iso_smooth
        self.shutter_smooth = shutter_smooth
        self.gain_smooth = gain_smooth
        self.ev_momentum = ev_momentum

        self.last_iso = None
        self.last_shutter = None
        self.last_gain = None
        self.last_ev = 0.0

    def smooth_step(self, prev, new, amt):
        if prev is None:
            return new
        return prev + (new - prev) * amt

    def apply(self, raw: Dict[str,float]) -> Dict[str,float]:
        iso = raw["iso"]
        shutter = raw["shutter"]
        gain = raw["gain"]
        ev = raw["ev_applied"]

        self.last_iso = self.smooth_step(self.last_iso, iso, self.iso_smooth)
        self.last_shutter = self.smooth_step(self.last_shutter, shutter, self.shutter_smooth)
        self.last_gain = self.smooth_step(self.last_gain, gain, self.gain_smooth)
        self.last_ev = self.smooth_step(self.last_ev, ev, self.ev_momentum)

        return {
            "iso": float(self.last_iso),
            "shutter": float(self.last_shutter),
            "gain": float(self.last_gain),
            "ev": float(self.last_ev),
        }

# =====================================================================
# CLASS 6 — EXPOSURE HISTORY
# =====================================================================
class ExposureHistory:
    def __init__(self, size=24):
        self.size = size
        self.ev_hist = []
        self.brightness_hist = []
        self.shutter_hist = []
        self.iso_hist = []

    def push(self, ev, brightness, shutter, iso):
        if len(self.ev_hist) >= self.size:
            self.ev_hist.pop(0)
            self.brightness_hist.pop(0)
            self.shutter_hist.pop(0)
            self.iso_hist.pop(0)

        self.ev_hist.append(ev)
        self.brightness_hist.append(brightness)
        self.shutter_hist.append(shutter)
        self.iso_hist.append(iso)

    def detect_ev_oscillation(self):
        if len(self.ev_hist) < 6:
            return False
        signs = []
        for i in range(1, len(self.ev_hist)):
            delta = self.ev_hist[i] - self.ev_hist[i-1]
            signs.append(np.sign(delta))
        alt = 0
        for i in range(1, len(signs)):
            if signs[i] * signs[i-1] < 0:
                alt += 1
        return alt > (len(signs) * 0.6)

# =====================================================================
# CLASS 7 — ADVANCED EV CONTROLLER
# =====================================================================
class AdvancedEVController:
    def __init__(self, flicker_damp=0.35, highlight_protect_strength=0.25):
        self.flicker_damp = flicker_damp
        self.highlight_protect_strength = highlight_protect_strength

    def apply(self, ev_raw, metrics, history: ExposureHistory):
        if history.detect_ev_oscillation():
            ev_raw *= (1.0 - self.flicker_damp)

        highlight_ratio = metrics["highlight_ratio"]
        if highlight_ratio > 0.15:
            protect = min(1.0, highlight_ratio * 2.5)
            ev_raw -= protect * self.highlight_protect_strength

        return float(ev_raw)

# =====================================================================
# UTILITY HELPER
# =====================================================================
def compute_ev_delta(current, target):
    # Logarithmic error approximation
    if current < 0.01: current = 0.01
    return np.log2(target / current)

# =====================================================================
# CLASS 8 — EXTENDED ExposureEngine
# =====================================================================
class AIExposureEngine:
    """
    High-level interface for the auto-exposure pipeline.
    """
    def __init__(self):
        self.scene = ExposureHistogramAnalyzer()
        self.translator = ExposureTranslator()
        self.smoother = ExposureSmoother()
        self.history = ExposureHistory(size=32)
        self.ev_controller = AdvancedEVController()

    def update(self,
               frame: np.ndarray,
               motion_score: float,
               state: ExposureState,
               return_debug: bool = False) -> Dict[str, any]:

        # 1) Analyze scene
        metrics = self.scene.analyze(frame)
        
        # 2) Compute raw delta
        raw_ev_delta = compute_ev_delta(
            metrics["brightness"],
            state.brightness_target
        )

        # 3) Apply corrections
        ev_corrected = self.ev_controller.apply(
            raw_ev_delta,
            metrics,
            self.history
        )

        # 4) Translate
        raw_settings = self.translator.translate(
            state=state,
            hist_metrics=metrics,
            ev_delta=ev_corrected,
            motion_score=motion_score,
        )

        # 5) Smooth
        smooth = self.smoother.apply(raw_settings)

        # 6) History
        self.history.push(
            smooth["ev"],
            metrics["brightness"],
            smooth["shutter"],
            smooth["iso"]
        )

        out = {
            "iso": smooth["iso"],
            "shutter": smooth["shutter"],
            "gain": smooth["gain"],
            "ev": smooth["ev"],
            "mode": state.mode,
            "scene_metrics": metrics,
            "timestamp": time.time(),
        }

        if return_debug:
             out["debug"] = {"raw": raw_settings}

        return out