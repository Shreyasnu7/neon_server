
# File: laptop_ai/director_core.py
"""
Main orchestrator for the Laptop AI Director.

Responsibilities:
- Listen for new AI jobs from the VPS (via MessagingClient).
- Capture context frame(s) from the drone RTSP / local camera.
- Run VisionTracker to produce vision_context (smoothed tracks).
- Send multimodal request to the cloud prompter (text + optional images/video link).
- Convert cloud response into a validated cinematic primitive via cinematic_planner.
- Use UltraDirector for curve planning (Bezier + obstacle warping) when the primitive needs a trajectory.
- Send validated plan back to the VPS (server) using MessagingClient for Radxa to pick up.
- Strict safety-first behavior, simulation-friendly.
- Extensive logging + retry + backoff.

Important safety notes (READ BEFORE USING ON REAL DRONE):
- This module does NOT actuate motors directly. It emits *high-level safe primitives*.
- Always test in SITL / simulation (PX4 SITL, Gazebo, or indoors with props off).
- Keep human-in-loop override ready (joystick / RC switch).
"""

import asyncio
import time
import os
import json
import base64
import traceback
import cv2
import numpy as np
import threading
import aiohttp # NEW
from ultralytics import YOLO
from typing import Optional, List, Dict, Any

# Local modules
from laptop_ai.camera_fusion import CameraFusion
from laptop_ai.gopro_driver import GoProDriver
from laptop_ai.ai_camera_brain import AICameraBrain
from laptop_ai.camera_selector import choose_camera_for_request
from laptop_ai.messaging_client import MessagingClient
from laptop_ai.vision_tracker import VisionTracker
from laptop_ai.multimodal_prompter import ask_gpt
from laptop_ai.cinematic_planner import to_safe_primitive
from laptop_ai.ultra_director import UltraDirector
from laptop_ai.memory_client import read_memory, write_memory
from laptop_ai.esp32_driver import ESP32Driver
from laptop_ai.config import RTSP_URL, TEMPORAL_SMOOTHING, FRAME_SKIP, TEMP_ARTIFACT_DIR, CAM_WIDTH, CAM_HEIGHT

# FRAME_SKIP = 1 # Controlled by Config now
SIMULATION_ONLY = False 

# Safety configuration
# Safety configuration
MAX_FRAME_WAIT = 2.0
JOB_PROCESS_TIMEOUT = 60.0
DEBUG_SAVE_FRAME = True

# --- THREADED YOLO CLASS ---
class ThreadedYOLO:
    """
    Runs YOLO inference in a separate thread to avoid blocking the render loop.
    """
    def __init__(self, model_path):
        import time
        import threading
        self.model = YOLO(model_path)
        self.lock = threading.Lock()
        self.frame = None
        self.latest_detections = []
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def update(self, frame):
        """Push a new frame for inference."""
        if frame is None: return
        with self.lock:
            self.frame = frame.copy()

    def get_latest_detections(self):
        """Get the most recent detection results."""
        with self.lock:
            return self.latest_detections

    def _worker(self):
        while self.running:
            input_frame = None
            with self.lock:
                if self.frame is not None:
                    input_frame = self.frame.copy()
                    self.frame = None # Consume
            
            if input_frame is not None:
                # Inference
                results = self.model(input_frame, verbose=False)
                new_dets = results[0].boxes
                
                with self.lock:
                    self.latest_detections = new_dets
            else:
                time.sleep(0.01)

class Director:
    def __init__(self):
        print("Initializing Director Core...")
        self.ws = MessagingClient("laptop_vision")
        self.autopilot = AICameraBrain() # This is the Mavlink Controller
        self.frame_count = 0
        self.tracker = None
        self.classifier = None
        self.ultra_director = None
        
        # Init components
        self.camera_selector = "MAIN" # Default
        self.gopro = GoProDriver()
        self.esp32 = ESP32Driver()
        
        # State
        self.current_plan = None
        self.is_recording = False
        self.recording_start_time = 0
        
        self.gpu_tone_engine = None
        self.last_app_heartbeat = time.time() # Initialize active
        self.conn_failsafe_triggered = False
        self.last_known_user_loc = None # [lat, lon]

    async def start(self):
        await self.ws.connect()
        self.ws.add_recv_handler(self._handle_packet)
        # Start the continuous vision/streaming loop
        asyncio.create_task(self._vision_streaming_loop())
        # Start Polling for Plans (Server Disk -> Laptop)
        asyncio.create_task(self._poll_for_jobs()) 
        print("Director: connected to messaging service and vision loop started.")
        self.autopilot.connect()

    async def _poll_for_jobs(self):
        """
        Periodically poll the server for new AI plans.
        """
        from laptop_ai.config import API_BASE
        
        print(f"📡 Polling {API_BASE}/plan/next for jobs...")
        
        async with aiohttp.ClientSession() as session:
            while True:
                try:
                    async with session.get(f"{API_BASE}/plan/next") as resp:
                        if resp.status == 200:
                            data = await resp.json()
                            if data and data.get("plan"):
                                print(f"✨ NEW PLAN RECEIVED: {data['plan']}")
                                await self._execute_plan(data['plan'])
                except Exception as e:
                    print(f"Polling Error: {e}")
                
                await asyncio.sleep(2.0) # Poll every 2 seconds

    async def _execute_plan(self, plan: dict):
        """
        Execute the fetched plan. 
        Uses UltraDirector for complex path planning.
        """
        action = plan.get("action", "hover")
        params = plan.get("params", {})
        print(f"🎬 EXECUTING ACTION: {action} | Params: {params}")
        
        # 1. Simple Actions (Autopilot Direct)
        if action == "takeoff":
            self.autopilot.takeoff()
            return
        elif action == "land":
            self.autopilot.land()
            return
        elif action == "rth":
            self.autopilot.rth()
            return

        # 2. Cinematic Actions (Requires UltraDirector)
        if self.ultra_director:
            # Get current state from Tracker
            start_pos = self.autopilot.get_position() or [0,0,0]
            
            # Find subject for "Follow" / "Orbit"
            subject_pos = [0,0,0]
            if self.tracker:
                 # Get top ranked subject
                 ranked = self.tracker.get_ranked_subjects()
                 if ranked:
                     # Predict where subject will be
                     subject_pos = ranked[0].predict_position(dt=1.0).tolist() + [0] # 2D -> 3D
            
            # Generate Bezier Curve
            user_intent = plan.get("reasoning", "cinematic move")
            obstacles = [] # TODO: Get from depth map
            
            curve_plan = self.ultra_director.plan(user_intent, start_pos, subject_pos, obstacles)
    async def _vision_streaming_loop(self):
        print("👁️ Director Vision Loop Active")
        self.frame_count = 0
        
        # Initialize Threaded YOLO
        self.threaded_yolo = ThreadedYOLO("yolov8n.pt")
        # Initialize Ultra Director (Path Planner)
        try:
            from laptop_ai.ultra_director import UltraDirector
            self.ultra_director = UltraDirector()
            print("✅ DIRECTOR ON SET: UltraDirector Path Planner Active.")
        except ImportError:
            self.ultra_director = None
            print("⚠️ UltraDirector not found.")

            self.tone_engine = None
            self.rrt_enhancer = None

        # --- INTELLIGENCE MODULES (BRAIN) ---
        try:
            from laptop_ai.ai_subject_tracker import AdvancedAISubjectTracker
            from laptop_ai.ai_scene_classifier import SceneClassifier
            from laptop_ai.ai_autofocus import AIAutofocus
            self.tracker = AdvancedAISubjectTracker(max_lost=10)
            self.classifier = SceneClassifier(use_torch=False)
            self.autofocus = AIAutofocus()
        except ImportError:
            self.tracker = None
            self.classifier = None
            self.autofocus = None
            self.autofocus = None

        # --- CAMERA & VIDEO SETUP (5.3K / MAX RES) ---
        cam_stream = None
        actual_width, actual_height = CAM_WIDTH, CAM_HEIGHT # Default fallback
        
        if not SIMULATION_ONLY:
            try:
                from laptop_ai.threaded_camera import CameraStream
                # REQUEST MAX CONFIG RESOLUTION (5.3K / 5MP)
                print(f"📷 REQUESTING RESOLUTION: {CAM_WIDTH}x{CAM_HEIGHT}")
                cam_stream = CameraStream(src=0, width=CAM_WIDTH, height=CAM_HEIGHT, fps=30).start()
                if not cam_stream.working:
                    print("⚠️ Hardware Camera not found. Switched to SIMULATION.")
                    cam_stream = None
                else:
                    if cam_stream.frame is not None:
                         actual_height, actual_width = cam_stream.frame.shape[:2]
                         print(f"✅ CAMERA NEGOTIATED: {actual_width}x{actual_height}")
            except: cam_stream = None

        # --- VIDEO RECORDING SETUP ---
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        timestamp = int(time.time())
        # USE ACTUAL RESOLUTION
        video_out = cv2.VideoWriter(f"cinematic_master_{timestamp}.mp4", fourcc, 30.0, (actual_width, actual_height))
        print(f"🎥 RECORDING STARTED: cinematic_master_{timestamp}.mp4 ({actual_width}x{actual_height} ULTRA ACES)") 

        # --- MAIN LOOP ---
        while True:
            if cam_stream:
                raw_frame = cam_stream.read()
            else:
                # Simulation Frame
                raw_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(raw_frame, "SIMULATION MODE", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            if raw_frame is None:
                await asyncio.sleep(0.01)
                continue

            self.frame_count += 1
            display_frame = raw_frame.copy()

            # 1. Update YOLO
            if self.frame_count % FRAME_SKIP == 0:
                self.threaded_yolo.update(raw_frame)
            
            # 2. Get Tracks
            detections = self.threaded_yolo.get_latest_detections()
            tracked_subjects = []
            if self.tracker and detections:
                 # Mock conversion for YOLO results to tracker format
                 # In real usage we'd parse .boxes properly
                 pass

            # 3. Dynamic Grading (Unified Pipeline)
            if self.tone_engine:
                 # Calculate simple metrics
                 luminance = np.mean(raw_frame)
                 metrics = {"avg_luminance": luminance, "clipping_ratio": 0.05}
                 
                 # Apply ACES
                 graded = self.tone_engine.apply_adaptive(raw_frame, metrics)
                 # Apply Glow
                 if self.rrt_enhancer:
                     graded = self.rrt_enhancer.apply_all(graded)
                 
                 # Blend back for display (or use graded as display)
                 display_frame = graded

            # 4. Info Overlays
            cv2.putText(display_frame, f"REC: {self.is_recording}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 5. Show
            cv2.imshow("Director View", display_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.threaded_yolo.stop()
                break

            # 6. Record
            video_out.write(display_frame)
            
            # 7. Network Yield
            await asyncio.sleep(0.001)

        # Cleanup
        video_out.release()
        cv2.destroyAllWindows()

    async def _handle_packet(self, packet):
        """
        Messaging client will call this when any new packet arrives.
        """
        try:
            self.last_app_heartbeat = time.time() # Any packet is a heartbeat
            
            # Extract User GPS from any packet if present
            if "user_gps" in packet:
                self.last_known_user_loc = packet["user_gps"] # [lat, lon]
                
            t = packet.get("type")
            if t == "ai_job":
                asyncio.create_task(self.process_job(packet))
            elif t == "command":
                cmd = packet.get("action", "").upper()
                if cmd == "RTH":
                     print("🏠 MANUAL RTH TRIGGERED via APP")
                     # Reuse Failsafe Logic
                     if self.autopilot.connected:
                        if self.last_known_user_loc:
                            print(f"📍 RTH TARGET: USER LOC {self.last_known_user_loc}")
                            self.autopilot.fly_to_coords(self.last_known_user_loc[0], self.last_known_user_loc[1])
                        else:
                            print("⚠️ RTH TARGET: HOME (No User Loc)")
                            self.autopilot.return_to_launch()
                elif cmd == "LAND":
                     if self.autopilot.connected: self.autopilot.execute_primitive({"action": "LAND"})
                elif cmd == "TAKEOFF":
                     if self.autopilot.connected: self.autopilot.execute_primitive({"action": "TAKEOFF"})
                elif cmd.startswith("SET_CONFIG"):
                     # Format: SET_CONFIG: key=value
                     try:
                         _, kv = cmd.split(":", 1)
                         key, val = kv.split("=", 1)
                         key = key.strip()
                         val = val.strip()
                         print(f"⚙️ EXECUTING CONFIG CHANGE: {key} -> {val}")
                         
                         if key == "avoidance":
                             self.avoidance_enabled = (val.lower() == "true")
                             print(f"🛡️ AVOIDANCE SYSTEM: {'ENABLED' if self.avoidance_enabled else 'DISABLED'}")
                         elif key == "vision":
                             print(f"👁️ VISION POSITIONING: {val}")
                             # Enable/Disable Optical Flow logic if implemented
                         elif key == "res":
                             print(f"📷 CAMERA RESOLUTION SET: {val}")
                             # If using GoPro/RPi, re-init stream here
                     except Exception as e:
                         print(f"⚠️ CONFIG ERROR: {e}")
            else:
                pass 
        except Exception:
            traceback.print_exc()
            traceback.print_exc()

    async def process_job(self, job: dict):
        """
        Top-level job processing pipeline.
        """
        if self.processing:
            print("Director busy. Requeuing job.")
            await asyncio.sleep(1.0)
            return

        self.processing = True
        job_id = job.get("job_id", f"job_{int(time.time())}")
        user_id = job.get("user_id")
        drone_id = job.get("drone_id")
        user_text = job.get("text", "")
        images = job.get("images", [])
        video_link = job.get("video")
        
        print(f"\n=== Starting job {job_id} text='{user_text[:50]}' ===")
        
        try:
            # 1. Grab Frame
            frame = await asyncio.to_thread(self._grab_frame, RTSP_URL, MAX_FRAME_WAIT)
            if frame is None:
                await self._send_plan(job_id, user_id, drone_id, {"action": "HOVER"}, reason="no_frame")
                return

            if DEBUG_SAVE_FRAME:
                cv2.imwrite(os.path.join(TEMP_ARTIFACT_DIR, f"job_{job_id}_ctx.jpg"), frame)

            # 2. Vision Context
            vision_context, annotated = await asyncio.to_thread(self.tracker.process_frame, frame)
            
            # 3. Memory
            memory = read_memory(user_id, drone_id) or {}
            
            # 4. Cloud Prompt (DeepSeek/GPT)
            print("Director: calling multimodal prompter...")
            # Extract Keys from Job if present (BYOK Support for Laptop)
            job_keys = job.get("api_keys", {}) 
            raw = await asyncio.to_thread(ask_gpt, user_text, vision_context, images, video_link, memory, api_keys=job_keys)
            
            # 5. Planning
            primitive = to_safe_primitive(raw or {"action": "HOVER"})
            if primitive is None: primitive = {"action": "HOVER"}
            
            camera_choice = choose_camera_for_request(user_text, primitive, vision_context)
            if "meta" not in primitive: primitive["meta"] = {}
            primitive["meta"]["camera_choice"] = camera_choice
            
            # 6. Ultra Director (Curves)
            if primitive.get("action") in ("FOLLOW", "ORBIT", "TRACK_PATH"):
                start_pos = primitive.get("params", {}).get("start_pos") or [0.0, 0.0, 2.5]
                target_pos = primitive.get("params", {}).get("target_pos") or [1.5, 0.0, 2.5]
                
                curve, mode = self.ultra.plan_shot(primitive.get("params", {}), vision_context, start_pos, target_pos)
                if curve:
                    primitive["plan_curve"] = {
                        "duration": self.ultra.duration,
                        "control_points": [list(map(float, p)) for p in [curve.p0, curve.p1, curve.p2, curve.p3]]
                    }
                    primitive["meta"]["mode"] = mode
                elif mode == "unsafe_hover":
                    primitive = {"action": "HOVER"}
            
            # 6b. Gimbal Control (from AI meta/reasoning)
            # If AI suggests an angle, move gimbal
            cam_angle = raw.get("camera_angle", "eye_level")
            pitch = 0
            if cam_angle == "high_angle": pitch = -30
            elif cam_angle == "low_angle": pitch = 20
            # TODO: Add dynamic look_at based on YOLO center
            
            if self.esp32:
                self.esp32.set_gimbal(pitch, 0)
                if primitive.get("action") == "ORBIT":
                     self.esp32.set_led("BLUE") # Recording Color
                else:
                     self.esp32.set_led("GREEN")
            
            # 7. Final Send (Server)
            primitive = to_safe_primitive(primitive)
            await self._send_plan(job_id, user_id, drone_id, primitive, reason="ok")
            
            # 8. Local Execution (Fast)
            if self.autopilot.connected:
                self.autopilot.execute_primitive(primitive)
            
        except Exception as e:
            print(f"Job Error: {e}")
            traceback.print_exc()
            await self._send_plan(job_id, user_id, drone_id, {"action": "HOVER"}, reason="error")
        finally:
            self.processing = False
            print(f"=== Finished job {job_id} ===\n")

    def _grab_frame(self, rtsp_url: str, timeout_s: float) -> Optional[any]:
        cap = cv2.VideoCapture(rtsp_url if rtsp_url else 0)
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            ret, frame = cap.read()
            if ret and frame is not None:
                cap.release()
                return frame
            time.sleep(0.05)
        try: cap.release() 
        except: pass
        return None

    async def _send_plan(self, job_id, user_id, drone_id, primitive, reason="ok"):
        if self.simulate:
            print(f"[SIMULATION] Sending plan: {primitive.get('action')}")
            return
            
        packet = {
            "target": "server",
            "type": "ai_plan",
            "job_id": job_id,
            "user_id": user_id,
            "drone_id": drone_id,
            "primitive": primitive,
            "meta": {"source": "laptop", "reason": reason, "ts": time.time()}
        }
        
        def safe_serialize(obj):
            if hasattr(obj, 'tolist'): return obj.tolist()
            if hasattr(obj, '__dict__'): return obj.__dict__
            return str(obj)

        for attempt in range(3):
            try:
                msg = json.dumps(packet, default=safe_serialize)
                await self.ws.send(json.loads(msg))
                print(f"Plan sent (job={job_id})")
                return
            except Exception:
                await asyncio.sleep(0.1)

# --- CLI ---
async def main_loop(simulate=False):
    d = Director(simulate=simulate)
    await d.start()
    try:
        while True: await asyncio.sleep(1.0)
    except asyncio.CancelledError:
        pass

if __name__ == "__main__":
    asyncio.run(main_loop(simulate=False))
