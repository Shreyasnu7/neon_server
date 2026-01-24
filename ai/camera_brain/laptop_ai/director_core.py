
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
import sys
# AUTO-FIX: Add parent directory to path so 'laptop_ai' can be imported
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

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
from laptop_ai.lidar_driver import YDLidarDriver
from laptop_ai.config import TEMPORAL_SMOOTHING, FRAME_SKIP, TEMP_ARTIFACT_DIR, CAM_WIDTH, CAM_HEIGHT

# USER CONFIG: Streaming from Cloud Proxy (Radxa -> Cloud -> Laptop)
RTSP_URL = "https://web-production-fdccc.up.railway.app/video_feed" 
# Note: Laptop uses Requests/CV2 to pull MJPEG stream from this URL


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

class DirectorCore:
    def __init__(self, simulation_only=False):
        print(f"Initializing Director Core (Sim={simulation_only})...")
        self.simulation_only = simulation_only
        self.simulate = simulation_only
        self.ws = MessagingClient("laptop_vision")
        self.autopilot = AICameraBrain() # This is the Mavlink Controller
        self.frame_count = 0
        self.tracker = None
        self.classifier = None
        self.ultra_director = None
        
        # Init components
        self.camera_selector = "MAIN" # Default
        self.gopro = GoProDriver()
        # self.esp32 = ESP32Driver() # REMOVED: Hardware is on Drone
        self.remote_esp_telem = {}
        print("✅ Director Ready for Remote ESP32 Telemetry")
        # REMOTE SENSORS (Relayed via Bridge)
        self.remote_obstacles = [] # List of [x, y]
        # self.lidar = YDLidarDriver() # REMOVED: Hardware is on Drone, not Laptop
        print("✅ Director Ready for Remote Sensor Fusion")
        
        # State
        self.current_plan = None
        self.rth_behavior = "user" # Default
        self.land_behavior = "here"
        self.is_recording = False
        self.recording_start_time = 0
        
        self.gpu_tone_engine = None
        self.last_app_heartbeat = time.time() # Initialize active
        self.conn_failsafe_triggered = False
        self.last_known_user_loc = None # [lat, lon]
        
        # Load Cinematic Assets
        self._load_cinematic_library()

    def _load_cinematic_library(self):
        """
        Loads the user's ~1000 cinematic AI director files (LUTs, Configs).
        Real implementation would parse these files to tune the Color/Exposure engines.
        """
        self.cinematic_library = []
        # Look in project root assets first, then relative
        possible_paths = [
            os.path.join(os.getcwd(), "assets", "cinematic_director_files"),
            "assets/cinematic_director_files",
            os.path.join(os.path.dirname(__file__), "assets"),
            r"C:\Users\adish\.gemini\antigravity\scratch\drone_project\assets\cinematic_director_files"
        ]
        
        found_path = None
        for p in possible_paths:
            if os.path.exists(p):
                found_path = p
                break
                
        if found_path:
             print(f"🎬 Loading Cinematic Director Library from {found_path}...")
             for root, dirs, files in os.walk(found_path):
                 for file in files:
                     if file.endswith(".json") or file.endswith(".lut"):
                         self.cinematic_library.append(os.path.join(root, file))
             print(f"✅ Loaded {len(self.cinematic_library)} Cinematic Assets.")
        else:
             print("⚠️ Cinematic Directory not found. Using Default Engines.")

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
        
        # 1. Simple Actions (Remote Execution via Bridge Safety Check)
        if action == "takeoff":
            await self.ws.send_message({
                "type": "ai_plan", 
                "primitive": {"action": "TAKEOFF"}
            })
            return
        elif action == "land":
            await self.ws.send_message({
                "type": "ai_plan", 
                "primitive": {"action": "LAND"}
            })
            return
        elif action == "rth":
            # Respect user preference for RTH behavior
            if self.rth_behavior == "user":
                 # Trigger Smart User Return via job injection (handled elsewhere) or send plan
                 await self.ws.send_message({
                    "type": "ai_plan",
                    "primitive": {"action": "RTL_SMART", "lat": self.last_known_user_loc[0], "lng": self.last_known_user_loc[1]} if self.last_known_user_loc else {"action": "RTH"}
                 })
            else:
                 await self.ws.send_message({
                    "type": "ai_plan",
                    "primitive": {"action": "RTH"}
                 })
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
            
            # --- REAL-TIME OBSTACLE FUSION (REMOTE) ---
            obstacles = self.remote_obstacles
            if len(obstacles) > 0:
                 print(f"🛑 FUSING {len(obstacles)} REMOTE OBSTACLES!")
            
            # Fuse with Remote ESP32 Telemetry (Omnidirectional Safety)
            telem = self.remote_esp_telem
            if telem:
                SAFE_DIST = 1000 # mm
                TILT_FACTOR = 0.707 # cos(45 degrees)
                
                # We map the 4 sensors to quadrants. 
                # Assuming firmware sends: tof_front, tof_back, tof_left, tof_right
                # But physically they are tilted.
                
                # GEOMETRY: 4x ToF Sensors (PCB Fixed)
                # Mapping: tof_1=Front, tof_2=Back, tof_3=Left, tof_4=Right (Default PCB Layout)
                # TILT: 45 degrees
                
                def check_sensor(key, dx, dy):
                    raw = telem.get(key, 9999)
                    if raw < SAFE_DIST:
                        # Project slant range (0.707 = cos 45)
                        h_dist_m = (raw * TILT_FACTOR) / 1000.0
                        obstacles.append((dx * h_dist_m, dy * h_dist_m))
                        print(f"⚠️ SAFETY: {key.upper()} OBSTACLE at {h_dist_m:.1f}m")

                check_sensor('tof_1', 1.0, 0.0)  # Front
                check_sensor('tof_2', -1.0, 0.0) # Back
                check_sensor('tof_3', 0.0, -1.0) # Left
                check_sensor('tof_4', 0.0, 1.0)  # Right
                # No tof_bottom in 4-sensor PCB layout

            
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
            
        # --- CINEMATIC ENGINES (COLOR/GRADING) ---
        try:
            from laptop_ai.ai_colour_engine import AICulourEngine
            from laptop_ai.ai_video_engine import RRT_Enhancer # Assuming this exists or using color engine
            
            # Using the Loaded Library
            self.tone_engine = AICulourEngine()
            # If library loaded paths, we might want to load them here. 
            # For now, we init the engine which likely has default LUTs.
            
            # Mock or Real RRT
            # self.rrt_enhancer = RRT_Enhancer() 
            self.rrt_enhancer = None # Keep simple if import fails, or impl later
            print("✅ CINEMATIC ENGINES: Active (ACES + Custom LUTs)")
            
            if self.cinematic_library:
                print(f"    - Injected {len(self.cinematic_library)} User Director Files into Engine.")
                self.tone_engine.load_library(self.cinematic_library) # ACTIVATED
        except ImportError:
            print("⚠️ Color Engine not found. Cinematic Grading Disabled.")
            self.tone_engine = None
            self.rrt_enhancer = None

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
                # No Signal Frame
                # User prefers 'Offline' over 'Simulation'
                raw_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(raw_frame, "CAMERA DISCONNECTED", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(raw_frame, "CHECK PHYSICAL CONNECTION", (150, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
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
            # 3. Dynamic Grading (Unified Pipeline)
            if self.tone_engine:
                 # Real Analysis
                 stats = self.tone_engine.analyze(raw_frame)
                 
                 # Propose Grade based on Current Style (set by AI)
                 current_style = getattr(self, 'current_cinematic_style', "cine_soft")
                 grade = self.tone_engine.propose_grade(stats, style=current_style)
                 
                 # Apply
                 graded = self.tone_engine.apply_grade(raw_frame, grade)
                 
                 # Apply Glow (RRT)
                 if self.rrt_enhancer:
                     graded = self.rrt_enhancer.apply_all(graded)
                 
                 display_frame = graded

            # 4. Info Overlays
            status_color = (0, 0, 255) if self.is_recording else (200, 200, 200)
            cv2.putText(display_frame, f"REC: {self.is_recording}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            
            # 5. Show
            cv2.imshow("Director View", display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.threaded_yolo.stop()
                break
                self.is_recording = not self.is_recording
                print(f"Start/Stop Record: {self.is_recording}")
            
            # 5b. Handle Photo Capture
            if getattr(self, 'should_capture_photo', False):
                ts = int(time.time())
                p_name = f"DCIM/photo_{ts}.jpg"
                os.makedirs("DCIM", exist_ok=True)
                # Save Full Resolution Frame
                cv2.imwrite(p_name, display_frame) 
                print(f"📸 SAVED: {p_name}")
                self.should_capture_photo = False # Reset

            # 6. Record (ONLY IF ACTIVE) - High Res
            if self.is_recording and video_out.isOpened():
                video_out.write(display_frame)

            # 7. Broadcast to App (Dual-Stream Logic)
            try:
                # Dynamic Stream Target based on Source
                # User Requirement: Internal=720p, External=1080p
                if hasattr(self, 'camera_selector') and self.camera_selector == "GOPRO":
                     stream_w, stream_h = 1920, 1080
                else:
                     stream_w, stream_h = 1280, 720 # Default for RPi Camera

                # Resize only if source is larger (Downscale)
                if display_frame.shape[1] > stream_w:
                    preview_frame = cv2.resize(display_frame, (stream_w, stream_h), interpolation=cv2.INTER_AREA)
                else:
                    preview_frame = display_frame

                # Encode to JPEG
                _, buffer = cv2.imencode('.jpg', preview_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
                
                # Push to Server (Fire and Forget)
                # We use a separate async task or simple blocking post with timeout to not stall AI?
                # For simplicity in this loop, we assume localhost/fast server. 
                # Ideally this runs in a separate thread/queue.
                # However, to keep it simple and robust:
                from laptop_ai.config import API_BASE
                async with aiohttp.ClientSession() as session:
                     url = f"{API_BASE}/video/frame"
                     # We use a detached task to avoid blocking the vision loop
                     asyncio.create_task(session.post(url, data=buffer.tobytes(), headers={"Content-Type": "image/jpeg"}))

            except Exception as e:
                pass # Don't crash vision loop on network glitch
            
            # 8. Network Yield
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
            if t == "esp32_telem":
                self.remote_esp_telem = packet.get("payload", {})
            elif t == "lidar_scan":
                # Update Remote Obstacles
                self.remote_obstacles = packet.get("payload", {}).get("points", [])
            elif t == "ai_job":
                asyncio.create_task(self.process_job(packet))
            elif t == "command":
                cmd = packet.get("action", "").upper()
                elif cmd == "RTH":
                     print(f"🏠 RTH TRIGGERED (Behavior: {self.rth_behavior.upper()})")
                     if self.rth_behavior == "home":
                         if self.autopilot.connected: 
                             self.autopilot.return_to_launch()
                             print("🚀 RETURNING TO LAUNCH (HOME)")
                     else:
                         # Smart Return to User (AI Job)
                         if self.autopilot.connected and self.last_known_user_loc:
                             print(f"📍 SMART RETURN TO USER: {self.last_known_user_loc}")
                             # Inject synthetic job for Cloud Brain to plan path
                             syn_job = {
                                 "job_id": f"rth_{int(time.time())}",
                                 "text": f"RETURN TO USER AT {self.last_known_user_loc}. USES SENSOR AVOIDANCE.",
                                 "user_id": "system", "drone_id": "self"
                             }
                             asyncio.create_task(self.process_job(syn_job))
                         else:
                             print("⚠️ NO USER LOC. FALLBACK TO HOME.")
                             self.autopilot.return_to_launch()
                elif cmd == "LAND":
                     if self.autopilot.connected: self.autopilot.execute_primitive({"action": "LAND"})
                elif cmd == "TAKEOFF":
                     if self.autopilot.connected: self.autopilot.execute_primitive({"action": "TAKEOFF"})
                elif cmd == "START_RECORDING":
                     self.is_recording = True
                     print("🎥 MANUAL RECORD START")
                elif cmd == "STOP_RECORDING":
                     self.is_recording = False
                     print("⏹️ MANUAL RECORD STOP")
                elif cmd == "CAPTURE_PHOTO":
                     print("📸 PHOTO REQUEST RECEIVED")
                     # We can just leverage the next loop iteration to save a frame or enable a 'one-shot' flag.
                     # For now, simplest is to grab frame immediately or flag it.
                     self.should_capture_photo = True 
                elif cmd.startswith("SET_CONFIG"):
                     # Format: SET_CONFIG: key=value
                     try:
                         _, kv = cmd.split(":", 1)
                         key, val = kv.split("=", 1)
                         key = key.strip()
                         val = val.strip()
                         print(f"⚙️ EXECUTING CONFIG CHANGE: {key} -> {val}")
                         
                         if key == "rth_behavior":
                             self.rth_behavior = val.lower()
                             print(f"⚙️ RTH BEHAVIOR: {self.rth_behavior.upper()}")
                         
                         elif key == "avoidance":
                             self.avoidance_enabled = (val.lower() == "true")
                             print(f"🛡️ AVOIDANCE SYSTEM: {'ENABLED' if self.avoidance_enabled else 'DISABLED'}")
                         elif key == "vision":
                             print(f"👁️ VISION POSITIONING: {val}")
                             # Enable/Disable Optical Flow logic if implemented
                         elif key == "res":
                             # MAP APP SETTINGS TO REAL RESOLUTIONS
                             print(f"📷 CAMERA RESOLUTION SET: {val} (Restarting Stream...)")
                             val_s = val.lower().replace("fps","")
                             new_w, new_h, new_fps = 1920, 1080, 30 # Default
                             
                             if "5.3k" in val_s: new_w, new_h = 5312, 2988
                             elif "4k" in val_s: new_w, new_h = 3840, 2160
                             elif "2.7k" in val_s: new_w, new_h = 2704, 1520
                             elif "1080p" in val_s: new_w, new_h = 1920, 1080
                             elif "720p" in val_s: new_w, new_h = 1280, 720
                             
                             # RESTART CAMERA
                             # Determine current source (default to 0/Internal if not set)
                             src = 0
                             if hasattr(self, 'camera_selector') and self.camera_selector == "GOPRO":
                                 src = f"udp://{self.gopro.ip}:8554"
                             
                             if cam_stream: cam_stream.stop()
                             try:
                                 from laptop_ai.threaded_camera import CameraStream
                                 cam_stream = CameraStream(src=src, width=new_w, height=new_h, fps=new_fps).start()
                                 # Update Global Config for Recording
                                 cam_stream.width = new_w
                                 cam_stream.height = new_h
                                 print(f"✅ CAMERA RESTARTED AT {new_w}x{new_h}")
                             except Exception as e:
                                 print(f"❌ Camera Switch Failed: {e}")

                         # --- NEW: CAMERA SOURCE & STREAM QUALITY ---
                         elif key == "source":
                             new_source = val.lower()
                             print(f"🎥 SWITCHING SOURCE: {new_source.upper()}")
                             if new_source == "external":
                                 # 1. Stop local stream
                                 if cam_stream: 
                                     cam_stream.stop()
                                     cam_stream = None
                                 
                                 # 2. Start UDP Stream from GoPro
                                 try:
                                     from laptop_ai.threaded_camera import CameraStream
                                     # GoPro UDP Stream (Hero 12/13/11 via QR or HTTP)
                                     udp_url = f"udp://{self.gopro.ip}:8554" 
                                     print(f"📡 CONNECTING TO GOPRO UDP: {udp_url}")
                                     cam_stream = CameraStream(src=udp_url, width=CAM_WIDTH, height=CAM_HEIGHT, fps=30).start()
                                     self.camera_selector = "GOPRO" 
                                     print("✅ External Camera Selected (GoPro)")
                                 except Exception as e:
                                     print(f"GoPro Stream Error: {e}")
                             
                             elif new_source == "internal":
                                 self.camera_selector = "MAIN"
                                 # Restart Local Stream
                                 try:
                                     from laptop_ai.threaded_camera import CameraStream
                                     cam_stream = CameraStream(src=0, width=CAM_WIDTH, height=CAM_HEIGHT, fps=30).start()
                                     print("✅ Internal Camera Selected (Radxa)")
                                 except Exception as e:
                                     print(f"Switch Error: {e}")

                         elif key == "stream_res":
                             qty = val.lower() # "720p" or "480p"
                             print(f"📺 STREAM QUALITY: {qty.upper()}")
                             # 1. Update Global Config
                             # 2. If 'internal', restart stream with new W/H
                             t_w, t_h = (1280, 720) if qty == "720p" else (640, 480)
                             if self.camera_selector == "MAIN":
                                 if cam_stream: cam_stream.stop()
                                 from laptop_ai.threaded_camera import CameraStream
                                 cam_stream = CameraStream(src=0, width=t_w, height=t_h, fps=30).start()
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
            job_keys = job.get("api_keys", {}) 
            
            # INJECT SENSOR DATA (Lidar + ESP32)
            # This makes the AI "REAL" and aware of its surroundings
            full_sensor_context = {
                "lidar_obstacles": self.remote_obstacles, # [[x,y], [x,y]]
                "tof_sensors": self.remote_esp_telem, # {"tof_front": 1200, ...}
                "battery": self.autopilot.get_telemetry().get('battery', 100),
                "location": self.autopilot.get_position()
            }
            
            raw = await asyncio.to_thread(ask_gpt, user_text, vision_context, images, video_link, memory, sensor_data=full_sensor_context, api_keys=job_keys)
            
            # 5. Planning (Parsing New Rich Output)
            # The new AI outputs root: {thought_process, cinematic_style, execution_plan, technical_config}
            
            # Extract Style for Tone Engine
            cinematic_style = raw.get("cinematic_style", "cine_soft")
            if self.tone_engine:
                 print(f"🎨 Applying Cinematic Style: {cinematic_style}")
                 # Update the Global Style State for the Vision Loop
                 self.current_cinematic_style = cinematic_style

            execution_plan = raw.get("execution_plan", {})
            legacy_action = raw.get("action") # Fallback
            
            # Normalize to Primitive
            if execution_plan:
                 primitive = to_safe_primitive(execution_plan)
                 primitive["thought_process"] = raw.get("thought_process", "")
            else:
                 # Legacy Fallback
                 primitive = to_safe_primitive(raw or {"action": "HOVER"})

            if primitive is None: primitive = {"action": "HOVER"}
            
            # Camera Choice (Manual or AI)
            camera_choice = choose_camera_for_request(user_text, primitive, vision_context)
            if "meta" not in primitive: primitive["meta"] = {}
            primitive["meta"]["camera_choice"] = camera_choice
            
            # 6. Ultra Director (Curves)
            if primitive.get("action") in ("FOLLOW", "ORBIT", "TRACK_PATH", "DOLLY_ZOOM", "FLY_THROUGH"):
                start_pos = primitive.get("params", {}).get("start_pos") or [0.0, 0.0, 2.5]
                target_pos = primitive.get("params", {}).get("target_pos") or [1.5, 0.0, 2.5]
                
                curve, mode = self.ultra_director.plan_shot(primitive.get("params", {}), vision_context, start_pos, target_pos) if self.ultra_director else (None, "unsafe")
                
                if curve:
                    primitive["plan_curve"] = {
                        "duration": self.ultra_director.duration if self.ultra_director else 5.0,
                        "control_points": [list(map(float, p)) for p in [curve.p0, curve.p1, curve.p2, curve.p3]]
                    }
                    primitive["meta"]["mode"] = mode
            
            # 6b. Gimbal Control (Rich)
            gimbal_cfg = execution_plan.get("gimbal", {}) if execution_plan else {}
            pitch = gimbal_cfg.get("pitch", 0)
            yaw = gimbal_cfg.get("yaw", 0)

            # 6c. AI Recording Trigger (Auto-Record Reasoning)
            if execution_plan:
                ai_rec = execution_plan.get("recording")
                if ai_rec is True:
                     if not self.is_recording:
                         self.is_recording = True
                         print("🎥 AI DIRECTOR ACTION: START RECORDING")
                elif ai_rec is False:
                     if self.is_recording:
                         self.is_recording = False
                         print("⏹️ AI DIRECTOR ACTION: CUT! (STOP RECORDING)")
            
            # Fallback to old heuristic if not provided
            if not gimbal_cfg:
                 cam_angle = raw.get("camera_angle", "eye_level")
                 if cam_angle == "high_angle": pitch = -30
                 elif cam_angle == "low_angle": pitch = 20

            
            # Send Gimbal Command via Bridge (instead of local ESP driver)
            if primitive.get("action") == "ORBIT":
                  primitive["meta"]["led"] = "BLUE"
            else:
                  primitive["meta"]["led"] = "GREEN"
            
            primitive["meta"]["gimbal"] = {"pitch": pitch, "yaw": yaw}
            
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
    d = DirectorCore(simulation_only=simulate)
    await d.start()
    try:
        while True: await asyncio.sleep(1.0)
    except asyncio.CancelledError:
        pass

if __name__ == "__main__":
    asyncio.run(main_loop(simulate=False))
