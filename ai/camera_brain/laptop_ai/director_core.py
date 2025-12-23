
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
from laptop_ai.config import RTSP_URL, TEMPORAL_SMOOTHING, FRAME_SKIP, TEMP_ARTIFACT_DIR

FRAME_SKIP = 2
SIMULATION_ONLY = False 

# Safety configuration
MAX_FRAME_WAIT = 2.0
JOB_PROCESS_TIMEOUT = 60.0
DEBUG_SAVE_FRAME = True

os.makedirs(TEMP_ARTIFACT_DIR, exist_ok=True)

# --- THREADED YOLO CLASS ---
class ThreadedYOLO:
    """
    Runs YOLO inference in a separate thread to avoid blocking the render loop.
    """
    def __init__(self, model_name="yolov8n.pt"):
        print(f"🚀 Initializing Threaded YOLO ({model_name})...")
        self.model = YOLO(model_name)
        self.model.to('cuda' if os.environ.get('CUDA_VISIBLE_DEVICES') != '-1' else 'cpu')
        print(f"🚀 YOLO Device: {self.model.device}")
        
        self.frame = None
        self.lock = threading.Lock()
        self.running = True
        self.latest_detections = []
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.names = self.model.names

    def start(self):
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
                    input_frame = self.frame
                    self.frame = None # Consume it
            
            if input_frame is None:
                time.sleep(0.001)
                continue
            
            try:
                # Run Inference
                results = self.model(input_frame, stream=True, verbose=False)
                new_dets = []
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        conf = float(box.conf[0])
                        if conf > 0.4:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            cls_id = int(box.cls[0])
                            label = self.names[cls_id]
                            new_dets.append({
                                "label": label,
                                "cls": label,
                                "conf": conf,
                                "box": [x1, y1, x2, y2],
                                "bbox": box.xyxy[0].tolist(),
                                "confidence": conf
                            })
                
                with self.lock:
                    self.latest_detections = new_dets
                    
            except Exception as e:
                print(f"YOLO Thread Error: {e}")
                time.sleep(0.01)


class Director:
    def __init__(self, client_id="laptop", simulate: bool = False):
        self.ws = MessagingClient(client_id=client_id)
        self.tracker = VisionTracker()
        self.ultra = UltraDirector()
        self.fusion = CameraFusion()
        self.gopro = GoProDriver()
        self.ai_cam = AICameraBrain()
        self.simulate = simulate
        self.processing = False
        self.last_job_time = 0
        self.frame_skip = FRAME_SKIP
        
        self.threaded_yolo = None
        self.gpu_tone_engine = None

    async def start(self):
        await self.ws.connect()
        self.ws.add_recv_handler(self._handle_packet)
        # Start the continuous vision/streaming loop
        asyncio.create_task(self._vision_streaming_loop())
        print("Director: connected to messaging service and vision loop started.")

    async def _vision_streaming_loop(self):
        """
        Continuous loop: Capture -> Inference -> Brain -> Stream to Cloud.
        """
        print("👁️ Director Vision Loop Active")
        self.frame_count = 0
        
        # Initialize Threaded YOLO
        self.threaded_yolo = ThreadedYOLO("yolov8n.pt")
        self.threaded_yolo.start()

        # Initialize Camera
        if SIMULATION_ONLY:
            print("⚠️ SIMULATION_MODE: Camera Disabled")
            cam_stream = None
        else:
            try:
                from laptop_ai.threaded_camera import CameraStream
                cam_stream = CameraStream(src=0, width=640, height=480, fps=60).start()
                if not cam_stream.working:
                    print("⚠️ Hardware Camera not found. Switched to SIMULATION.")
                    cam_stream = None
            except Exception as e:
                print(f"⚠️ Camera Error: {e}")
                cam_stream = None

        last_loop_time = time.time()
        t_cap_start = 0; t_cap_end = 0
        
        # Main Loop
        while True:
            self.frame_count += 1
            t_cap_start = time.time()
            
            # 1. Capture Frame
            if cam_stream:
                frame = cam_stream.read()
                if frame is None:
                     frame = np.zeros((480, 640, 3), dtype=np.uint8)
            else:
                frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "SIMULATION", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            t_cap_end = time.time()
            
            if frame is None:
                await asyncio.sleep(0.01)
                continue
                
            display_frame = frame.copy()
            
            # --- 2. GPU ACES (Tone Mapping) ---
            if not self.gpu_tone_engine:
                 try:
                     from ai.camera_brain.laptop_ai.ai_pipeline.tone_curve.gpu_aces import GPUACESToneCurve
                     self.gpu_tone_engine = GPUACESToneCurve()
                 except: pass

            if self.gpu_tone_engine:
                 try:
                    display_frame = self.gpu_tone_engine.apply(display_frame)
                    cv2.putText(display_frame, "GPU ACES (RTX 5070 Ti)", (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                 except: pass
            
            # --- 3. Update Camera Fusion ---
            self.fusion.update_internal_frame(frame)
            fusion_state = self.fusion.get_fusion_state()
            
            # --- 4. Threaded YOLO Inference ---
            # Send current frame to YOLO thread
            af_detect = self.fusion.get_active_frame()
            active_frame = af_detect if af_detect is not None else frame
            self.threaded_yolo.update(active_frame)
            
            # Get latest results immediately
            detections = self.threaded_yolo.get_latest_detections()
            
            # Draw Detections
            for d in detections:
                x1, y1, x2, y2 = d["box"]
                label_text = f"{d['label']} {d['conf']:.2f}"
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(display_frame, label_text, (x1, y1 - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # --- 5. Async Brain Decision ---
            brain_plan = self.ai_cam.decide(
                user_text="", 
                fusion_state=fusion_state, 
                frame=frame,
                vision_context={"detections": detections}
            )
            
            # Visualize Brain Mode
            mode = "CINEMATIC"
            if brain_plan.get("scene", {}).get("action", 0) > 0.5: mode = "ACTION"
            cv2.putText(display_frame, f"BRAIN: {mode}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # --- 6. Telemetry & Display ---
            curr_time = time.time()
            fps = 1.0 / (curr_time - last_loop_time + 1e-9)
            last_loop_time = curr_time
            
            cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            cv2.imshow("Director View - Laptop AI", display_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.threaded_yolo.stop()
                break
            
            # Yield for network IO
            await asyncio.sleep(0)

    async def _handle_packet(self, packet):
        """
        Messaging client will call this when any new packet arrives.
        """
        try:
            t = packet.get("type")
            if t == "ai_job":
                asyncio.create_task(self.process_job(packet))
            else:
                pass 
        except Exception:
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
            raw = await asyncio.to_thread(ask_gpt, user_text, vision_context, images, video_link, memory)
            
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
            
            # 7. Final Send
            primitive = to_safe_primitive(primitive)
            await self._send_plan(job_id, user_id, drone_id, primitive, reason="ok")
            
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
