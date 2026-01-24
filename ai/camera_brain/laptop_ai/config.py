# laptop_ai/config.py
import os

# EDIT THESE VALUES for your environment.

# WebSocket relay (Railway/Render).
VPS_WS = os.getenv("VPS_WS_URL", "wss://drone-server-r0qe.onrender.com/ws/connect/laptop_vision")

# HTTP server base (FastAPI endpoints)
API_BASE = os.getenv("API_BASE", "https://drone-server-r0qe.onrender.com")

# Shared secret token (must match server)
AUTH_TOKEN = os.getenv("AUTH_TOKEN", "SUPER_SECRET_DRONE_KEY_123")

# OpenAI / GPT config
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")
OPENAI_MODEL = os.getenv("OPENAI_MODEL", "gpt-4o-mini")

# DeepSeek / Local LLM Config (For Ultra-Low Latency Reasoning)
DEEPSEEK_API_KEY = os.getenv("DEEPSEEK_API_KEY", "sk-...")
DEEPSEEK_URL = os.getenv("DEEPSEEK_URL", "https://api.deepseek.com/v1")
USE_LOCAL_LLM = os.getenv("USE_LOCAL_LLM", "False").lower() == "true"

# YOLO model path (we default to yolov8n.pt). Put the .pt file in laptop_ai/ or change path.
YOLO_MODEL_PATH = os.getenv("YOLO_MODEL_PATH", "yolov8n.pt")

# RTSP / camera input for laptop
_rtsp = os.getenv("RTSP_URL", "0")
RTSP_URL = int(_rtsp) if _rtsp.isdigit() else _rtsp

# Camera Resolution Config (High Res Logic - Downscaled for Stream)
# 5.3K = 5312 x 2988 (Recording / AI Analysis)
CAM_WIDTH = 5312  
CAM_HEIGHT = 2988
CAM_FPS = 30

# Performance config
AI_CALL_INTERVAL = 2.0   # Fast re-planning for maximum responsiveness
FRAME_SKIP = 1           # PROCESS EVERY SINGLE FRAME (Cinema Quality)
TEMPORAL_SMOOTHING = 0.7 # Increased smoothing for steadycam feel

# artifact dir
TEMP_ARTIFACT_DIR = "./artifacts"
MAX_IMAGES_UPLOAD = 25
MAX_VIDEO_UPLOAD_MB = 100
