# laptop_ai/config.py
import os
# EDIT THESE VALUES for your environment.

# WebSocket relay (Railway). Use wss://... - include path your server expects (e.g. /ws/connect/<id>)
# Example: wss://web-production-fdccc.up.railway.app/ws/connect/laptop
# WebSocket relay (Railway). Use wss://... - include path your server expects (e.g. /ws/connect/<id>)
VPS_WS = os.getenv("VPS_WS_URL", "wss://web-production-fdccc.up.railway.app/ws/connect/laptop")

# HTTP server base (FastAPI endpoints)
API_BASE = os.getenv("API_BASE", "https://web-production-fdccc.up.railway.app")

# Shared secret token (must match server)
AUTH_TOKEN = os.getenv("AUTH_TOKEN", "SUPER_SECRET_DRONE_KEY_123")

# OpenAI / GPT config (place your key after verification)
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")
OPENAI_MODEL = os.getenv("OPENAI_MODEL", "gpt-4o-mini")

# YOLO model path (we default to yolov8n.pt). Put the .pt file in laptop_ai/ or change path.
YOLO_MODEL_PATH = os.getenv("YOLO_MODEL_PATH", "yolov8n.pt")

# RTSP / camera input for laptop (set to your Radxa RTSP or local camera index)
# If env var is set to a digit, cast to int for webcam index
_rtsp = os.getenv("RTSP_URL", "0")
RTSP_URL = int(_rtsp) if _rtsp.isdigit() else _rtsp

# Performance config
AI_CALL_INTERVAL = 4.0   # seconds between full AI cloud calls for same job
FRAME_SKIP = 2           # process every Nth frame for vision (increase to save CPU)

# artifact dir
TEMP_ARTIFACT_DIR = "./artifacts"
MAX_IMAGES_UPLOAD = 25
MAX_VIDEO_UPLOAD_MB = 100
