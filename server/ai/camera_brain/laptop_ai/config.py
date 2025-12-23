# laptop_ai/config.py
# EDIT THESE VALUES for your environment.

# WebSocket relay (Railway). Use wss://... - include path your server expects (e.g. /ws/connect/<id>)
# Example: wss://web-production-fdccc.up.railway.app/ws/connect/laptop
VPS_WS = "wss://web-production-fdccc.up.railway.app/ws/connect/laptop"

# HTTP server base (FastAPI endpoints)
API_BASE = "https://web-production-fdccc.up.railway.app"

# Shared secret token (must match server)
AUTH_TOKEN = "SUPER_SECRET_DRONE_KEY_123"

# OpenAI / GPT config (place your key after verification)
OPENAI_API_KEY = "REPLACE_WITH_YOUR_OPENAI_KEY"
OPENAI_MODEL = "gpt-4o-mini"   # or whichever you can access (gpt-4o, gpt-4o-vision)

# YOLO model path (we default to yolov8n.pt). Put the .pt file in laptop_ai/ or change path.
YOLO_MODEL_PATH = "yolov8n.pt"

# RTSP / camera input for laptop (set to your Radxa RTSP or local camera index)
RTSP_URL = "rtsp://YOUR_VPS_IP:8554/drone_feed"  # set to actual stream or "0" for local webcam.

# Performance config
AI_CALL_INTERVAL = 4.0   # seconds between full AI cloud calls for same job
FRAME_SKIP = 2           # process every Nth frame for vision (increase to save CPU)

# artifact dir
TEMP_ARTIFACT_DIR = "./artifacts"
MAX_IMAGES_UPLOAD = 25
MAX_VIDEO_UPLOAD_MB = 100
