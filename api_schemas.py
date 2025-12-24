# api_schemas.py
# server/api_schemas.py

from pydantic import BaseModel, Field
from typing import Optional, Dict, Any


class GoProSettings(BaseModel):
    resolution: str = "5.3K"
    framerate: int = 60
    shutter: str = "auto" # "1/120", "auto"
    iso_max: int = 1600
    color_profile: str = "flat" # "flat", "natural", "vibrant"
    sharpness: str = "low"
    lens: str = "linear-horizon-level" # "wide", "superview", "linear"

class GimbalSettings(BaseModel):
    mode: str = "follow" # "fpv", "lock", "follow"
    pitch_angle: float = 0.0

class PostProcessSettings(BaseModel):
    lut_style: str = "ACES_Standard" # "ACES_Standard", "Vintage_Film", "Matrix_Green"
    global_tone_curve: Dict[str, float] = {
        "contrast_stretch": 1.0, 
        "highlight_rolloff": 0.8, # Controls clipped highlights
        "shadow_toe": 0.15 # Controls crushed blacks
    }
    super_resolution: bool = False # Use the SuperRes files

class DronePlan(BaseModel):
    action: str
    style: Optional[str] = "neutral"
    target: Optional[str] = None
    constraints: Optional[Dict[str, Any]] = {}
    reasoning: Optional[str] = "Command processed."
    
    # Hollywood Additions
    camera_config: Optional[GoProSettings] = None
    gimbal_config: Optional[GimbalSettings] = None
    post_process: Optional[PostProcessSettings] = None
    flight_behavior: Optional[str] = "standard" # "cinematic-smooth", "fpv-aggressive"

MAX_IMAGE_UPLOAD = 25
MAX_VIDEO_SIZE_MB = 100

class MultimodalRequest(BaseModel):
    user_id: str
    drone_id: str
    text: str
    include_vision: bool = False
    images: Optional[list[str]] = [] # Base64 or URLs
    video: Optional[str] = None      # URL or path
    api_keys: Optional[Dict[str, str]] = {} # User provided keys (BYOK)

# Backward compatibility alias
TextRequest = MultimodalRequest

class VideoLinkRequest(BaseModel):
    user_id: str
    drone_id: str
    url: str

class AIPlan(BaseModel):
    primitive: dict
    confidence: float = Field(default=1.0)
    vision_used: bool = False
    memory_used: bool = False

class MemoryWrite(BaseModel):
    user_id: str
    drone_id: str
    key: str
    value: str

class MemoryRead(BaseModel):
    user_id: str
    drone_id: str
    key: Optional[str] = None
