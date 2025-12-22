
from typing import Dict, Any, Optional

class SessionState:
    """
    Holds the runtime state of a Drone Session.
    This is what the Orchestrator updates when it receives telemetry.
    """
    def __init__(self, drone_id: str = "unknown"):
        self.drone_id: str = drone_id
        self.connected_since: float = 0
        self.last_seen: float = 0
        
        # Telemetry
        self.last_known_position: Dict[str, float] = {"lat": 0.0, "lon": 0.0, "alt": 0.0}
        self.last_health_status: Dict[str, Any] = {"battery": 0, "rssi": 0}
        
        # Context
        self.metadata: Dict[str, Any] = {}
        self.active_plan_id: Optional[str] = None
        
    def update_telemetry(self, t: dict):
        if "lat" in t:
             self.last_known_position = {
                 "lat": t.get("lat", 0),
                 "lon": t.get("lon", 0),
                 "alt": t.get("alt", 0)
             }
        if "battery" in t:
             self.last_health_status = {
                 "battery": t.get("battery", 0),
                 "rssi": t.get("rssi", 0)
             }
        self.last_seen = t.get("ts", 0)
