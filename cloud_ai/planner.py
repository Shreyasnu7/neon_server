from typing import Dict, Any
import uuid


class ExecutionPlanner:
    """
    Converts ShotIntent into a high-level execution plan.
    """

    def create_plan(
        self,
        shot_intent: Dict[str, Any],
    ) -> Dict[str, Any]:
        """
        Output is descriptive, not prescriptive.
        """

        return {
            "plan_id": str(uuid.uuid4()),
            "intent_signature": shot_intent,
            # DEEP LOGIC: We translate "Orbit" and "Follow" into actual metadata
            # The Laptop/Radxa will convert these into raw MAVLink points.
            "maneuver": self._derive_maneuver(shot_intent),
            "stages": [
                {
                    "stage": "approach",
                    "constraints": {
                        "motion_energy": shot_intent.get("motion_energy", 0.5),
                        "risk_tolerance": shot_intent.get("risk_tolerance", 0.1),
                    },
                },
                {
                    "stage": "primary_capture",
                    "constraints": {
                        "camera_presence": shot_intent.get("camera_presence", 0.5),
                        "imperfection_tolerance": shot_intent.get("imperfection_tolerance", 0.2),
                    },
                }
            ],
        }

    def _derive_maneuver(self, intent):
        """
        Translates Abstract Intent -> Concrete Maneuver Primitive
        """
        stype = intent.get("shot_type", "static")
        
        if "orbit" in stype:
            return {
                "type": "ORBIT",
                "radius": intent.get("zoom", 1.0) * 5.0, # zoom correlates to distance
                "speed": intent.get("motion_energy", 0.5) * 2.0
            }
        elif "follow" in stype or "track" in stype:
            return {
                "type": "FOLLOW",
                "offset": {"x": -2.0, "y": 0, "z": 1.5}, # Standard framing
                "aggressiveness": intent.get("forward_motion", 0.5)
            }
        else:
             return {
                "type": "HOVER",
                "look_at_subject": True
             }