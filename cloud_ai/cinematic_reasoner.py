# server/cloud_ai/cinematic_reasoner.py

class CinematicReasoner:
    """
    Applies cinematic intelligence:
    - pacing
    - emotional shaping
    - peak suppression
    """

    def refine(self, intent: dict) -> dict:
        # Robust access to emotional vector
        try:
             emotion = intent.get("emotional_model", {}).get("vector", {"neutral": 1.0})
             peak_allowed = intent.get("emotional_model", {}).get("peak_allowed", True)
        except:
             emotion = {"neutral": 1.0}
             peak_allowed = True

        # Prevent early climax
        if not peak_allowed:
            emotion = {
                k: min(v, 0.85) for k, v in emotion.items()
            }

        # Shape shot energy based on emotion
        dominant = max(emotion, key=emotion.get)

        if dominant in ("awe", "grief"):
            intent["camera_plan"]["shot_energy"] *= 0.85
        elif dominant in ("tension", "urgency"):
            intent["camera_plan"]["shot_energy"] *= 1.1

        # Clamp values
        intent["camera_plan"]["shot_energy"] = max(
            0.1, min(intent["camera_plan"]["shot_energy"], 1.0)
        )

        return intent