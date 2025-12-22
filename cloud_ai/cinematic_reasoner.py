
class CinematicReasoner:
    """
    Refines the raw intent with cinematic rules.
    """
    def refine(self, intent: dict) -> dict:
        # Add basic polish if missing
        if "camera_plan" not in intent:
             intent["camera_plan"] = {}
        return intent