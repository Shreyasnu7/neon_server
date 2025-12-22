
class IntentBuilder:
    """
    Validates and constructs the definitive Shot Intent.
    """
    def build_base_intent(self, raw_intent: dict) -> dict:
        # Pass-through validation for now
        if not isinstance(raw_intent, dict):
            raise ValueError("Intent must be a dictionary")
        return raw_intent