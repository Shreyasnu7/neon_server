
import json

def ask_ai(prompt: str, context: dict = None) -> dict:
    """
    Mock LLM implementation to mock cloud reasoning.
    In production, this would call OpenAI/Gemini API.
    """
    # Simple keyword spotting for testing
    prompt_lower = prompt.lower()
    
    action = "hover"
    target = None
    style = "cinematic"
    
    if "land" in prompt_lower:
        action = "land"
    elif "monitor" in prompt_lower or "track" in prompt_lower:
        action = "orbit"
        target = "detected_object"
    elif "takeoff" in prompt_lower:
        action = "takeoff"
    elif "scan" in prompt_lower:
        action = "scan_area"
        
    return {
        "action": action,
        "style": style,
        "target": target,
        "constraints": {"safety_distance": 2.0},
        "reasoning": "Mock AI logic processed request."
    }
