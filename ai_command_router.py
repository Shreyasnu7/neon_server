from cloud_ai.dependencies import get_orchestrator
from api_schemas import DronePlan
from fastapi import APIRouter
from plan_router import submit_plan

router = APIRouter(prefix="/director", tags=["AI-Command"])

@router.post("/ai/command")
async def ai_command(payload: dict):
    # Use Singleton Orchestrator
    orchestrator = get_orchestrator()
    """
    Structured Pipeline: 
    Input -> Orchestrator -> Plan -> Queue
    Payload expects:
    {
        "text": "chase car",
        "provider": "gemini" | "openai",
        "camera": "internal" | "external",
        "sensitivity": 0.5,
        "media": [base64_string] (optional)
    }
    """
    # 1. Extract Config
    config = {
        "provider": payload.get("provider", "gemini"),
        "camera": payload.get("camera", "external"),
        "sensitivity": payload.get("sensitivity", 0.5)
    }
    
    # 2. Delegate to Orchestrator (Pass Full Payload)
    # The Orchestrator will use 'provider' to select the backend if implemented there,
    # or we set it on the LLM client here if it was stateful (it's stateless, so we assume client handles env vars or we pass context)
    
    # We pass the config into the context for the Orchestrator/LLM
    # process_multimodal_request(text, user_id, drone_id, images=None, video=None)
    
    # Extract text from payload or default
    text_prompt = payload.get("text", "Execute autonomous behavior")
    
    # 1. Extract Keys (BYOK Support)
    # The app sends "api_keys": {"gemini": "...", "openai": "..."}
    client_keys = payload.get("api_keys", {})
    
    # Fallback for legacy calls (singular key)
    if not client_keys:
        direct_key = (
            payload.get("api_key") or 
            payload.get("apiKey") or 
            payload.get("key") or 
            payload.get("gemini_api_key")
        )
        if direct_key:
            client_keys = {"gemini": direct_key}

    # DEBUG PAYLOAD KEYS (Security: Don't print values)
    print(f"DEBUG: /director/ai/command Payload Keys: {list(payload.keys())}")
    if client_keys:
        safe_log = {k: f"{v[:8]}..." if v else "EMPTY" for k, v in client_keys.items()}
        print(f"DEBUG: 🔑 KEYS RECEIVED: {safe_log}")
    else:
        print("DEBUG: ⚠️ NO API KEYS IN PAYLOAD")
    else:
        print("DEBUG: NO API KEY FOUND IN PAYLOAD")

    # Pass api_keys dict to orchestrator
    plan_result = await orchestrator.process_multimodal_request(
        text=text_prompt,
        user_id="default_user", 
        drone_id="default_drone",
        images=payload.get("media"), 
        brain_context=config,
        api_keys=client_keys # <--- Correct Argument
    )
    
    # 3. Ensure it matches DronePlan schema
    if isinstance(plan_result, dict):
        plan = DronePlan(**plan_result)
    else:
        plan = plan_result

    # 4. Push to Execution Queue & Broadcast to Laptop (via websocket usually, or laptop polls this)
    # Ideally, Laptop AI listens to 'plans'.
    submit_plan(plan)

    return {
        "status": "queued",
        "plan": plan.dict(),
        "used_config": config,
        "message": plan.reasoning or "Command queued." # Frontend will display this
    }