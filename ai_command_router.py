from fastapi import APIRouter
from cloud_ai.orchestrator import CloudOrchestrator
from plan_router import submit_plan
from api_schemas import DronePlan
from cloud_ai.llm import RealLLMClient

router = APIRouter(prefix="/director", tags=["AI-Command"])
# Initialize Orchestrator with Real AI
orchestrator = CloudOrchestrator(llm_client=RealLLMClient())

@router.post("/ai/command")
async def ai_command(payload: dict):
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
    plan_result = await orchestrator.process_request(payload)
    
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