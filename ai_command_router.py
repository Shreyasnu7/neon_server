from fastapi import APIRouter
from cloud_ai.orchestrator import CloudOrchestrator
from plan_router import submit_plan
from api_schemas import DronePlan
# Mock LLM Client for now or import real one
class MockLLM:
    def chat(self, system, user):
        import json
        return json.dumps({
            "emotional_model": {"vector": {"awe": 0.9}, "peak_allowed": True},
            "camera_plan": {"shot_energy": 0.8}
        })

router = APIRouter(prefix="/director", tags=["AI-Command"])
# Initialize Orchestrator (Dependency Injection best practice in real app)
orchestrator = CloudOrchestrator(llm_client=MockLLM())

@router.post("/ai/command")
async def ai_command(payload: dict):
    """
    Structured Pipeline: 
    Input -> Orchestrator -> Plan -> Queue
    """
    # 1. Delegate to Orchestrator
    plan_result = await orchestrator.process_request(payload)
    
    # 2. Ensure it matches DronePlan schema
    # (Assuming PlanGenerator returns a DronePlan object or dict)
    if isinstance(plan_result, dict):
        plan = DronePlan(**plan_result)
    else:
        plan = plan_result

    # 3. Push to Execution Queue
    await submit_plan(plan)

    return {
        "status": "queued",
        "plan": plan.dict()
    }