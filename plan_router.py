# plan_router.py
import uuid
from fastapi import APIRouter
from api_schemas import DronePlan

router = APIRouter()

# Safe In-Memory Queue
_PLAN_QUEUE = []

@router.post("/plan")
async def submit_plan(plan: DronePlan):
    # Log logic here if needed
    _PLAN_QUEUE.append(plan)
    return {"status": "queued"}

@router.get("/plan/next")
async def get_next_plan():
    if not _PLAN_QUEUE:
        return {"plan": None}
    return {"plan": _PLAN_QUEUE.pop(0).dict()}

# Legacy /mnt access removed to prevent crashes on Render

