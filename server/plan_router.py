# plan_router.py
import os, json, uuid
from fastapi import APIRouter
from api_schemas import AIPlan
from fastapi import APIRouter
from api_schemas import DronePlan

router = APIRouter()

_PLAN_QUEUE = []


@router.post("/plan")
async def submit_plan(plan: DronePlan):
    _PLAN_QUEUE.append(plan)
    return {"status": "queued"}


@router.get("/plan/next")
async def get_next_plan():
    if not _PLAN_QUEUE:
        return {"plan": None}
    return {"plan": _PLAN_QUEUE.pop(0).dict()}

PLAN_DIR = "/tmp/jobs"
os.makedirs(PLAN_DIR, exist_ok=True)

plan_router = APIRouter(tags=["Plans"])

@plan_router.post("/submit")
def submit_plan(plan: AIPlan):
    plan_id = str(uuid.uuid4())
    path = f"{PLAN_DIR}/{plan_id}.json"
    json.dump(plan.dict(), open(path, "w"))
    return {"plan_id": plan_id}

@plan_router.get("/next")
def fetch_plan():
    files = sorted(os.listdir(PLAN_DIR))
    if not files:
        return {"plan": None}
    path = f"{PLAN_DIR}/{files[0]}"
    data = json.load(open(path))
    os.remove(path)
    return {"plan": data}
