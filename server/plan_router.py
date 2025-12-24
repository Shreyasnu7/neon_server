# plan_router.py
import os, json, uuid
from fastapi import APIRouter
from api_schemas import AIPlan
from fastapi import APIRouter
from api_schemas import DronePlan

router = APIRouter()
PLAN_DIR = "/tmp/jobs"
os.makedirs(PLAN_DIR, exist_ok=True)

def save_plan_to_disk(plan_data: dict) -> str:
    plan_id = str(uuid.uuid4())
    path = f"{PLAN_DIR}/{plan_id}.json"
    with open(path, "w") as f:
        json.dump(plan_data, f)
    return plan_id

plan_router = APIRouter(tags=["Plans"])

@plan_router.post("/submit")
def submit_plan(plan: AIPlan):
    plan_id = save_plan_to_disk(plan.dict())
    return {"plan_id": plan_id}

@plan_router.get("/next")
def fetch_plan():
    files = sorted(os.listdir(PLAN_DIR))
    if not files:
        return {"plan": None}
    path = f"{PLAN_DIR}/{files[0]}"
    try:
        data = json.load(open(path))
        os.remove(path)
        return {"plan": data}
    except Exception:
        return {"plan": None}
