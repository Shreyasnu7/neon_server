# memory_router.py
import os, json
from fastapi import APIRouter
from api_schemas import MemoryWrite, MemoryRead

MEMORY_DIR = "/mnt/data/memory"
os.makedirs(MEMORY_DIR, exist_ok=True)

memory_router = APIRouter(tags=["Memory"])

def mem_path(user, drone):
    return f"{MEMORY_DIR}/{user}_{drone}.json"

@memory_router.post("/write")
def write_memory(req: MemoryWrite):
    path = mem_path(req.user_id, req.drone_id)
    mem = {}
    if os.path.exists(path):
        mem = json.load(open(path))
    mem[req.key] = req.value
    json.dump(mem, open(path, "w"))
    return {"status": "stored"}

@memory_router.post("/read")
def read_memory(req: MemoryRead):
    path = mem_path(req.user_id, req.drone_id)
    if not os.path.exists(path):
        return {}
    mem = json.load(open(path))
    if req.key:
        return {req.key: mem.get(req.key)}
    return mem
