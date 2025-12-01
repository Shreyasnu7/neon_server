# server.py
# FastAPI drone-relay backend: auth, drones, logs, telemetry, ws relay to drones
import uvicorn
import uuid
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Dict, List, Any
import asyncio

app = FastAPI(title="Neon Drone Relay")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # tighten in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# In-memory stores (swap to DB later)
USERS: Dict[str, str] = {}              # username -> token
DRONES: Dict[str, Dict] = {}            # id -> {name, owner}
LOGS: Dict[str, List[Dict]] = {}        # droneId -> list of logs
TELEMETRY: Dict[str, Dict] = {}         # droneId -> last telemetry

# WebSocket manager for drones: device_id -> websocket
DRONE_WS: Dict[str, WebSocket] = {}

# ---------- Models ----------
class LoginIn(BaseModel):
    username: str
    password: str

class RegisterDroneIn(BaseModel):
    name: str
    model: str = ""

class CommandIn(BaseModel):
    device: str
    cmd: str
    params: Dict[str, Any] = {}

class FlightLogIn(BaseModel):
    device: str
    log: Dict[str, Any]

# ---------- Endpoints ----------
@app.get("/health")
async def health():
    return {"ok": True}

@app.post("/login")
async def login(payload: LoginIn):
    # Very simple auth: accept any username/password for now
    token = str(uuid.uuid4())
    USERS[payload.username] = token
    return {"token": token}

@app.post("/drones")
async def add_drone(payload: RegisterDroneIn, token: str = ""):
    did = str(uuid.uuid4())
    DRONES[did] = {"id": did, "name": payload.name, "model": payload.model}
    LOGS[did] = []
    return {"id": did, "name": payload.name}

@app.get("/drones")
async def list_drones():
    return list(DRONES.values())

@app.post("/logs")
async def upload_log(payload: FlightLogIn):
    d = payload.device
    if d not in LOGS:
        LOGS[d] = []
    LOGS[d].append(payload.log)
    return {"ok": True}

@app.get("/logs")
async def get_logs(device: str):
    return LOGS.get(device, [])

@app.post("/telemetry")
async def post_telemetry(device: str, t: Dict[str, Any]):
    TELEMETRY[device] = t
    return {"ok": True}

@app.get("/telemetry")
async def get_telemetry(device: str):
    return TELEMETRY.get(device, {})

# Send command to drone (relay via ws)
@app.post("/command")
async def send_command(c: CommandIn):
    ws = DRONE_WS.get(c.device)
    payload = {"cmd": c.cmd, "params": c.params}
    if ws:
        try:
            await ws.send_json(payload)
            return {"ok": True, "relayed": True}
        except Exception as e:
            return {"ok": False, "error": str(e)}
    else:
        # Drone not connected - store last command in memory (or queue)
        return {"ok": True, "relayed": False, "message": "drone offline"}

# ---------- WebSocket for drone clients ----------
@app.websocket("/ws/drone/{device_id}")
async def drone_ws(websocket: WebSocket, device_id: str):
    await websocket.accept()
    DRONE_WS[device_id] = websocket
    try:
        while True:
            data = await websocket.receive_json()
            # Expect telemetry messages {"telemetry": {...}} or keepalive
            if "telemetry" in data:
                TELEMETRY[device_id] = data["telemetry"]
            if "log" in data:
                LOGS.setdefault(device_id, []).append(data["log"])
    except WebSocketDisconnect:
        DRONE_WS.pop(device_id, None)

# ---------- Run ----------
if __name__ == "__main__":
    uvicorn.run("server:app", host="0.0.0.0", port=8000, reload=False)
