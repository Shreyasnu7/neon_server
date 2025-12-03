# server.py
# FastAPI drone-relay backend: auth, drones, logs, telemetry, ws relay to drones
import uvicorn
import uuid
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Dict, List, Any
import asyncio
import json
from pathlib import Path
from threading import Lock

STORAGE_FILE = Path("storage.json")
_db_lock = Lock()

# Default structures if no file exists
DEFAULT_DB = {
    "users": {},   # username -> {username, password_hash, drones: []}
    "drones": {},  # id -> {id, name, model}
    "logs": {}     # id -> [logs]
}

def load_storage():
    if STORAGE_FILE.exists():
        try:
            with STORAGE_FILE.open("r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            # If corrupt, fallback to defaults
            return DEFAULT_DB.copy()
    else:
        return DEFAULT_DB.copy()

def save_storage(db):
    # Use a lock to avoid concurrent writes
    with _db_lock:
        with STORAGE_FILE.open("w", encoding="utf-8") as f:
            json.dump(db, f, indent=2)

# load persisted storage at startup (will create storage.json with DEFAULT_DB if missing)
_storage = load_storage()
USERS = _storage.get("users", {})
DRONES = _storage.get("drones", {})
LOGS = _storage.get("logs", {})

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

import hashlib   # add this near the top with your imports if not already present

@app.post("/login")
async def login(payload: LoginIn):
    # simple auth: check hashed password
    user = USERS.get(payload.username)
    if not user:
        raise HTTPException(status_code=401, detail="Invalid credentials")

    password_hash = hashlib.sha256(payload.password.encode()).hexdigest()
    if password_hash != user.get("password_hash"):
        raise HTTPException(status_code=401, detail="Invalid credentials")

    token = str(uuid.uuid4())
    # store token and persist
    user["token"] = token
    _storage["users"] = USERS
    save_storage(_storage)

    return {"token": token}

@app.post("/drones")
async def add_drone(payload: RegisterDroneIn, token: str = ""):
    did = str(uuid.uuid4())
    DRONES[did] = {"id": did, "name": payload.name, "model": payload.model}
    LOGS[did] = []

    # Optionally attach to user if token -> username mapping exists
    # e.g. find username by token and append: USERS[username]["drones"].append(did)

    # persist
    _storage["drones"] = DRONES
    _storage["logs"] = LOGS
    save_storage(_storage)

    return {"id": did, "name": payload.name}

import hashlib

@app.post("/signup")
async def signup(data: LoginIn):
    username = data.username
    password = data.password

    # check if user exists
    if username in USERS:
        raise HTTPException(status_code=400, detail="User already exists")

    # store hashed password (don't store plaintext)
    password_hash = hashlib.sha256(password.encode()).hexdigest()

    USERS[username] = {
        "username": username,
        "password_hash": password_hash,
        "drones": []
    }

    # persist
    _storage["users"] = USERS
    save_storage(_storage)

    return {"ok": True, "message": "User created"}

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
