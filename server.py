# --------------------------------------------
# Neon Drone Relay - SIMPLE WORKING SERVER
# --------------------------------------------
import uvicorn
import hashlib
import uuid
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Dict, List, Any
import json
from pathlib import Path
from threading import Lock

# ---------------- STORAGE ----------------
STORAGE_FILE = Path("storage.json")
_lock = Lock()

DEFAULT_DB = {
    "users": {},       # username -> {username, password_hash}
    "drones": {},      # droneId -> {id, name, model}
    "logs": {},        # droneId -> [logs]
    "telemetry": {}    # droneId -> last telemetry
}


def load_db():
    if STORAGE_FILE.exists():
        try:
            with STORAGE_FILE.open("r", encoding="utf-8") as f:
                return json.load(f)
        except:
            return DEFAULT_DB.copy()
    else:
        return DEFAULT_DB.copy()


def save_db(db):
    with _lock:
        with STORAGE_FILE.open("w", encoding="utf-8") as f:
            json.dump(db, f, indent=2)


db = load_db()

# ---------------- MODELS ----------------
class UserIn(BaseModel):
    username: str
    password: str


class LoginIn(BaseModel):
    username: str
    password: str


class RegisterDroneIn(BaseModel):
    name: str
    model: str = ""


class FlightLogIn(BaseModel):
    device: str
    log: Dict[str, Any]


class CommandIn(BaseModel):
    device: str
    cmd: str
    params: Dict[str, Any] = {}


# ---------------- FASTAPI APP ----------------
app = FastAPI(title="Neon Drone Relay")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

DRONE_WS: Dict[str, WebSocket] = {}  # deviceId -> websocket


# ---------------- AUTH ----------------
@app.post("/signup")
async def signup(user: UserIn):
    if user.username in db["users"]:
        raise HTTPException(status_code=400, detail="User already exists")

    hashed = hashlib.sha256(user.password.encode()).hexdigest()

    db["users"][user.username] = {
        "username": user.username,
        "password": hashed
    }

    save_db(db)
    return {"ok": True}


@app.post("/login")
async def login(data: LoginIn):
    user = db["users"].get(data.username)

    if not user:
        raise HTTPException(status_code=401, detail="User not found")

    incoming_hash = hashlib.sha256(data.password.encode()).hexdigest()

    if incoming_hash != user["password"]:
        raise HTTPException(status_code=401, detail="Invalid password")

    token = f"{data.username}-valid-token"
    return {"token": token}


# ---------------- DRONES ----------------
@app.post("/drones")
async def add_drone(info: RegisterDroneIn):
    drone_id = str(uuid.uuid4())

    db["drones"][drone_id] = {
        "id": drone_id,
        "name": info.name,
        "model": info.model,
    }
    db["logs"][drone_id] = []

    save_db(db)
    return {"id": drone_id, "name": info.name}


@app.get("/drones")
async def list_drones():
    return list(db["drones"].values())


# ---------------- LOGGING ----------------
@app.post("/logs")
async def upload_log(data: FlightLogIn):
    dev = data.device
    if dev not in db["logs"]:
        db["logs"][dev] = []

    db["logs"][dev].append(data.log)
    save_db(db)
    return {"ok": True}


@app.get("/logs")
async def get_logs(device: str):
    return db["logs"].get(device, [])


# ---------------- TELEMETRY ----------------
@app.post("/telemetry")
async def post_telemetry(device: str, t: Dict[str, Any]):
    db["telemetry"][device] = t
    save_db(db)
    return {"ok": True}


@app.get("/telemetry")
async def get_telemetry(device: str):
    return db["telemetry"].get(device, {})


# ---------------- COMMAND ----------------
@app.post("/command")
async def send_command(c: CommandIn):
    ws = DRONE_WS.get(c.device)

    if ws:
        await ws.send_json({"cmd": c.cmd, "params": c.params})
        return {"ok": True, "relayed": True}
    else:
        return {"ok": False, "relayed": False, "message": "Drone offline"}


# ---------------- WEBSOCKET ----------------
@app.websocket("/ws/drone/{device}")
async def drone_ws(websocket: WebSocket, device: str):
    await websocket.accept()
    DRONE_WS[device] = websocket

    try:
        while True:
            msg = await websocket.receive_json()
            if "telemetry" in msg:
                db["telemetry"][device] = msg["telemetry"]
                save_db(db)
            if "log" in msg:
                db["logs"].setdefault(device, []).append(msg["log"])
                save_db(db)

    except WebSocketDisconnect:
        DRONE_WS.pop(device, None)


# ---------------- RUN LOCAL ----------------
if __name__ == "__main__":
    uvicorn.run("server:app", host="0.0.0.0", port=8000)
