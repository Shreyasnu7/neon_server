import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security import OAuth2PasswordBearer
import jwt
import time

app = FastAPI()

# CORS FOR MOBILE APP
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

SECRET = "NEONLINKSECRET123"   # change later
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

users = {}          # { username: password }
drones = {}         # { drone_name: owner_username }
client_sockets = {} # { drone_id: WebSocket }
drone_sockets = {}  # { drone_id: WebSocket }


def make_token(username):
    return jwt.encode(
        {"user": username, "exp": time.time() + 86400},
        SECRET,
        algorithm="HS256",
    )


def require_token(token: str = Depends(oauth2_scheme)):
    try:
        data = jwt.decode(token, SECRET, algorithms=["HS256"])
        return data["user"]
    except:
        raise HTTPException(401, "Invalid or expired token")


@app.post("/signup")
async def signup(data: dict):
    u = data.get("username")
    p = data.get("password")

    if not u or not p:
        return {"error": "Missing fields"}

    if u in users:
        return {"error": "User exists"}

    users[u] = p
    return {"success": True}


@app.post("/login")
async def login(data: dict):
    u = data.get("username")
    p = data.get("password")

    if users.get(u) != p:
        return {"error": "Invalid login"}

    return {"token": make_token(u)}


@app.post("/register_drone")
async def register_drone(data: dict, user=Depends(require_token)):
    name = data.get("name")
    if not name:
        return {"error": "Missing drone name"}

    drones[name] = user
    return {"success": True, "drone_id": name}


# ===============================
# CLIENT WEBSOCKET (PHONE)
# ===============================
@app.websocket("/ws/client/{drone_id}")
async def client_socket(ws: WebSocket, drone_id: str):
    await ws.accept()
    client_sockets[drone_id] = ws

    try:
        while True:
            msg = await ws.receive_text()
            if drone_id in drone_sockets:
                await drone_sockets[drone_id].send_text(msg)

    except WebSocketDisconnect:
        client_sockets.pop(drone_id, None)


# ===============================
# DRONE WEBSOCKET (ESP32/RADXA)
# ===============================
@app.websocket("/ws/drone/{drone_id}")
async def drone_socket(ws: WebSocket, drone_id: str):
    await ws.accept()
    drone_sockets[drone_id] = ws

    try:
        while True:
            msg = await ws.receive_text()
            if drone_id in client_sockets:
                await client_sockets[drone_id].send_text(msg)

    except WebSocketDisconnect:
        drone_sockets.pop(drone_id, None)


# ===============================
# AI COMMAND PARSER (GPT-LIKE)
# ===============================
@app.post("/ai/parse")
async def ai_parse(data: dict, user=Depends(require_token)):
    text = data["text"].lower()

    # DUMMY PARSER (replace with real GPT later)
    if "follow" in text:
        return {"cmd": "FOLLOW_TARGET"}

    if "orbit" in text:
        return {"cmd": "ORBIT", "radius": 20}

    if "go" in text and "home" in text:
        return {"cmd": "RTH"}

    return {"cmd": "UNKNOWN"}
    

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000)
