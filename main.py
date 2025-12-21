from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import json
import os
import uuid

# --- ROUTER IMPORTS ---
from ai_router import ai_router
from memory_router import memory_router
from plan_router import plan_router
from ws_router import ws_router
from ai_command_router import router as ai_command_router

# --- Define Models Inline (Auth) ---
class LoginRequest(BaseModel):
    email: str
    password: str

class RegisterRequest(BaseModel):
    username: str
    email: str
    password: str

# --- App Setup ---
app = FastAPI(title="AI Drone Server (Unified)", version="3.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Database Helper ---
DB_FILE = "users.json"

def load_db():
    if not os.path.exists(DB_FILE):
        return {}
    try:
        with open(DB_FILE, 'r') as f:
            return json.load(f)
    except:
        return {}

def save_db(db):
    with open(DB_FILE, 'w') as f:
        json.dump(db, f)

# --- REGISTER ROUTERS ---
# 1. AI & Drone Control
app.include_router(ai_router, prefix="/ai")
app.include_router(memory_router, prefix="/memory")
app.include_router(plan_router, prefix="/plan")
app.include_router(ws_router, prefix="/ws")
app.include_router(ai_command_router)

# 2. Auth Routes (Inline)
@app.post("/session/register")
async def register_inline(req: RegisterRequest):
    print(f"REGISTER REQUEST: {req.email}")
    db = load_db()
    if req.email in db:
        raise HTTPException(status_code=400, detail="User already exists")
    
    token = str(uuid.uuid4())
    db[req.email] = {"username": req.username, "password": req.password, "token": token}
    save_db(db)
    
    return {"status": "success", "token": token, "user": {"username": req.username, "email": req.email, "id": token}}

@app.post("/session/create")
async def login_inline(req: LoginRequest):
    print(f"LOGIN REQUEST: {req.email}")
    db = load_db()
    
    # DEBUG PRINT FOR 401 ERROR
    user = db.get(req.email)
    
    if not user:
        print(f"DEBUG: User {req.email} NOT FOUND in DB.")
        raise HTTPException(status_code=404, detail="User not found")
        
    print(f"DEBUG: Checking Password. Received='{req.password}' vs Stored='{user.get('password')}'")
    
    if user["password"] != req.password:
        print("DEBUG: Password MISMATCH!")
        raise HTTPException(status_code=401, detail="Invalid password")
    
    return {"status": "success", "token": user["token"], "user": {"username": user["username"], "email": req.email, "id": user["token"]}}

# --- Startup Event ---
@app.on_event("startup")
async def startup_event():
    print("\n\n" + "="*50)
    print("--> SERVER STARTUP: AUTH + AI ROUTERS LOADED <--")
    print("--> /session/create [READY]")
    print("--> /ai/command     [READY]")
    print("--> /ws/connect     [READY]")
    print("="*50 + "\n\n")

@app.get("/")
def root():
    return {"status": "AI Drone Server Running", "auth": "enabled_inline", "ai": "active"}