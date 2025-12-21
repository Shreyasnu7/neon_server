from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
import json
import os
import uuid

router = APIRouter()
DB_FILE = "users.json"

# --- Models ---
class LoginRequest(BaseModel):
    email: str
    password: str

class RegisterRequest(BaseModel):
    username: str
    email: str
    password: str

# --- Helper Functions ---
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

# --- Routes ---
@router.post("/session/register")
async def register(req: RegisterRequest):
    db = load_db()
    
    # Check if user exists
    if req.email in db:
        raise HTTPException(status_code=400, detail="User already exists")
    
    # Create simple token
    token = str(uuid.uuid4())
    
    # Save user (In production, hash passwords!)
    db[req.email] = {
        "username": req.username,
        "password": req.password, 
        "token": token
    }
    save_db(db)
    
    return {
        "status": "success",
        "token": token,
        "user": {
            "username": req.username,
            "email": req.email,
            "id": token
        }
    }

@router.post("/session/create")
async def login(req: LoginRequest):
    db = load_db()
    user = db.get(req.email)
    
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
        
    if user["password"] != req.password:
        raise HTTPException(status_code=401, detail="Invalid password")
        
    return {
        "status": "success",
        "token": user["token"],
        "user": {
            "username": user["username"],
            "email": req.email,
            "id": user["token"]
        }
    }
