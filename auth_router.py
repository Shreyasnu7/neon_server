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
# --- Routes ---
@router.post("/session/register")
async def register(req: RegisterRequest):
    print(f"--> REGISTER ATTEMPT: {req.email}")
    db = load_db()
    
    email_key = req.email.lower().strip()
    
    # Check if user exists
    if email_key in db:
        print(f"    FAILED: User {email_key} already exists")
        raise HTTPException(status_code=400, detail="User already exists")
    
    # Create simple token
    token = str(uuid.uuid4())
    
    # Sanitize Password
    safe_password = req.password.strip()
    
    # Save user (In production, hash passwords!)
    user_data = {
        "username": req.username,
        "password": safe_password, 
        "token": token
    }
    db[email_key] = user_data
    save_db(db)
    
    # DEBUG: print what we just saved
    print(f"    DEBUG SAVED: {repr(user_data)}")
    print(f"    SUCCESS: User {email_key} registered with token {token}")
    
    return {
        "status": "success",
        "token": token,
        "user": {
            "username": req.username,
            "email": req.email, # Return original casing if desired, or normalized
            "id": token
        }
    }

@router.post("/session/create")
async def login(req: LoginRequest):
    print(f"--> LOGIN ATTEMPT: {req.email}")
    db = load_db()
    
    email_key = req.email.lower().strip()
    user = db.get(email_key)
    
    if not user:
        print(f"    FAILED: User {email_key} not found in DB keys: {list(db.keys())}")
        raise HTTPException(status_code=404, detail="User not found")
    
    # DEBUG: Print stored vs received
    stored_pass = user.get("password", "")
    received_pass = req.password.strip() # Strip on login too
    
    print(f"    DEBUG LOGIN: Stored={repr(stored_pass)} vs Received={repr(received_pass)}")
    
    # Check for direct match
    if stored_pass != received_pass:
        print(f"    FAILED: Password mismatch for {email_key}")
        raise HTTPException(status_code=401, detail="Invalid password")
        
    print(f"    SUCCESS: User {email_key} logged in")
    return {
        "status": "success",
        "token": user["token"],
        "user": {
            "username": user["username"],
            "email": req.email,
            "id": user["token"]
        }
    }
