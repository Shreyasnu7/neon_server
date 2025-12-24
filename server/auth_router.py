from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
import uuid

auth_router = APIRouter(tags=["Auth"])

class AuthRequest(BaseModel):
    username: str = None
    email: str = None
    password: str

# In-memory user store for demo (since we have no DB on free tier)
# For persistence, we could use the /tmp/memory trick, but simple is better for now.
# Or just allow any login.
USERS = {}

@auth_router.post("/register")
def register(req: AuthRequest):
    # Mock registration - Accept all
    token = f"tok_{uuid.uuid4()}"
    user_id = req.username or req.email or "pilot"
    return {"token": token, "user": {"id": user_id, "username": user_id}}

@auth_router.post("/create")
def login(req: AuthRequest):
    # Mock login - Accept all
    token = f"tok_{uuid.uuid4()}"
    user_id = req.email or req.username or "pilot"
    return {"token": token, "user": {"id": user_id, "username": user_id}}
