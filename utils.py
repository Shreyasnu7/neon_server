from passlib.context import CryptContext
import jwt
from datetime import datetime, timedelta

# ---- PASSWORD HASHING ----
pwd = CryptContext(schemes=["bcrypt"], deprecated="auto")

def hash_password(password: str) -> str:
    return pwd.hash(password)

def verify_password(password: str, hashed: str) -> bool:
    return pwd.verify(password, hashed)


# ---- JWT SETTINGS ----
SECRET = "MY_SUPER_SECRET_KEY"   # You can change this
ALGO = "HS256"
EXP_MINUTES = 60   # Token valid for 60 minutes


def create_jwt(user_id: int):
    expire = datetime.utcnow() + timedelta(minutes=EXP_MINUTES)
    payload = {"user_id": user_id, "exp": expire}
    return jwt.encode(payload, SECRET, algorithm=ALGO)


def verify_jwt(token: str):
    try:
        return jwt.decode(token, SECRET, algorithms=[ALGO])
    except:
        return None