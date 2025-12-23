# server/main.py

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from ai_router import ai_router
from memory_router import memory_router
from plan_router import plan_router
from ws_router import ws_router


from ai_command_router import router as ai_command_router


app = FastAPI(
    title="AI Drone Server (V3)",
    version="1.0"
)

# Middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Routers
app.include_router(ai_router, prefix="/ai")
app.include_router(memory_router, prefix="/memory")
app.include_router(plan_router, prefix="/plan")
app.include_router(ws_router, prefix="/ws")
app.include_router(ai_command_router)

# Health check
@app.get("/")
def root():
    return {
        "status": "AI Drone Server Running",
        "version": "1.0"
    }