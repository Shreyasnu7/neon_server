# ws_router.py
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import json

ws_router = APIRouter()

@ws_router.websocket("/connect/{client_id}")
async def connect_client(websocket: WebSocket, client_id: str):
    await websocket.accept()
    print(f"WS connected: {client_id}")

    try:
        while True:
            msg = await websocket.receive_text()
            print(f"[{client_id}] -> {msg}")
            await websocket.send_text(json.dumps({"ack": msg}))  # JSON response
    except WebSocketDisconnect:
        print(f"WS disconnected: {client_id}")
