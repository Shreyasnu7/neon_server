# ws_router.py
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import json

router = APIRouter()

connected_clients = {} # Global State

@router.websocket("/connect/{client_id}")
async def connect_client(websocket: WebSocket, client_id: str):
    await websocket.accept()
    connected_clients[client_id] = websocket
    print(f"✅ WS CONNECTED: {client_id} (Total: {len(connected_clients)})")

    try:
        while True:
            msg = await websocket.receive_text()
            print(f"[{client_id}] -> {msg}")
            # Broadcast to all OTHER connected clients (e.g. Drone -> App)
            print(f"📣 Broadcast from {client_id} to {len(connected_clients)-1} clients")
            for cid, sock in connected_clients.items():
                if cid != client_id:
                    try:
                         await sock.send_text(msg)
                    except Exception as e:
                        print(f"❌ Broadcast Fail to {cid}: {e}")
                        pass # Handle stale sockets later 
    except WebSocketDisconnect:
        if client_id in connected_clients:
            del connected_clients[client_id]
        print(f"WS disconnected: {client_id}")
