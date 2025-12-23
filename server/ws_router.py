# ws_router.py
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import json
from typing import List, Dict

ws_router = APIRouter()

class ConnectionManager:
    def __init__(self):
        # We can store by ID to target specific drones
        self.active_connections: Dict[str, WebSocket] = {}

    async def connect(self, websocket: WebSocket, client_id: str):
        await websocket.accept()
        self.active_connections[client_id] = websocket
        print(f"WS Connected: {client_id}")

    def disconnect(self, client_id: str):
        if client_id in self.active_connections:
            del self.active_connections[client_id]
        print(f"WS Disconnected: {client_id}")

    async def broadcast(self, message: str, sender_id: str):
        """Send message to everyone EXCEPT sender (e.g. App -> Drone)"""
        for cid, connection in self.active_connections.items():
            if cid != sender_id:
                try:
                    await connection.send_text(message)
                except Exception as e:
                    print(f"Broadcast error to {cid}: {e}")

manager = ConnectionManager()

@ws_router.websocket("/connect/{client_id}")
async def connect_client(websocket: WebSocket, client_id: str):
    await manager.connect(websocket, client_id)
    try:
        while True:
            data = await websocket.receive_text()
            print(f"[{client_id}] -> Broadcasting: {data[:50]}...")
            # Automatically route to others (App -> Drone)
            await manager.broadcast(data, sender_id=client_id)
            
    except WebSocketDisconnect:
        manager.disconnect(client_id)

