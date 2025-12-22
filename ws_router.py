# ws_router.py
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import json
from typing import Dict, List

ws_router = APIRouter()

class ConnectionManager:
    def __init__(self):
        # Store active connections
        self.mobile_clients: List[WebSocket] = []
        self.drone_client: WebSocket | None = None

    async def connect_mobile(self, websocket: WebSocket):
        await websocket.accept()
        self.mobile_clients.append(websocket)
        print(f"📱 Mobile Client Connected. Total: {len(self.mobile_clients)}")

    async def connect_drone(self, websocket: WebSocket):
        await websocket.accept()
        self.drone_client = websocket
        print("🚁 Drone (Laptop) Connected!")

    def disconnect_mobile(self, websocket: WebSocket):
        if websocket in self.mobile_clients:
            self.mobile_clients.remove(websocket)
            print("📱 Mobile Client Disconnected")

    def disconnect_drone(self):
        self.drone_client = None
        print("🚁 Drone Disconnected")

    async def broadcast_to_mobile(self, message: str):
        """Relay message from Drone to ALL Mobile Clients"""
        for connection in self.mobile_clients:
            try:
                await connection.send_text(message)
            except Exception as e:
                print(f"Error broadcasting to mobile: {e}")

    async def send_to_drone(self, message: str):
        """Relay message from Mobile to Drone"""
        if self.drone_client:
            try:
                await self.drone_client.send_text(message)
            except Exception as e:
                print(f"Error sending to drone: {e}")

manager = ConnectionManager()

from cloud_ai.dependencies import get_orchestrator

@ws_router.websocket("/connect/{client_id}")
async def websocket_endpoint(websocket: WebSocket, client_id: str):
    print(f"🔌 INCOMING WS CONNECTION: {client_id}")
    """
    Main Relay Logic:
    - If client_id == 'laptop_vision', it's the Drone.
    - If client_id == 'mobile_app', it's the App.
    """
    is_drone = (client_id == "laptop_vision")
    orchestrator = get_orchestrator()
    
    if is_drone:
        await manager.connect_drone(websocket)
        # REGISTER DRONE WITH BRAIN (Sync call)
        orchestrator.dispatcher.register_drone_connection(websocket)
        print("✅ Drone registered with Enterprise Brain")
    else:
        await manager.connect_mobile(websocket)

    try:
        while True:
            data = await websocket.receive_text()
            
            # ROUTING LOGIC
            if is_drone:
                # 1. Drone -> Brain (Telemetry/Video)
                # Parse packet
                try:
                    packet = json.loads(data)
                    # Feed brain context if present
                    if "brain_context" in packet or "telemetry" in packet:
                        # 1a. FEED THE BRAIN
                        # This allows the Orchestrator to update SessionState (position, battery, status)
                        orchestrator.monitor_telemetry(packet)
                except: pass

                # 2. Drone -> App (Pass-through)
                await manager.broadcast_to_mobile(data)
                
            else:
                # 3. App -> Drone (Commands)
                await manager.send_to_drone(data)
                
    except WebSocketDisconnect:
        if is_drone:
            manager.disconnect_drone()
            orchestrator.dispatcher.remove_drone_connection()
        else:
            manager.disconnect_mobile(websocket)
    except Exception as e:
        print(f"WS Error [{client_id}]: {e}")
        if is_drone:
            manager.disconnect_drone()
        else:
            manager.disconnect_mobile(websocket)
