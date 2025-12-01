# radxa_agent.py
# Connect to server websocket, send telemetry and receive commands.
import asyncio
import websockets
import json
import uuid
import time
import argparse
import psutil
import platform
from pathlib import Path

SERVER = "ws://<SERVER_IP>:8000"  # replace with your server ip or hostname
DEVICE_ID = "raxda-" + uuid.uuid4().hex[:8]
AUTH_TOKEN = ""  # optional

async def send_telemetry(ws):
    while True:
        tel = {
            "device": DEVICE_ID,
            "telemetry": {
                "time": time.time(),
                "cpu": psutil.cpu_percent(),
                "mem": psutil.virtual_memory().percent,
                "battery": None,
                "gps": {"lat": None, "lng": None},
            }
        }
        try:
            await ws.send(json.dumps(tel))
        except:
            break
        await asyncio.sleep(1.0)

async def handle_command(cmd):
    # cmd example: {"cmd":"takeoff","params":{"alt":10}}
    print("COMMAND RECEIVED:", cmd)
    # Hook: map to your flight controller/esp32 interface
    # Example: spawn a UDP packet to FC or use pymavlink to send MAV_CMD.
    # Placeholder: just log
    if cmd.get("cmd") == "takeoff":
        print(" -> executing TAKEOFF to alt", cmd.get("params"))
    elif cmd.get("cmd") == "rth":
        print(" -> executing RTH")
    elif cmd.get("cmd") == "rc":
        # rc contains pitch/roll/yaw/throttle
        vals = cmd.get("params", {})
        print(" -> RC", vals)
    # Add hooks here to call local AI modules (object tracking, exposure)
    return

async def ws_loop():
    uri = f"{SERVER}/ws/drone/{DEVICE_ID}"
    print("Connecting to", uri)
    while True:
        try:
            async with websockets.connect(uri, ping_interval=10) as ws:
                print("Connected to server")
                # start telemetry sender
                telemetry_task = asyncio.create_task(send_telemetry(ws))
                while True:
                    msg = await ws.recv()
                    try:
                        data = json.loads(msg)
                    except:
                        continue
                    # data contains {"cmd":"...", "params":{...}}
                    await handle_command(data)
        except Exception as e:
            print("WS connection failed:", e)
            await asyncio.sleep(2.0)

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        SERVER = sys.argv[1]
    # Replace SERVER placeholder if needed
    SERVER = SERVER.replace("<SERVER_IP>", "127.0.0.1")  # change to real server
    asyncio.run(ws_loop())
