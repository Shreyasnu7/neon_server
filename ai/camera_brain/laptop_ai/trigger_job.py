import asyncio
import websockets
import json
import time

VPS_WS = "wss://drone-server-r0qe.onrender.com/ws/connect/user_manual_trigger"

async def trigger():
    print(f"Connecting to {VPS_WS}...")
    async with websockets.connect(VPS_WS) as ws:
        print("Connected.")
        
        # Payload matching director_core.py expectations
        job = {
            "type": "ai_job",
            "job_id": f"trigger_{int(time.time())}",
            "user_id": "user_manual",
            "drone_id": "drone_alpha",
            "text": "Find the person in the red shirt and orbit them technically",
            "images": [],
            "video": None
        }
        
        print(f"Sending Job: {json.dumps(job, indent=2)}")
        await ws.send(json.dumps(job))
        print("Job sent!")
        
        # Wait a bit to see if we get an ACK (though router is echo, so we might just get our own message back)
        try:
            response = await asyncio.wait_for(ws.recv(), timeout=2.0)
            print(f"Received echo/response: {response}")
        except asyncio.TimeoutError:
            print("No immediate response (normal if just broadcasting)")

if __name__ == "__main__":
    asyncio.run(trigger())
