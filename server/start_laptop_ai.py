import asyncio
import cv2
import json
import time
import aiohttp
from ultralytics import YOLO

# --- CONFIGURATION ---
# Use wss:// for secure websocket
SERVER_URL = "wss://web-production-fdccc.up.railway.app/ws/connect/laptop_vision"
MODEL_PATH = "yolov8n.pt"
# ---------------------

model = None

async def run_vision_loop(websocket):
    """
    Main loop: Capture WebCam -> YOLO -> Telemetry -> Websocket
    """
    cap = cv2.VideoCapture(0) # 0 for Webcam
    print("👀 Vision System Active - Looking for objects...")
    
    last_send = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # 1. Run AI Inference
        results = model(frame, stream=True, verbose=False)
        
        detections = []
        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = model.names[cls_id]
                conf = float(box.conf[0])
                
                if conf > 0.5:
                    detections.append({
                        "class": cls_name,
                        "confidence": round(conf, 2),
                        "bbox": box.xyxy[0].tolist()
                    })
                    
            im_array = r.plot()
            cv2.imshow("Laptop AI Brain", im_array)

        # 2. Send Insight to Cloud (Rate Limited to 5Hz to save bandwidth)
        now = time.time()
        if detections and (now - last_send > 0.2):
            payload = {
                "type": "vision_update",
                "detections": detections,
                "timestamp": now
            }
            try:
                await websocket.send_str(json.dumps(payload))
                last_send = now
            except Exception as e:
                print(f"Send Error: {e}")
                break # Break inner loop to reconnect

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
        await asyncio.sleep(0.01)

    cap.release()
    cv2.destroyAllWindows()

async def listen_for_commands(websocket):
    """
    Listen for messages from Server (Orchestrator Commands)
    """
    async for msg in websocket:
        if msg.type == aiohttp.WSMsgType.TEXT:
            data = msg.data
            print(f"📩 Received: {data}")
            # Handle commands here...
        elif msg.type == aiohttp.WSMsgType.ERROR:
            print('ws connection closed with exception %s', websocket.exception())

async def main():
    global model
    
    print("🧠 Initializing Laptop AI (WebSocket v2)...")
    
    # 1. Load Model
    print(f"📂 Loading Model: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)
    print("✅ Model Loaded")

    # 2. Connect Loop
    while True:
        try:
            print(f"🔌 Connecting to {SERVER_URL}...")
            async with aiohttp.ClientSession() as session:
                async with session.ws_connect(SERVER_URL) as websocket:
                    print("✅ Connected to Cloud!")
                    
                    # Run Vision and Listen routines concurrently
                    # (Vision is blocking CV2 loop, so we run it carefully)
                    # For simplicity in this script, we interleave send inside vision loop
                    # and polling inside vision loop is hard.
                    
                    # Better approach: Just run vision loop which sends data.
                    # Reading commands can be a background task if needed.
                    
                    await run_vision_loop(websocket)
                    
        except Exception as e:
            print(f"⚠️ Connection Failed: {e}")
            print("Retrying in 5 seconds...")
            time.sleep(5)

if __name__ == "__main__":
    asyncio.run(main())
