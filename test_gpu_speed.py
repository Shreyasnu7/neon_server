
import time
import torch
import cv2
import numpy as np
from ultralytics import YOLO
from ai.camera_brain.laptop_ai.ai_pipeline.tone_curve.gpu_aces import GPUACESToneCurve

def benchmark():
    print(f"🏎️ Starting GPU Benchmark on {torch.cuda.get_device_name(0)}...")
    
    # 1. Setup
    frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    # Load YOLO
    t0 = time.time()
    model = YOLO("yolov8n.pt")
    model.to('cuda')
    print(f"✅ YOLO Loaded: {time.time()-t0:.2f}s")
    
    # Load ACES
    t0 = time.time()
    aces = GPUACESToneCurve()
    print(f"✅ ACES Loaded: {time.time()-t0:.2f}s")
    
    # 2. Warmup
    print("🔥 Warming up GPU...")
    for _ in range(10):
        model(frame, verbose=False)
        aces.apply(frame)
    
    # 3. Speed Test
    print("🚀 Running 100 Frames...")
    t_start = time.time()
    for i in range(100):
        # ACES
        f_aces = aces.apply(frame)
        # YOLO
        res = model(f_aces, verbose=False)
    
    t_end = time.time()
    total_time = t_end - t_start
    fps = 100 / total_time
    
    print(f"🏁 Result: {fps:.2f} FPS")
    print(f"⏱️ Total Time: {total_time:.2f}s")

if __name__ == "__main__":
    benchmark()
