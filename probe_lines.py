
import os

target = r"ai/camera_brain/laptop_ai/ai_pipeline/tone_curve/global_tone_curve.py"

with open(target, "r", encoding="utf-8") as f:
    lines = f.readlines()

print("--- DIAGNOSTIC ---")
for i in range(14, 25):  # Lines 15-25 (0-indexed 14-24)
    line = lines[i]
    print(f"L{i+1}: {repr(line)}")
