
import os

target = r"ai/camera_brain/laptop_ai/ai_pipeline/tone_curve/global_tone_curve.py"

print(f"[INFO] Reading {target}...")
with open(target, "r", encoding="utf-8") as f:
    lines = f.readlines()

# Find the specific indentation of the first import
indent_to_strip = 0
for line in lines:
    if "import numpy as np" in line:
        stripped = line.lstrip()
        indent_to_strip = len(line) - len(stripped)
        print(f"[INFO] Found baseline indentation: {indent_to_strip} spaces on 'import numpy as np'")
        break

if indent_to_strip == 0:
    print("[WARN] No indentation found on target line. Checking regex artifacts...")
    # Fallback: check if we just need to lstrip everything if it's consistent
    pass

new_lines = []
for line in lines:
    # We only strip if the line actually starts with the indent pattern
    # Otherwise (like empty lines or comments at 0), we leave them
    if line.startswith(" " * indent_to_strip):
        new_lines.append(line[indent_to_strip:])
    else:
        new_lines.append(line)

print(f"[INFO] Writing {len(new_lines)} fixed lines...")
with open(target, "w", encoding="utf-8") as f:
    f.writelines(new_lines)

print("[SUCCESS] Indentation fixed.")
