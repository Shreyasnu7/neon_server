
import os
import sys

target = r"ai/camera_brain/laptop_ai/ai_pipeline/tone_curve/global_tone_curve.py"

print(f"[INFO] Processing {target}...")

with open(target, "r", encoding="utf-8") as f:
    lines = f.readlines()

# Detect offset based on 'class ACESHelpers'
offset = None
for line in lines:
    if "class ACESHelpers:" in line:
        # Find where 'class' starts
        offset = line.find("class ACESHelpers:")
        print(f"[INFO] Detected slice offset: {offset} (based on ACESHelpers)")
        break

if offset is None:
    print("[ERROR] Could not find anchor line 'class ACESHelpers:'. Aborting.")
    sys.exit(1)

# Safety check on other lines
def verify_offset(str_to_find, expected_rel_indent):
    for line in lines:
        if str_to_find in line:
            idx = line.find(str_to_find)
            rel = idx - offset
            print(f"[INFO] Check '{str_to_find}': Index {idx} (Rel {rel}). Expected around {expected_rel_indent}.")
            return

verify_offset("def __init__(self):", 4)
verify_offset("def srgb_to_linear(self, img):", 4)

print(f"[INFO] Slicing file at column {offset}...")

new_lines = []
for line in lines:
    if len(line) > offset:
        # Slice
        content = line[offset:]
        new_lines.append(content)
    else:
        # Line too short, probably just maintain it (newlines)
        new_lines.append(line)

print(f"[INFO] Writing {len(new_lines)} cleaned lines...")
with open(target, "w", encoding="utf-8") as f:
    f.writelines(new_lines)

print("[SUCCESS] Fixed.")
