
import re
import os

target = r"ai/camera_brain/laptop_ai/ai_pipeline/tone_curve/global_tone_curve.py"

print(f"[INFO] Re-aligning classes in {target}...")

with open(target, "r", encoding="utf-8") as f:
    lines = f.readlines()

new_lines = []
current_offset = 0

# Regex to detect START of a class definition
# strictly matching "class Name" with optional leading whitespace
class_regex = re.compile(r"^(\s*)class\s+\w+")

for line in lines:
    match = class_regex.match(line)
    if match:
        # We found a class start!
        leading_spaces = match.group(1)
        offset = len(leading_spaces)
        
        # We want this class to be at 0.
        # So we set the current_offset for this BLOCK to 'offset'.
        current_offset = offset
        
        # Apply dedent to this line immediately
        new_lines.append(line.lstrip())
    else:
        # For normal lines, we apply the current_offset
        if current_offset > 0:
            if len(line) >= current_offset:
                new_lines.append(line[current_offset:])
            else:
                new_lines.append(line.lstrip()) 
        else:
            new_lines.append(line)

print(f"[INFO] Writing {len(new_lines)} re-aligned lines...")
with open(target, "w", encoding="utf-8") as f:
    f.writelines(new_lines)

print("[SUCCESS] Classes aligned.")
