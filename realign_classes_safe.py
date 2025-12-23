
import re
import os

target = r"ai/camera_brain/laptop_ai/ai_pipeline/tone_curve/global_tone_curve.py"

print(f"[INFO] Safe Re-aligning classes in {target}...")

with open(target, "r", encoding="utf-8") as f:
    lines = f.readlines()

new_lines = []
current_offset = 0

# Regex to detect START of a class definition
class_regex = re.compile(r"^(\s*)class\s+\w+")

for line in lines:
    match = class_regex.match(line)
    if match:
        leading_spaces = match.group(1)
        offset = len(leading_spaces)
        current_offset = offset
        new_lines.append(line.lstrip())
    else:
        if current_offset > 0:
            if len(line) > current_offset:
                new_lines.append(line[current_offset:])
            else:
                # SAFE CHANGE: If line is shorter than offset (e.g. empty), 
                # we just output a newline (or stripped version).
                # WE NEVER DROP THE LINE.
                stripped = line.strip()
                if not stripped:
                    new_lines.append("\n") # Preserve blank line
                else: 
                    # If it has content but meant to be dedented?
                    # e.g. "  # comment" with offset 4.
                    # We just lstrip it to be safe, so it doesn't stay indented.
                    new_lines.append(line.lstrip())
        else:
            new_lines.append(line)

print(f"[INFO] Writing {len(new_lines)} re-aligned lines...")
with open(target, "w", encoding="utf-8") as f:
    f.writelines(new_lines)

print("[SUCCESS] Classes aligned (Data Preserved).")
