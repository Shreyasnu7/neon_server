
import os
import re

target = r"ai/camera_brain/laptop_ai/ai_pipeline/tone_curve/global_tone_curve.py"

print(f"[INFO] Processing {target}...")

with open(target, "r", encoding="utf-8") as f:
    lines = f.readlines()

new_lines = []
fixed_count = 0

# Regex for things that look like broken comments
# Starts with 2 or more =, or 2 or more -, or specific words
# 'HUNK' comes from 'CHUNK' sliced. 'ines:' from 'Lines:'
regex = re.compile(r"^(={2,}|-{2,}|HUNK|ines:|tone_curve/|ACES TRANSFORM)")

for line in lines:
    if regex.match(line):
        new_line = "# " + line
        new_lines.append(new_line)
        fixed_count += 1
    else:
        new_lines.append(line)

print(f"[INFO] Fixed {fixed_count} broken header lines.")
print(f"[INFO] Writing {len(new_lines)} lines...")

with open(target, "w", encoding="utf-8") as f:
    f.writelines(new_lines)

print("[SUCCESS] Done.")
