
import os

target = r"ai/camera_brain/laptop_ai/ai_pipeline/tone_curve/global_tone_curve.py"

print(f"[INFO] Reading {target}...")
with open(target, "r", encoding="utf-8") as f:
    lines = f.readlines()

# Find indentation of the first import
indent_to_strip = 0
found_anchor = False

for line in lines:
    if "import numpy as np" in line:
        stripped = line.lstrip()
        indent_to_strip = len(line) - len(stripped)
        print(f"[INFO] Anchor found: 'import numpy as np' with {indent_to_strip} spaces.")
        found_anchor = True
        break

if not found_anchor:
    print("[WARN] Anchor not found. Checking 'class ACESHelpers'...")
    for line in lines:
        if "class ACESHelpers:" in line:
            stripped = line.lstrip()
            indent_to_strip = len(line) - len(stripped)
            print(f"[INFO] Anchor found: 'class ACESHelpers' with {indent_to_strip} spaces.")
            break

print(f"[INFO] Stripping {indent_to_strip} spaces from all lines...")

new_lines = []
for line in lines:
    if len(line.strip()) == 0:
        new_lines.append("\n")
        continue

    # Careful: comments might be at 0 already. 
    # If a line has LESS indent than indent_to_strip, we probably just lstrip it (it's likely a comment or broken line).
    # But if it's code, it should be indented >= indent_to_strip.
    
    current_indent = len(line) - len(line.lstrip())
    
    if current_indent >= indent_to_strip:
        new_lines.append(line[indent_to_strip:])
    else:
        # It's to the left of our anchor. Probably a header comment.
        # Just leave it or lstrip it?
        # Headers like "# ===" are fine at 0.
        new_lines.append(line)

print(f"[INFO] Writing {len(new_lines)} lines...")
with open(target, "w", encoding="utf-8") as f:
    f.writelines(new_lines)

print("[SUCCESS] Dedented.")
