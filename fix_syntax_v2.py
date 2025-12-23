
import re
import os
import sys

target = r"ai/camera_brain/laptop_ai/ai_pipeline/tone_curve/global_tone_curve.py"

print(f"[INFO] Processing {target}...")

if not os.path.exists(target):
    print("[ERROR] File not found.")
    sys.exit(1)

with open(target, "r", encoding="utf-8") as f:
    lines = f.readlines()

print(f"[INFO] Read {len(lines)} lines.")

# Regex captures everything AFTER the digits, including spaces
# Group 1 is the "content" (spaces + code)
pattern = re.compile(r"^\s*\d+(.*)")

extracted_lines = []
for line in lines:
    match = pattern.match(line)
    if match:
        content = match.group(1)
        # Convert tabs to spaces just in case, for consistent dedenting
        extracted_lines.append(content.replace("\t", "    "))
    else:
        # Keep non-numbered lines as is
        extracted_lines.append(line)

# Determine minimum indentation to unshift the file
min_indent = float("inf")
for line in extracted_lines:
    stripped = line.lstrip()
    if not stripped: # Skip empty lines
        continue
    # Calculate indent level
    indent = len(line) - len(stripped)
    if indent < min_indent:
        min_indent = indent

print(f"[INFO] Detected baseline indentation: {min_indent} spaces.")

# Dedent
final_lines = []
for line in extracted_lines:
    if len(line.strip()) == 0:
        final_lines.append("\n")
    else:
        if len(line) >= min_indent:
            final_lines.append(line[min_indent:] + "\n" if not line.endswith("\n") else line[min_indent:])
        else:
             final_lines.append(line) # Should not match if min_indent is correct

print(f"[INFO] Writing {len(final_lines)} dedented lines...")
with open(target, "w", encoding="utf-8") as f:
    f.writelines(final_lines)

print("[SUCCESS] Done.")
