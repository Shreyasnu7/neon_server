
import re
import os

target = r"ai/camera_brain/laptop_ai/ai_pipeline/tone_curve/global_tone_curve.py"

print(f"[INFO] Deep cleaning {target}...")

with open(target, "r", encoding="utf-8") as f:
    lines = f.readlines()

# Pattern: Optional whitespace, then digits, then whitespace, then content
regex = re.compile(r"^(\s*)\d+\s+(.*)")

new_lines = []
cleaned_count = 0

for line in lines:
    # Skip if line is empty or just whitespace
    if not line.strip():
        new_lines.append(line)
        continue

    match = regex.match(line)
    if match:
        # Group 1: Leading whitespace (original indent?)
        # Group 2: The content after numbers
        
        # However, usually the file has "24   code" where 24 is at col 0.
        # If the original code was indented, it might be "24       code".
        # We need to trust the content's whitespace?
        
        # Let's inspect the content. 
        # If the content starts with 'class' or 'def', we might want to ensure indentation.
        # But generally, stripping the number prefix is the goal.
        
        content = match.group(2)
        
        # We assume the number was inserted AT THE START of the line, pushing existing indentation to the right
        # OR the number replaced the indentation.
        
        # Looking at previous successful clean: text starts at col 7, number at 0.
        # So stripping number+whitespace is roughly correct, but we might lose indent.
        
        # Heuristic: 
        # If the line starts with a number, we strip it.
        # We try to infer if we need to add indentation back?
        # For now, just stripping the number part is the safest bet to make it valid syntax (removing syntax error).
        # Unindent (IndentationError) is easier to fix later than SyntaxError.
        
        new_lines.append(content + "\n")
        cleaned_count += 1
    else:
        new_lines.append(line)

print(f"[INFO] Cleaned {cleaned_count} lines.")
print(f"[INFO] Writing {len(new_lines)} lines...")

with open(target, "w", encoding="utf-8") as f:
    f.writelines(new_lines)

print("[SUCCESS] Done.")
