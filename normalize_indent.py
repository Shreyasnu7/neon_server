
import re
import os

target = r"ai/camera_brain/laptop_ai/ai_pipeline/tone_curve/global_tone_curve.py"

print(f"[INFO] Processing {target}...")

with open(target, "r", encoding="utf-8") as f:
    lines = f.readlines()

# 1. Find the Zero-Reference Column (where 'class ACESHelpers' starts)
zero_ref_col = None

# Regex to find lines that are "numbered" or just indented code
# Matches: (garbage)(content)
# We want to find the INDEX of content in the original raw line.
for line in lines:
    if "class ACESHelpers" in line:
        idx = line.find("class ACESHelpers")
        if idx != -1:
            zero_ref_col = idx
            print(f"[INFO] Found Zero-Ref Column at index: {zero_ref_col}")
            break

if zero_ref_col is None:
    # Fallback to 'import numpy'
    for line in lines:
        if "import numpy" in line:
            idx = line.find("import numpy")
            if idx != -1:
                zero_ref_col = idx
                print(f"[INFO] Found Zero-Ref Column at index: {zero_ref_col} (via import)")
                break

if zero_ref_col is None:
    print("[ERROR] Could not find anchor. Defaulting to 0?")
    zero_ref_col = 0 # Fallback

# 2. Normalize every line
new_lines = []
digits_regex = re.compile(r"^\s*\d+\s*") # Matches leading numbers/whitespace

count_fixed = 0

for line in lines:
    if not line.strip():
        new_lines.append("\n")
        continue

    # Determine "Content Start Index"
    # Logic: We must identify "The Code".
    # If the line starts with digits, we skip them.
    
    match = digits_regex.match(line)
    if match:
        # Line has numbers. Content starts after the match.
        content_start = match.end()
        # But wait, 'content_start' is the index in the string.
        # BUT 'digits_regex' includes following spaces `\s*`.
        # So `line[content_start:]` is the code without leading spaces.
        
        # We need the VISUAL start index of that code in the original line?
        # NO.
        # Example: "9      def"
        # Match matches "9      ". End is 7.
        # content is "def".
        # visual_col is 7.
        # indentation = 7 - zero_ref (3) = 4.
        
        # Example: "14     def"
        # Match matches "14     ". End is 7.
        # visual_col is 7.
        # indentation = 7 - 3 = 4.
        
        # So YES, `match.end()` IS the visual column index of the code start!
        visual_col = match.end()
        content = line[visual_col:].rstrip() # The code itself
        
        # Calculate new indent
        rel_indent = visual_col - zero_ref_col
        # Normalize to 0 if negative (e.g. headers to the left of class)
        if rel_indent < 0:
            rel_indent = 0
            
        new_line = (" " * rel_indent) + content + "\n"
        new_lines.append(new_line)
        count_fixed += 1
        
    else:
        # Line does NOT have numbers. (Already cleaned? Or headers?)
        # We assume it's fine or we try to dedent it based on zero_ref?
        # If it's "import numpy..." at index 7?
        # It should probably be dedented too.
        
        # Let's find the first non-whitespace char
        lstripped = line.lstrip()
        visual_col = len(line) - len(lstripped)
        
        content = lstripped.rstrip()
        
        if not content:
            new_lines.append("\n")
            continue

        rel_indent = visual_col - zero_ref_col
        if rel_indent < 0:
            rel_indent = 0
            
        new_line = (" " * rel_indent) + content + "\n"
        new_lines.append(new_line)

print(f"[INFO] Normalized {count_fixed} numbered lines.")
print(f"[INFO] Writing {len(new_lines)} lines...")

with open(target, "w", encoding="utf-8") as f:
    f.writelines(new_lines)

print("[SUCCESS] Done.")
