
import re
import os

target = r"ai/camera_brain/laptop_ai/ai_pipeline/tone_curve/global_tone_curve.py"

print(f"[INFO] Processing {target}...")

with open(target, "r", encoding="utf-8") as f:
    lines = f.readlines()

print(f"[INFO] Read {len(lines)} lines.")

# Step 1: Replace leading digits/garbage with Spaces to preserve alignment
# Regex: Start of line, optional space, digits, optional dots/colons (garbage)
# We capture the WHOLE garbage prefix
regex_junk = re.compile(r"^([ \t]*\d+[:.]?)\s*")

# Actually, usually it is "24   code".
# If we replace "24" with "  ", we keep alignment.
# But we also need to handle Headers which might be "HUNK 1" -> "# HUNK 1"

# Let's iterate and be smart.
output_lines_step1 = []

for line in lines:
    # Heuristic 1: If it starts with digits, mask them with spaces.
    match = regex_junk.match(line)
    if match:
        prefix = match.group(1) # e.g. "105" or "  105"
        
        # We replace the non-whitespace chars in the prefix with spaces
        # But wait, looking at the file:
        # "10          mt_luma"
        # If we replace "10" with "  ", we get "            mt_luma" (12 spaces?).
        # That seems plausible.
        
        # Construct new line: 
        # Replace the numeric prefix in the original string with space-equivalent
        # Be careful not to replace digits inside the code.
        
        # Taking the substring length of the match
        end_idx = match.end() # Index where "real code" or "more spaces" begins
        
        # But wait, `regex_junk` included `\s*` at the end? 
        # No, I put `\s*` outside the group 1? No, `regex_junk` group 1 ends at digit.
        pass

    # Simplified approach:
    # 1. Match `^\s*\d+\s+` (The garbage block)
    # 2. Look at the code AFTER that block.
    # 3. Calculate "visual indent" of that code assuming the garbage was indentation.
    
    # Actually, looking at the user's view:
    # Line 21: '9      def __init__(self):' -> Code at index 7. (0-indexed)
    # Line 11840: '11          mt_luma' -> "11" then spaces.
    
    # If we assume the file was consistently formatted with line numbers on the left...
    # We should just strip the line number token AND the spaces immediately following it?
    # NO, that caused the flattening!
    
    # We should strip the line number token, BUT KEEP the spaces?
    # '9      def' -> remove '9' -> '      def' (6 spaces).
    # '11          mt_luma' -> remove '11' -> '          mt_luma' (10 spaces).
    
    # Let's try that.
    stripped_line = re.sub(r"^(\s*)\d+", r"\1", line) 
    # This replaces "  123" with "  ". It removes digits but keeps leading space.
    # AND it keeps trailing space (between digit and code).
    
    output_lines_step1.append(stripped_line)

# Step 2: Global Dedent
# Now we have a file that is indented way too far to the right (e.g. 7 spaces or 10 spaces).
# We need to find the "minimum indent" of the root-level classes/imports.
# Imports should be at 0.
# Classes should be at 0.

min_indent = float("inf")

for line in output_lines_step1:
    s = line.lstrip()
    if not s: continue # Skip empty
    
    if s.startswith("import ") or s.startswith("class ") or s.startswith("def ") or s.startswith("#"):
        # This is a candidate for root level
        indent = len(line) - len(s)
        if indent < min_indent:
            min_indent = indent

print(f"[INFO] Detected min indent: {min_indent}")

if min_indent == float("inf"):
    min_indent = 0

final_lines = []
for line in output_lines_step1:
    if len(line.strip()) == 0:
        final_lines.append("\n")
    else:
        if len(line) >= min_indent:
            final_lines.append(line[min_indent:])
        else:
            final_lines.append(line.lstrip())

# Verify Headers
# The headers usually get messed up into "HUNK ...".
# We'll fix them quickly here too.
fixed_lines = []
regex_header = re.compile(r"^(={2,}|-{2,}|HUNK|ines:|tone_curve/|ACES TRANSFORM)")

for line in final_lines:
    if regex_header.match(line):
        fixed_lines.append("# " + line)
    else:
        fixed_lines.append(line)

print(f"[INFO] Writing {len(fixed_lines)} lines...")
with open(target, "w", encoding="utf-8") as f:
    f.writelines(fixed_lines)

print("[SUCCESS] Done.")
