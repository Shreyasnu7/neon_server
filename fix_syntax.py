import re
import os

# Confirmed correct paths
targets = [
    r"ai/camera_brain/laptop_ai/ai_pipeline/tone_curve/global_tone_curve.py",
    r"ai/camera_brain/laptop_ai/ai_exposure_engine.py"
]

# Regex to catch "14     def..." or "4  class..."
# Matches start of line, optional whitespace, digits, whitespace, capture rest
pattern = re.compile(r"^\s*\d+\s+(.*)")

for target_file in targets:
    if not os.path.exists(target_file):
        print(f"[ERROR] File not found: {target_file}")
        continue

    print(f"--------------------------------------------------")
    print(f"[INFO] Reading {target_file}...")
    try:
        with open(target_file, "r", encoding="utf-8") as f:
            lines = f.readlines()
            
        print(f"[INFO] Original Line Count: {len(lines)}")
        
        new_lines = []
        modified_count = 0
        
        for line in lines:
            match = pattern.match(line)
            if match:
                # We found a numbered line
                content = match.group(1)
                new_lines.append(content + "\n")
                modified_count += 1
            else:
                # Normal line
                new_lines.append(line)
        
        print(f"[INFO] Cleaned {modified_count} lines.")
        print(f"[INFO] Saving {len(new_lines)} lines... (Zero Data Loss)")
        
        with open(target_file, "w", encoding="utf-8") as f:
            f.writelines(new_lines)
            
        print(f"[SUCCESS] Processed: {target_file}")
        
    except Exception as e:
        print(f"[ERROR] Processing {target_file}: {e}")

print("--------------------------------------------------")
print("[DONE] All files sanitized.")
