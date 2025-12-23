
target = r"ai/camera_brain/laptop_ai/ai_pipeline/tone_curve/global_tone_curve.py"

print("--- Column Alignment Check ---")
with open(target, "r", encoding="utf-8") as f:
    lines = f.readlines()

# Check lines 15, 21, 27 (lines from previous errors)
indices = [14, 15, 20, 26] # 0-indexed
for i in indices:
    if i < len(lines):
        line = lines[i].rstrip()
        print(f"Line {i+1}: {repr(line)}")
        # Find where the letters start
        for idx, char in enumerate(line):
            if char.isalpha() or char == '"': # Start of code (class, def, quotes)
                print(f"  -> Code starts at index: {idx}")
                break
