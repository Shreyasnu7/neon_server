
import py_compile
import sys
import os

target = r"ai/camera_brain/laptop_ai/ai_pipeline/tone_curve/global_tone_curve.py"

max_iterations = 200
iteration = 0

print(f"[INFO] Starting self-healing on {target}")

def safe_print(msg):
    try:
        print(msg)
    except UnicodeEncodeError:
        print(msg.encode("ascii", "replace").decode("ascii"))

while iteration < max_iterations:
    iteration += 1
    try:
        # Try to compile
        py_compile.compile(target, doraise=True)
        safe_print(f"[SUCCESS] File compiled successfully after {iteration-1} fixes!")
        sys.exit(0)
    except py_compile.PyCompileError as e:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        
        syntax_err = None
        if hasattr(exc_value, 'exc_value'):
            syntax_err = exc_value.exc_value
        elif hasattr(exc_value, 'lineno'):
            syntax_err = exc_value
            
        if not syntax_err:
            safe_print(f"[ERROR] Could not parse error: {e}")
            sys.exit(1)
            
        lineno = syntax_err.lineno
        msg = syntax_err.msg
        safe_print(f"[FIXING] Error at line {lineno}: {msg}")
        
        # Read file
        with open(target, "r", encoding="utf-8") as f:
            lines = f.readlines()
            
        if lineno is None or lineno > len(lines):
             safe_print("[ERROR] Line number out of range.")
             sys.exit(1)
             
        # Fix the line - indent it if it's an indentation error, or comment it if it looks like garbage
        bad_line = lines[lineno - 1]
        safe_print(f"   -> Bad Line Content: {repr(bad_line)}")
        
        # Strategy: Comment it out. It's usually a broken header or description.
        # Check if it's already a comment but merely indented wrong? 
        # No, usually IndentationError means it's unexpected code.
        lines[lineno - 1] = "# " + bad_line
        
        # Write back
        with open(target, "w", encoding="utf-8") as f:
            f.writelines(lines)
            
    except Exception as e:
        safe_print(f"[CRITICAL] Unexpected error: {e}")
        sys.exit(1)

safe_print("[FAILED] Max iterations reached.")
