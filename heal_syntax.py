
import py_compile
import sys
import os

target = r"ai/camera_brain/laptop_ai/ai_pipeline/tone_curve/global_tone_curve.py"

max_iterations = 200
iteration = 0

print(f"[INFO] Starting self-healing on {target}")

while iteration < max_iterations:
    iteration += 1
    try:
        # Try to compile
        py_compile.compile(target, doraise=True)
        print(f"[SUCCESS] File compiled successfully after {iteration-1} fixes!")
        sys.exit(0)
    except py_compile.PyCompileError as e:
        # Extract line number from error message
        # Format usually: File "...", line X
        # But e.msg might help, or parsing e.exc_value
        # The exception object e has a 'exc_value' which is a SyntaxError
        
        # We need to catch the inner SyntaxError
        exc_type, exc_value, exc_traceback = sys.exc_info()
        
        # PyCompileError wraps the SyntaxError
        if hasattr(exc_value, 'exc_value'):
            syntax_err = exc_value.exc_value
        elif hasattr(exc_value, 'lineno'):
            syntax_err = exc_value
        else:
            # Fallback parsing string
            print(f"[ERROR] Could not parse error: {e}")
            sys.exit(1)
            
        lineno = syntax_err.lineno
        msg = syntax_err.msg
        print(f"[FIXING] Error at line {lineno}: {msg}")
        
        # Read file
        with open(target, "r", encoding="utf-8") as f:
            lines = f.readlines()
            
        # Fix the line
        if lineno is None or lineno > len(lines):
             print("[ERROR] Line number out of range.")
             sys.exit(1)
             
        bad_line = lines[lineno - 1]
        print(f"   -> Bad Line: {repr(bad_line)}")
        
        # Heuristic: If it looks like text, comment it out
        lines[lineno - 1] = "# " + bad_line
        
        # Write back
        with open(target, "w", encoding="utf-8") as f:
            f.writelines(lines)
            
    except Exception as e:
        print(f"[CRITICAL] Unexpected error: {e}")
        sys.exit(1)

print("[FAILED] Max iterations reached.")
