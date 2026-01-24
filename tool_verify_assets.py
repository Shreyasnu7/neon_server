import os
import sys

# Setup Path to find the AI Engine
sys.path.append(os.path.join(os.getcwd())) # Add root to path

try:
    from drone_project.ai.camera_brain.laptop_ai.ai_colour_engine import AIColorEngine
except ImportError:
    try:
        from ai.camera_brain.laptop_ai.ai_colour_engine import AIColorEngine
    except ImportError:
        print("[X] Could not import AIColorEngine. Check paths.")
        sys.exit(1)

def verify_assets():
    print("[*] Scanning for AI Assets (Data & Logic)...")
    
    base_path = os.getcwd()
    if 'drone_project' not in base_path:
        base_path = os.path.join(base_path, 'drone_project')
        
    print(f"[>] Base Path: {base_path}")

    # 1. Scan for Data Files (JSON, LUT, MP4)
    data_files = []
    video_assets = []
    
    ignored_dirs = ['venv', '.git', '__pycache__', 'node_modules']
    
    for root, dirs, files in os.walk(base_path):
        # Filter Ignored
        dirs[:] = [d for d in dirs if d not in ignored_dirs]
        
        for f in files:
            full_path = os.path.join(root, f)
            if f.endswith(('.json', '.lut', '.cube')):
                 data_files.append(full_path)
            elif f.endswith('.mp4'):
                 video_assets.append(full_path)
                 
    # 2. Scan for Code Logic (User definition of "1000 files")
    code_files = []
    for root, dirs, files in os.walk(base_path):
        dirs[:] = [d for d in dirs if d not in ignored_dirs]
        
        for f in files:
            if f.endswith('.py') and ('ai_' in f or 'camera_' in f):
                code_files.append(os.path.join(root, f))

    print(f"\n[=] ASSET INVENTORY:")
    print(f"   - Logic Modules (Python): {len(code_files)}")
    print(f"   - Config Assets (JSON/LUT): {len(data_files)}")
    print(f"   - Video Assets (MP4): {len(video_assets)}")
    
    total_assets = len(code_files) + len(data_files) + len(video_assets)
    print(f"[+] Total AI Components Found: {total_assets}")
    
    # 3. Dry Run Load
    print("\n[*] Testing AI Engine Load...")
    engine = AIColorEngine()
    
    # We pass the data files to the engine. 
    # If empty, engine should auto-generate based on strict user logic.
    engine.load_library(data_files)
    
    # 4. Final Verdict
    if len(code_files) > 0 and len(engine.library) > 0:
        print(f"\n[+] VERIFICATION SUCCESSFUL.")
        print(f"   The system detected {len(code_files)} AI Logic Modules.")
        print(f"   The Engine successfully prepared {len(engine.library)} Cinematic Styles.")
    else:
        print("\n[!] VERIFICATION WARNING: Engine did not initialize correctly.")

if __name__ == "__main__":
    verify_assets()
