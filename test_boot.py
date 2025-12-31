
import sys
import os
sys.path.append(os.getcwd())

print("Testing Server Boot Integrity...")

try:
    from main import app
    print("Main App Imported Successfully")
except Exception as e:
    print(f"Main App Import FAILED: {e}")
    import traceback
    traceback.print_exc()

print("Checking Video Router dependencies...")
try:
    from video_router import router
    print("Video Router Imported")
except Exception as e:
    print(f"Video Router Import FAILED: {e}")
    import traceback
    traceback.print_exc()

print("Server Boot Test Complete")
