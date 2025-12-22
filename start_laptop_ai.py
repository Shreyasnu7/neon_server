import asyncio
from dotenv import load_dotenv
load_dotenv()
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "ai", "camera_brain"))

from laptop_ai.director_core import main_loop

if __name__ == "__main__":
    print("🎬 Starting Unified AI Director (Vision + Planning + Fusion)")
    print("   [x] UltraDirector Active")
    print("   [x] ObstacleWarp Active")
    print("   [x] 96-Module Brain Active")
    asyncio.run(main_loop(simulate=False))
