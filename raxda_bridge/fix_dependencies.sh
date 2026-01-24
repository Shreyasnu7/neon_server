#!/bin/bash
# fix_dependencies.sh v2
USER_HOME="/home/shreyash"

echo "========================================"
echo "🔧 FORCE INSTALLING DEPENDENCIES (v2)"
echo "========================================"

# 1. FIX SERIAL (Force Repip install aiohttp opencv-python-headless pyserial it)
echo "📦 Force-Reinstalling Pyserial..."
sudo python3 -m pip install pyserial --break-system-packages --force-reinstall
sudo python3 -m pip install ydlidar --break-system-packages --force-reinstall

# 2. REMOVED YDLIDAR SDK BUILD (SAVES DISK SPACE)
# The SDK build was filling the eMMC and failing on Python 3.13.
# We will run without Lidar for now.

echo "========================================="
echo "✅ DEPENDENCIES FIXED."
echo "Now run: sudo ~/drone_project/raxda_bridge/fix_launcher_v2.sh"
echo "========================================="
