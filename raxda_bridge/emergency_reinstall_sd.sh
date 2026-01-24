#!/bin/bash
# emergency_reinstall_sd.sh
# The "Repair" fixed the disk, but the files are gone/broken.
# We must reinstall the software onto the "healed" disk.

SD_MOUNT="/mnt/sdcard"
SD_VENV="$SD_MOUNT/venv"

echo "========================================="
echo "🚑 EMERGENCY REINSTALL (SD CARD)"
echo "========================================="

# 1. FIX UART (Again)
echo "🔧 Ensuring UART2 Overlay..."
ENV_FILE="/boot/armbianEnv.txt"
if grep -q "overlays=uart2-m0" "$ENV_FILE"; then
    echo "   - Overlay found."
else
    echo "   - Adding Overlay..."
    echo "overlays=uart2-m0" | sudo tee -a "$ENV_FILE"
fi

# 2. REINSTALL VENV
echo "🐍 Rebuilding Python Brain..."
# Delete the likely-corrupted venv
sudo rm -rf $SD_VENV

# Create fresh
sudo python3 -m venv $SD_VENV --system-site-packages

# 3. INSTALL LIBRARIES
echo "📦 Installing Libraries (Force)..."
sudo $SD_VENV/bin/pip install --upgrade pip --break-system-packages
# Force reinstall to ensure files are written to GOOD blocks
sudo $SD_VENV/bin/pip install --force-reinstall pyserial aiohttp websockets pymavlink opencv-python-headless --break-system-packages

# 4. VERIFY
echo "🔍 Verifying Installation..."
if sudo $SD_VENV/bin/python3 -c "import aiohttp; print('aiohttp: OK')" 2>/dev/null; then
    echo "✅ Software Fixed."
else
    echo "❌ FATAL: Installation failed. SD Card still corrupt."
fi

echo "========================================="
echo "🎉 DONE. TRY LAUNCHING NOW."
echo "sudo /mnt/sdcard/drone_project/raxda_bridge/run_clean.sh"
echo "========================================="
