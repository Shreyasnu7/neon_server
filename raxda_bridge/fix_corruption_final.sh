#!/bin/bash
# fix_corruption_final.sh
# Fixes "SyntaxError: null bytes" and "ModuleNotFoundError"
# STRATEGY: Install libraries to SYSTEM (eMMC) to bypass SD Card corruption.
# CODE remains on SD Card.

echo "========================================="
echo "💊 FIXING CORRUPTION (SYSTEM INSTALL)"
echo "   Installing libraries to Internal Storage..."
echo "========================================="

# 1. INSTALL SYSTEM PACKAGES (Stable, on eMMC)
# These are big files, so we put them on the good chip.
sudo apt-get update
sudo apt-get install -y python3-opencv python3-numpy python3-aiohttp python3-serial python3-websockets python3-pip

# 2. INSTALL REMAINING LIB (Pymavlink)
# We install this to /usr/local/lib (eMMC)
echo "📦 Installing Pymavlink..."
sudo python3 -m pip install pymavlink --break-system-packages

# 3. FIX UART (One last time)
echo "🔧 checking UART..."
ENV_FILE="/boot/armbianEnv.txt"
# Ensure line exists
if ! grep -q "overlays=uart2-m0" "$ENV_FILE"; then
    echo "overlays=uart2-m0" | sudo tee -a "$ENV_FILE"
fi

# 4. REWRITE LAUNCHER (Simple & Robust)
echo "🚀 Updating Launcher..."
LAUNCHER="/mnt/sdcard/drone_project/raxda_bridge/run_clean.sh"

cat <<'EOF' | sudo tee $LAUNCHER > /dev/null
#!/bin/bash
# Force mount
sudo mount -o remount,exec /mnt/sdcard 2>/dev/null

echo "========================================="
echo "💀 CHECKING HARDWARE..."
if [ -e /dev/ttyS2 ]; then
    sudo fuser -k /dev/ttyS2
    echo "✅ UART2 Ready."
else
    echo "⚠️  UART2 Missing. (Reboot might be needed)"
fi
echo "========================================="
sleep 0.5
echo "🚀 LAUNCHING DRONE BRIDGE"

# USE SYSTEM PYTHON (Directly)
# No venv needed because we installed libs to system
sudo /usr/bin/python3 /mnt/sdcard/drone_project/raxda_bridge/real_bridge_service.py
EOF
sudo chmod +x $LAUNCHER

echo "========================================="
echo "✅ REPAIR COMPLETE."
echo "We moved the heavy libraries to the internal chip."
echo "Your code is still on the SD card."
echo "-----------------------------------------"
echo "1. REBOOT: sudo reboot"
echo "2. LAUNCH: sudo $LAUNCHER"
echo "========================================="
