#!/bin/bash
# force_system_drivers.sh
# Fixes: "UnicodeDecodeError" (Corrupted files in Venv)
# Strategy: IGNORE the broken SD Card Venv. Use System Libraries instead.

echo "========================================="
echo "🛡️  SWITCHING TO SYSTEM DRIVERS"
echo "    (Bypassing broken SD card files)"
echo "========================================="

# 1. INSTALL EVERYTHING VIA APT (System)
echo "📦 Installing System Libraries..."
sudo apt-get update
# We install ALL dependencies here so we don't need the broken venv anymore
sudo apt-get install -y python3-opencv python3-numpy python3-serial python3-aiohttp python3-websockets python3-pip

# 2. INSTALL PYMAVLINK (System Pip)
echo "📦 Installing Pymavlink (System)..."
sudo python3 -m pip install pymavlink --break-system-packages

# 3. FIX UART (Overlay)
echo "🔧 checking UART..."
ENV_FILE="/boot/armbianEnv.txt"
# Remove duplicates/bad entries
sudo sed -i '/overlays=/d' $ENV_FILE
# Add clean line
echo "overlays=uart2-m0" | sudo tee -a "$ENV_FILE" > /dev/null
echo "✅ UART Configured."

# 4. REWRITE LAUNCHER (PURE SYSTEM MODE)
echo "🚀 Creating Clean Launcher..."
LAUNCHER="/mnt/sdcard/drone_project/raxda_bridge/run_clean.sh"

cat <<'EOF' | sudo tee $LAUNCHER > /dev/null
#!/bin/bash
# PURE SYSTEM LAUNCHER (No Venv)

# 1. Mount SD Card (Files Only)
sudo mount -o remount,exec /mnt/sdcard 2>/dev/null

echo "========================================="
echo "💀 CHECKING HARDWARE..."
if [ -e /dev/ttyS2 ]; then
    sudo fuser -k /dev/ttyS2
    echo "✅ UART2 Ready."
else
    echo "⚠️  UART2 Missing. (Reboot Required)"
fi
echo "========================================="

echo "🚀 LAUNCHING DRONE BRIDGE (SYSTEM DRIVERS)"
# IMPORTANT: We do NOT set PYTHONPATH to the SD card venv.
# We trust the system libraries we just installed.
export PYTHONPATH="" 

# Run with /usr/bin/python3
sudo -E /usr/bin/python3 /mnt/sdcard/drone_project/raxda_bridge/real_bridge_service.py
EOF
sudo chmod +x $LAUNCHER

echo "========================================="
echo "✅ SUCCESS."
echo "We are now using System Drivers. The broken files are ignored."
echo "-----------------------------------------"
echo "1. REBOOT NOW: sudo reboot"
echo "2. LAUNCH: sudo $LAUNCHER"
echo "========================================="
