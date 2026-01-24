#!/bin/bash
# setup_hybrid.sh
# STRATEGY: COMPROMISE
# 1. SD Card is used for CODE ONLY (Read-Only access mostly).
# 2. VENV & Libraries are installed on EMMC (Internal Storage).
# This avoids the "Structure needs cleaning" error because we stop writing to the bad SD card.

echo "========================================="
echo "🦄 STARTING HYBRID SETUP"
echo "   (Code on SD | Brain on EMMC)"
echo "========================================="

HOME_DIR="/home/shreyash"
INTERNAL_VENV="$HOME_DIR/venv"
SD_MOUNT="/mnt/sdcard"

# 1. CLEANUP OLD BROKEN VENV (On SD)
if [ -d "$SD_MOUNT/venv" ]; then
    echo "🗑️  Ignoring broken SD card venv..."
fi

# 2. SETUP FRESH VENV (On Internal Storage - SAFE)
echo "🐍 Creating Python Brain on Internal Storage..."
# Install venv tool if missing
sudo apt-get update
sudo apt-get install -y python3-venv python3-opencv

# Create venv locally
rm -rf $INTERNAL_VENV
python3 -m venv $INTERNAL_VENV --system-site-packages

# 3. INSTALL LIBRARIES (On Internal Storage - FAST)
echo "📦 Installing Libraries (Internal)..."
$INTERNAL_VENV/bin/pip install --upgrade pip --break-system-packages
$INTERNAL_VENV/bin/pip install pyserial aiohttp websockets pymavlink opencv-python-headless --break-system-packages
echo "✅ Brain Ready."

# 4. CREATE LAUNCHER (Run from EMMC, Read from SD)
echo "🚀 Updating Launcher..."
LAUNCHER="$SD_MOUNT/drone_project/raxda_bridge/run_clean.sh"

# We try to write the launcher to SD. If it fails (Corruption), we write it to Home.
TARGET_LAUNCHER=$LAUNCHER
if ! sudo touch $LAUNCHER 2>/dev/null; then
    echo "⚠️ SD Card Write Failed. Saving Launcher to Home instead."
    TARGET_LAUNCHER="$HOME_DIR/run_drone.sh"
fi

cat <<EOF | sudo tee $TARGET_LAUNCHER > /dev/null
#!/bin/bash
# Hybrid Launcher
# Exec: Internal Venv
# Code: SD Card

# Ensure SD is mounted
sudo mount -o remount,exec /mnt/sdcard 2>/dev/null

echo "========================================="
echo "🦄 LAUNCHING HYBRID MODE"
echo "========================================="

# Check Hardware
if [ -e /dev/ttyS2 ]; then
    sudo fuser -k /dev/ttyS2
else
    echo "⚠️ UART2 Missing."
fi

# RUN: Internal Python -> SD Code
sudo $INTERNAL_VENV/bin/python3 /mnt/sdcard/drone_project/raxda_bridge/real_bridge_service.py
EOF

sudo chmod +x $TARGET_LAUNCHER

echo "========================================="
echo "🎉 HYBRID SETUP COMPLETE."
echo "-----------------------------------------"
echo "Launch Command:"
echo "sudo $TARGET_LAUNCHER"
echo "========================================="
