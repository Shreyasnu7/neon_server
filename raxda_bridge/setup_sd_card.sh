#!/bin/bash
# setup_sd_card.sh
# Migrates Project & Environment to SD Card

SD_MOUNT="/mnt/sdcard"
SD_PROJECT="$SD_MOUNT/drone_project"
SD_VENV="$SD_MOUNT/venv"
USER_HOME="/home/shreyash"

echo "========================================"
echo "💾 MIGRATING TO SD CARD ($SD_MOUNT)"
echo "========================================"

# 1. MOVE CODE
echo "📂 Moving Project Code..."
sudo mkdir -p $SD_PROJECT/raxda_bridge
# Copy files if they exist in standard location, otherwise we assume a fresh start or they are already there
if [ -d "$USER_HOME/drone_project" ]; then
    sudo cp -r $USER_HOME/drone_project/* $SD_PROJECT/
    echo "✅ Code Copied."
else
    echo "⚠️ Source folder empty/missing. Creating empty structure."
fi

# Ensure permissions
sudo chown -R $USER:$USER $SD_MOUNT 2>/dev/null || sudo chown -R 1000:1000 $SD_MOUNT

# 2. CREATE VIRTUAL ENVIRONMENT (On SD Card)
echo "🐍 Creating Python Virtual Environment..."
if [ ! -d "$SD_VENV" ]; then
    sudo apt-get install -y python3-venv
    python3 -m venv $SD_VENV --system-site-packages
fi

# 3. INSTALL PIP DEPENDENCIES
echo "📦 Installing Libraries into VENV..."
source $SD_VENV/bin/activate
pip install --upgrade pip
pip install aiohttp websockets pymavlink opencv-python-headless pyserial

# 4. YDLIDAR SDK (SKIPPED TO SAVE SPACE)
echo "📡 Skipping YDLidar SDK (Saving Space)..."

# 5. UPDATE LAUNCHER
echo "🚀 Updating Launcher..."
LAUNCHER="$SD_PROJECT/raxda_bridge/run_clean.sh"

# Using SYSTEM PYTHON to bypass SD Card 'noexec' issue
cat <<'EOF' > $LAUNCHER
#!/bin/bash
echo "========================================="
echo "💀 KILLING ZOMBIE PROCESSES on ttyS2..."
sudo fuser -k /dev/ttyS2
echo "✅ Port Cleared."
echo "========================================="
sleep 1
echo "🚀 STARTING RADXA BRIDGE (FROM SD CARD)..."
# Use SYSTEM PYTHON (on eMMC) to run CODE (on SD Card)
sudo /usr/bin/python3 /mnt/sdcard/drone_project/raxda_bridge/real_bridge_service.py
EOF

chmod +x $LAUNCHER

echo "========================================"
echo "✅ MIGRATION COMPLETE!"
echo "New Launch Command:"
echo "sudo $LAUNCHER"
echo "========================================"
