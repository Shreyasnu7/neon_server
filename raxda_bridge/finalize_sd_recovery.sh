#!/bin/bash
# finalize_sd_recovery.sh
# RECOVERS THE SYSTEM USING *ONLY* THE SD CARD
# 1. Remounts SD with EXEC permissions (Crucial!)
# 2. Recreates VENV (Required due to new Python 3.13)
# 3. Installs libs to SD Card (Saves eMMC space)

SD_MOUNT="/mnt/sdcard"
SD_VENV="$SD_MOUNT/venv"

echo "========================================="
echo "💾 SD CARD RECOVERY PROTOCOL"
echo "========================================="

# 1. REMOUNT WITH EXEC PERMISSIONS
# This allows running python from the SD card
echo "🔧 Remounting SD Card with EXEC permissions..."
sudo mount -o remount,rw,exec $SD_MOUNT

# 2. REBUILD VENV
# We MUST do this because the OS Python version changed.
# Old venv (from previous OS) is incompatible.
echo "🐍 Rebuilding Virtual Env (Python 3.13)..."
echo "   (This deletes old broken venv and makes a new one)"
sudo rm -rf $SD_VENV
sudo python3 -m venv $SD_VENV --system-site-packages

# 3. INSTALL DEPENDENCIES (TO SD CARD)
# These files go to /mnt/sdcard/venv/lib/... -> NOT eMMC!
echo "📦 Installing Libraries to SD CARD..."
sudo $SD_VENV/bin/pip install --upgrade pip --break-system-packages
sudo $SD_VENV/bin/pip install aiohttp websockets pymavlink opencv-python-headless pyserial --break-system-packages

# 4. RESTORE LAUNCHER
echo "🚀 Updating Launcher..."
LAUNCHER="$SD_MOUNT/drone_project/raxda_bridge/run_clean.sh"

cat <<EOF > $LAUNCHER
#!/bin/bash
# Ensure SD is executable every time we run
sudo mount -o remount,exec $SD_MOUNT 2>/dev/null

echo "========================================="
echo "💀 KILLING ZOMBIE PROCESSES on ttyS2..."
sudo fuser -k /dev/ttyS2
echo "✅ Port Cleared."
echo "========================================="
sleep 1

echo "🚀 STARTING RADXA BRIDGE (PURE SD CARD)..."
# Now we can use the VENV python because we fixed the mount flags
sudo $SD_VENV/bin/python3 $SD_MOUNT/drone_project/raxda_bridge/real_bridge_service.py
EOF

chmod +x $LAUNCHER

echo "========================================="
echo "✅ SD CARD RECOVERY COMPLETE"
echo "Space Used on eMMC will stay low."
echo "Launch with: sudo $LAUNCHER"
echo "========================================="
