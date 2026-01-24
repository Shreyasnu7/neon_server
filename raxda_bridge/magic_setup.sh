#!/bin/bash
# magic_setup.sh (V4)
# THE "ONE CLICK" RECOVERY SCRIPT
# V4 Update:
# 1. Installs 'python3-venv' (Fixes ensurepip error)
# 2. Adds SD Card to /etc/fstab (Fixes "Command not found" after reboot)

echo "========================================="
echo "🧙 MAGIC SETUP V4: PERSISTENCE & PACKAGES"
echo "========================================="

USER_HOME="/home/shreyash"
SD_MOUNT="/mnt/sdcard"
SD_VENV="$SD_MOUNT/venv"
SD_DEV="/dev/mmcblk1p1" 

# 0. SYSTEM PACKAGES (CRITICAL FIX)
echo "📦 Installing System Requirements..."
sudo apt-get update
# Fixes "virtual environment was not created successfully"
sudo apt-get install -y python3-venv python3.13-venv python3-pip

# 1. REPAIR & MOUNT SD CARD
echo "❤️‍🩹 Repairing SD Card..."
sudo umount $SD_DEV 2>/dev/null
sudo umount $SD_MOUNT 2>/dev/null
sudo fsck -y $SD_DEV
echo "✅ Filesystem Repaired."

echo "💾 Mounting SD Card..."
if [ ! -d "$SD_MOUNT" ]; then
    sudo mkdir -p $SD_MOUNT
fi
sudo mount -o rw,exec $SD_DEV $SD_MOUNT

# 1.1 MAKE PERSISTENT (Fixes reboot issue)
echo "🔗 Configuring Auto-Mount (fstab)..."
if ! grep -q "$SD_MOUNT" /etc/fstab; then
    echo "   - Adding to /etc/fstab..."
    echo "$SD_DEV $SD_MOUNT auto defaults,noatime,rw,exec 0 0" | sudo tee -a /etc/fstab
    echo "✅ Auto-mount enabled."
else
    echo "   - Already in fstab."
fi

# 2. FIX BOOT CONFIG (UART2)
echo "🔧 Checking Boot Config..."
ENV_FILE="/boot/armbianEnv.txt"
if [ -f "$ENV_FILE" ]; then
    sudo cp $ENV_FILE $ENV_FILE.bak
    sudo sed -i '/overlays=/d' $ENV_FILE
    echo "overlays=uart2-m0" | sudo tee -a $ENV_FILE > /dev/null
    sudo sed -i '/^$/d' $ENV_FILE
    echo "✅ Boot Config Fixed."
else
    echo "⚠️ Warning: $ENV_FILE not found (Skipping UART fix)."
fi

# 3. SETUP PYTHON BRAIN (ON SD CARD)
echo "🐍 Setting up Python Brain..."
if [ -d "$SD_VENV" ]; then
     sudo rm -rf $SD_VENV
fi
# Rebuilding VENV
sudo python3 -m venv $SD_VENV --system-site-packages
echo "✅ Venv Created Success."

# 4. INSTALL DEPENDENCIES
echo "📦 Installing Libraries..."
sudo $SD_VENV/bin/pip install --upgrade pip --break-system-packages
sudo $SD_VENV/bin/pip install pyserial aiohttp websockets pymavlink opencv-python-headless --break-system-packages
echo "✅ Libraries Installed."

# 5. WRITE THE LAUNCHER
echo "🚀 Creating Launcher..."
LAUNCHER="$SD_MOUNT/drone_project/raxda_bridge/run_clean.sh"
sudo mkdir -p $(dirname $LAUNCHER)

cat <<'EOF' | sudo tee $LAUNCHER > /dev/null
#!/bin/bash
# Fail-safe remount (in case fstab fails)
sudo mount -o remount,exec /mnt/sdcard 2>/dev/null

echo "========================================="
echo "💀 CHECKING HARDWARE..."
if [ -e /dev/ttyS2 ]; then
    sudo fuser -k /dev/ttyS2
    echo "✅ UART2 Ready."
else
    echo "⚠️  WAIT! UART2 Missing."
fi
echo "========================================="
sleep 0.5
echo "🚀 LAUNCHING DRONE BRIDGE"
SITE_PACKAGES="/mnt/sdcard/venv/lib/python3.13/site-packages"
sudo env PYTHONPATH=$SITE_PACKAGES /usr/bin/python3 /mnt/sdcard/drone_project/raxda_bridge/real_bridge_service.py
EOF
sudo chmod +x $LAUNCHER

echo "========================================="
echo "🎉 SETUP COMPLETE."
echo "1. REBOOT NOW: sudo reboot"
echo "2. THEN LAUNCH: sudo $LAUNCHER"
echo "========================================="
