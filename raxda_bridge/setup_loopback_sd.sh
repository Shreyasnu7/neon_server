#!/bin/bash
# setup_loopback_sd.sh
# STRATEGY: Loopback Container
# If the SD card partition table is failing, we act like it's just raw storage.
# We create ONE big file, format IT as a disk, and put the python env inside.

echo "========================================="
echo "💿 STARTING LOOPBACK CONTAINER SETUP"
echo "========================================="

SD_MOUNT="/mnt/sdcard"
IMG_FILE="$SD_MOUNT/venv_disk.img"
MOUNT_POINT="$SD_MOUNT/venv_mount" # Temporary mount point for the virtual disk
REAL_VENV="$MOUNT_POINT/venv"

# 1. CREATE 1GB IMAGE FILE
echo "Creating 1GB Virtual Disk Image..."
# Using dd to write zeros. If this fails, the card is literally unwritable.
if ! sudo dd if=/dev/zero of=$IMG_FILE bs=1M count=1024 status=progress; then
    echo "❌ FATAL: Cannot write to SD card. It is read-only or dead."
    exit 1
fi

# 2. FORMAT THE IMAGE
echo "Formatting Virtual Disk..."
sudo mkfs.ext4 -F $IMG_FILE

# 3. MOUNT THE IMAGE
echo "Mounting Virtual Disk..."
sudo mkdir -p $MOUNT_POINT
sudo mount -o loop $IMG_FILE $MOUNT_POINT

# 4. SETUP VENV INSIDE IMAGE
echo "🐍 Creating Python Brain (Inside Virtual Disk)..."
sudo apt-get install -y python3-venv python3-opencv
sudo python3 -m venv $REAL_VENV --system-site-packages

# 5. INSTALL LIBS
echo "📦 Installing Libraries..."
sudo $REAL_VENV/bin/pip install --upgrade pip --break-system-packages
sudo $REAL_VENV/bin/pip install pyserial aiohttp websockets pymavlink opencv-python-headless --break-system-packages

# 6. SETUP LAUNCHER TO USE LOOPBACK
echo "🚀 Updating Launcher..."
LAUNCHER="$SD_MOUNT/drone_project/raxda_bridge/run_clean.sh"

cat <<EOF | sudo tee $LAUNCHER > /dev/null
#!/bin/bash
# Hybrid-Loopback Launcher

# 1. MOUNT SD CARD
sudo mount -o remount,exec /mnt/sdcard 2>/dev/null

# 2. MOUNT VIRTUAL DISK
if ! mountpoint -q $MOUNT_POINT; then
    sudo mount -o loop $IMG_FILE $MOUNT_POINT
fi

echo "========================================="
echo "💿 LAUNCHING FROM VIRTUAL DISK"
echo "========================================="

# Check Hardware
if [ -e /dev/ttyS2 ]; then
    sudo fuser -k /dev/ttyS2
fi

# RUN
sudo $REAL_VENV/bin/python3 /mnt/sdcard/drone_project/raxda_bridge/real_bridge_service.py
EOF

sudo chmod +x $LAUNCHER

echo "========================================="
echo "🎉 LOOPBACK SETUP COMPLETE."
echo "Launch: sudo $LAUNCHER"
echo "========================================="
