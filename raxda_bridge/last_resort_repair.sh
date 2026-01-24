#!/bin/bash
# last_resort_repair.sh
# ATTEMPTS TO SALVAGE A DYING SD CARD
# Uses "Bad Block" checking to mark corrupted areas so we don't write to them.

SD_DEV="/dev/mmcblk1p1"
SD_MOUNT="/mnt/sdcard"
SD_VENV="$SD_MOUNT/venv"

echo "========================================="
echo "🚑 LAST RESORT REPAIR (Searching Bad Blocks)"
echo "   This might take a minute..."
echo "========================================="

# 1. UNMOUNT
sudo umount $SD_DEV 2>/dev/null
sudo umount $SD_MOUNT 2>/dev/null

# 2. RUN DEEP REPAIR (Bad Block Check)
# -c = Check for bad blocks and add to bad block list
# -y = Assume Yes to fixes
# -v = Verbose
echo "🔍 Scanning for physical errors..."
sudo e2fsck -c -y -v $SD_DEV
echo "✅ Scan Complete."

# 3. MOUNT AGAIN
echo "💾 Mounting..."
sudo mkdir -p $SD_MOUNT
sudo mount -o rw,exec $SD_DEV $SD_MOUNT

# 4. RETRY MINIMAL INSTALL
echo "📦 Trying Minimal Install..."
# Try to just create venv without pip first
if [ -d "$SD_VENV" ]; then
    echo "   - Cleaning old venv..." 
    sudo rm -rf $SD_VENV
fi

echo "   - Creating lightweight venv..."
sudo python3 -m venv $SD_VENV --without-pip --system-site-packages

# Manually fetch pip if needed, or rely on system packages if available
# We try to install libraries again, hoping they land on "Good" blocks now.
sudo $SD_VENV/bin/python3 -m ensurepip
sudo $SD_VENV/bin/pip install pyserial aiohttp websockets pymavlink opencv-python-headless --break-system-packages

echo "========================================="
echo "🏁 REPAIR ATTEMPT FINISHED."
echo "If this failed, the card is 100% DEAD."
echo "Launch: sudo /mnt/sdcard/drone_project/raxda_bridge/run_clean.sh"
echo "========================================="
