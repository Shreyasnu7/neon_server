#!/bin/bash
# fix_final_drivers.sh
# Fixes: "UnicodeDecodeError" (CV2) and "UART2 Missing"
# Strategy: 
# 1. Install System Camera Drivers (Stable, on eMMC)
# 2. Find and apply the REAL UART overlay (Dynamically)

echo "========================================="
echo "🔧 INSTALLING DRIVERS & FIXING UART"
echo "========================================="

# 1. OPTIMIZED CAMERA DRIVER (System Install)
# This bypasses the corrupted SD card files.
echo "📸 Updating Camera Drivers..."
sudo apt-get update
sudo apt-get install -y python3-opencv python3-numpy python3-serial

# 2. INTELLIGENT UART REPAIR
echo "🔌 Detecting UART Overlay..."
BOOT_DIR="/boot/dtb/rockchip/overlay"
# Find the overlay file that matches *uart2*
OVERLAY_FILE=$(find $BOOT_DIR -name "*uart2-m0.dtbo" | head -n 1)

if [ -z "$OVERLAY_FILE" ]; then
    echo "⚠️  Could not find specific 'uart2-m0' overlay."
    echo "    Trying generic 'uart2'..."
    OVERLAY_NAME="uart2-m0" # Default fallback
else
    # Extract filename without path and extension
    OVERLAY_NAME=$(basename $OVERLAY_FILE .dtbo)
    echo "✅ Found Overlay: $OVERLAY_NAME"
fi

# Apply to Boot Config
ENV_FILE="/boot/armbianEnv.txt"
if grep -q "overlays=.*$OVERLAY_NAME" "$ENV_FILE"; then
    echo "   - Config already correct."
else
    echo "   - Applying fix to $ENV_FILE..."
    # Remove old overlay lines
    sudo sed -i '/overlays=/d' $ENV_FILE
    # Add new correct one
    echo "overlays=$OVERLAY_NAME" | sudo tee -a "$ENV_FILE"
fi

# 3. UPDATE LAUNCHER (To use new Drivers)
echo "🚀 Optimizing Launcher..."
LAUNCHER="/mnt/sdcard/drone_project/raxda_bridge/run_clean.sh"

cat <<EOF | sudo tee $LAUNCHER > /dev/null
#!/bin/bash
# Driver-Optimized Launcher
sudo mount -o remount,exec /mnt/sdcard 2>/dev/null

echo "========================================="
echo "💀 CHECKING HARDWARE..."
if [ -e /dev/ttyS2 ]; then
    sudo fuser -k /dev/ttyS2
    echo "✅ UART2 Ready."
else
    echo "⚠️  UART2 Missing. (Reboot required for new settings)"
fi
echo "========================================="

echo "🚀 LAUNCHING DRONE BRIDGE"
# We add /usr/lib/python3/dist-packages to PYTHONPATH to ensure we pick up the system OpenCV
export PYTHONPATH="/usr/lib/python3/dist-packages:/mnt/sdcard/venv_mount/venv/lib/python3.13/site-packages:\$PYTHONPATH"

# Run with System Python
sudo -E /usr/bin/python3 /mnt/sdcard/drone_project/raxda_bridge/real_bridge_service.py
EOF
sudo chmod +x $LAUNCHER

echo "========================================="
echo "✅ UPDATES COMPLETE."
echo "1. REBOOT: sudo reboot"
echo "2. LAUNCH: sudo $LAUNCHER"
echo "========================================="
