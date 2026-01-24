#!/bin/bash
# fix_runtime_v2.sh
# Fixes "No module named cv2" and Diagnoses UART
# 1. Installs system-level OpenCV (Reliable fallback)
# 2. Makes launcher robust (Dynamic PYTHONPATH)

echo "🔧 FIXING RUNTIME..."

# 1. INSTALL SYSTEM OPENCV (Fixes cv2 missing)
echo "📦 Installing System OpenCV..."
sudo apt-get update
sudo apt-get install -y python3-opencv

# 2. FIX LAUNCHER (Dynamic Path)
echo "🚀 Updating Launcher to find libraries automatically..."
LAUNCHER="/mnt/sdcard/drone_project/raxda_bridge/run_clean.sh"

cat <<'EOF' | sudo tee $LAUNCHER > /dev/null
#!/bin/bash
sudo mount -o remount,exec /mnt/sdcard 2>/dev/null

echo "========================================="
echo "💀 CHECKING HARDWARE..."
# DEBUG: List all TTYs
ls /dev/ttyS* 2>/dev/null || echo "No ttyS found"

if [ -e /dev/ttyS2 ]; then
    sudo fuser -k /dev/ttyS2
    echo "✅ UART2 Ready."
    SERIAL_PORT="/dev/ttyS2"
elif [ -e /dev/ttyS0 ]; then # Fallback?
    echo "⚠️ UART2 Missing, checking S0..."
    SERIAL_PORT="/dev/ttyS2" # Keep default but warn
else
    echo "❌ CRITICAL: /dev/ttyS2 NOT FOUND."
    echo "   Current overlays: $(grep overlays /boot/armbianEnv.txt)"
fi
echo "========================================="
sleep 0.5

echo "🚀 LAUNCHING DRONE BRIDGE"
# ROBUST: Find the site-packages folder regardless of python version
SITE_PACKAGES=$(find /mnt/sdcard/venv/lib -name "site-packages" -type d | head -n 1)

echo "ℹ️  Libs: $SITE_PACKAGES"
# Run with System Python + SD Card Libs
sudo env PYTHONPATH=$SITE_PACKAGES /usr/bin/python3 /mnt/sdcard/drone_project/raxda_bridge/real_bridge_service.py
EOF
sudo chmod +x $LAUNCHER

echo "========================================="
echo "✅ FIXES APPLIED."
echo "-----------------------------------------"
echo "DIAGNOSTIC INFO:"
echo "UART Devices:"
ls -l /dev/ttyS*
echo "Python Libs on SD:"
find /mnt/sdcard/venv/lib -maxdepth 2 -name "python*"
echo "-----------------------------------------"
echo "RUN NOW: sudo $LAUNCHER"
echo "========================================="
