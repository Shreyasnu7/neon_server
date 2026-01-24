#!/bin/bash
# fix_launcher_final.sh (V2)
# Fixes: "sudo: .../python3: command not found" (noexec issue)
# Fixes: "Specified filename /dev/ttyS2 does not exist" (Missing UART warning)

LAUNCHER="/mnt/sdcard/drone_project/raxda_bridge/run_clean.sh"

echo "🔧 REWRITING LAUNCHER (GOLDEN PATH)..."
rm -f $LAUNCHER

# Use cat EOF with quotes to prevent variable expansion during creation
cat <<'EOF' > $LAUNCHER
#!/bin/bash
echo "========================================="
echo "💀 KILLING ZOMBIE PROCESSES on ttyS2..."

if [ -e /dev/ttyS2 ]; then
    sudo fuser -k /dev/ttyS2
    echo "✅ Port Cleared."
else
    echo "⚠️  CRITICAL WARNING: /dev/ttyS2 NOT FOUND!"
    echo "👉  You MUST run 'enable_uart.sh' and then 'sudo reboot' if you haven't!"
fi

echo "========================================="
sleep 1

echo "🚀 STARTING RADXA BRIDGE..."
echo "ℹ️  Strategy: System Python (eMMC) + SD Card Libraries"

# MAGIC LINE: Use System Python (Executable) but load Libs from SD Card (Clean)
# This bypasses 'noexec' AND keeps eMMC clean.
# We explicitly set PYTHONPATH to the venv site-packages on the SD card.
SITE_PACKAGES="/mnt/sdcard/venv/lib/python3.13/site-packages"

# Executing with sudo env to pass the PYTHONPATH
sudo env PYTHONPATH=$SITE_PACKAGES /usr/bin/python3 /mnt/sdcard/drone_project/raxda_bridge/real_bridge_service.py
EOF

chmod +x $LAUNCHER
echo "✅ LAUNCHER FIXED (GOLDEN VERSION)."
echo "Run now: sudo $LAUNCHER"
