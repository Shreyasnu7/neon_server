#!/bin/bash
# fix_launcher_v2.sh
# Fixes "Command not found" by using System Python
# (Bypasses SD Card 'noexec' restriction)

LAUNCHER="/mnt/sdcard/drone_project/raxda_bridge/run_clean.sh"

echo "🔧 REWRITING LAUNCHER (SYSTEM PYTHON)..."

cat <<'EOF' > $LAUNCHER
#!/bin/bash
echo "========================================"
echo "💀 KILLING ZOMBIE PROCESSES on ttyS2..."
sudo fuser -k /dev/ttyS2
echo "✅ Port Cleared."
echo "========================================"
sleep 1
echo "🚀 STARTING RADXA BRIDGE (SD CARD + SYS PYTHON)..."
echo "----------------------------------------"

# Use SYSTEM PYTHON (on eMMC) to run CODE (on SD Card)
# This avoids "command not found" if SD card blocks execution
sudo /usr/bin/python3 /mnt/sdcard/drone_project/raxda_bridge/real_bridge_service.py
EOF

chmod +x $LAUNCHER
echo "✅ LAUNCHER FIXED (V2)."
echo "Run now: sudo $LAUNCHER"
