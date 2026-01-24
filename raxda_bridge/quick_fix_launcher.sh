#!/bin/bash
# quick_fix_launcher.sh
# Fixes "Syntax error: Unterminated quoted string" in run_clean.sh
# Does NOT re-install libraries (Fast Fix)

LAUNCHER="/mnt/sdcard/drone_project/raxda_bridge/run_clean.sh"
echo "🔧 QUICK FIXING LAUNCHER..."

# Use tee to safely write as root without quoting issues
cat <<'EOF' | sudo tee $LAUNCHER > /dev/null
#!/bin/bash
# Force exec permission
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
# STRATEGY: System Python (eMMC) + SD Card Libraries
SITE_PACKAGES="/mnt/sdcard/venv/lib/python3.13/site-packages"
sudo env PYTHONPATH=$SITE_PACKAGES /usr/bin/python3 /mnt/sdcard/drone_project/raxda_bridge/real_bridge_service.py
EOF

sudo chmod +x $LAUNCHER
echo "✅ LAUNCHER REWRITTEN."
echo "-----------------------------------------"
echo "TRY Launching NOW: sudo $LAUNCHER"
echo "-----------------------------------------"
