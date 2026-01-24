#!/bin/bash
# fix_launcher.sh
# Manually rewrite the launcher with HARDCODED paths to avoid syntax errors

LAUNCHER="/mnt/sdcard/drone_project/raxda_bridge/run_clean.sh"

echo "🔧 REWRITING LAUNCHER..."

cat <<'EOF' > $LAUNCHER
#!/bin/bash
echo "========================================"
echo "💀 KILLING ZOMBIE PROCESSES on ttyS2..."
sudo fuser -k /dev/ttyS2
echo "✅ Port Cleared."
echo "========================================"
sleep 1
echo "🚀 STARTING RADXA BRIDGE (FROM SD CARD)..."

# HARDCODED PATHS - NO VARIABLES
# Using the Virtual Environment Python to run the Code
sudo /mnt/sdcard/venv/bin/python3 /mnt/sdcard/drone_project/raxda_bridge/real_bridge_service.py
EOF

chmod +x $LAUNCHER
echo "✅ LAUNCHER FIXED."
echo "Run now: sudo $LAUNCHER"
