#!/bin/bash
# restore_launcher.sh
# Fixes "Syntax error: ) unexpected"
# The deep repair probably corrupted the old launcher file. This writes a fresh one.

LAUNCHER="/mnt/sdcard/drone_project/raxda_bridge/run_clean.sh"
echo "🔧 RESTORING LAUNCHER..."

# 1. Delete the bad file
sudo rm -f $LAUNCHER

# 2. Write fresh content using printf (Cleanest method)
# Using a temp file first to ensure atomic write
TMP="/tmp/launcher_fresh.sh"

printf "#!/bin/bash\n" > $TMP
printf "# Launcher Restored after Repair\n" >> $TMP
printf "sudo mount -o remount,exec /mnt/sdcard 2>/dev/null\n" >> $TMP
printf "\n" >> $TMP
printf "echo \"=========================================\"\n" >> $TMP
printf "echo \"💀 CHECKING HARDWARE...\"\n" >> $TMP
printf "if [ -e /dev/ttyS2 ]; then\n" >> $TMP
printf "    sudo fuser -k /dev/ttyS2\n" >> $TMP
printf "    echo \"✅ UART2 Ready.\"\n" >> $TMP
printf "else\n" >> $TMP
printf "    echo \"⚠️  UART2 Missing.\"\n" >> $TMP
printf "fi\n" >> $TMP
printf "echo \"=========================================\"\n" >> $TMP
printf "sleep 0.5\n" >> $TMP
printf "echo \"🚀 LAUNCHING DRONE BRIDGE\"\n" >> $TMP
printf "SITE_PACKAGES=\$(find /mnt/sdcard/venv/lib -name \"site-packages\" -type d | head -n 1)\n" >> $TMP
printf "sudo env PYTHONPATH=\$SITE_PACKAGES /usr/bin/python3 /mnt/sdcard/drone_project/raxda_bridge/real_bridge_service.py\n" >> $TMP

# 3. Move into place
sudo mv $TMP $LAUNCHER
sudo chmod +x $LAUNCHER

echo "✅ LAUNCHER RESTORED."
echo "Run: sudo $LAUNCHER"
