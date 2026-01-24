#!/bin/bash
echo "🔧 FIXING UART CONFLICTS (Radxa Zero 3)..."

# 1. Stop the System Console on ttyS2
echo "1. Disabling Serial Console on ttyS2..."
sudo systemctl stop serial-getty@ttyS2
sudo systemctl disable serial-getty@ttyS2
sudo systemctl mask serial-getty@ttyS2

# 2. Check boot overlay (optional, but good to know)
# We assume overlays=uart3-m0 is set in /boot/uEnv.txt

# 3. Fix Permissions
echo "2. Setting Permissions..."
sudo chmod 777 /dev/ttyS2

echo "✅ UART Freed. You may need to REBOOT for it to fully stick, but try running the bridge now."
