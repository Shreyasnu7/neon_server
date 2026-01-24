#!/bin/bash
# Radxa Zero 3W Setup Script for Ultra Drone
# Run this on the Radxa: sudo ./setup_radxa.sh

echo ">>> STARTING ULTRA DRONE SETUP <<<"

# 1. Update & Install Deps
echo "[1/4] Installing System dependencies..."
sudo apt update
sudo apt install -y python3-pip python3-venv network-manager

# 2. Python Environment (Install BEFORE switching network modes to ensure we have Internet)
echo "[2/4] Installing Python Libraries..."

# Robustly find the script directory (works with sudo)
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR/ai/camera_brain/laptop_ai"

# Use --break-system-packages because we are an embedded appliance
pip3 install -r requirements.txt --break-system-packages || echo "âš ï¸ Pip warning (check network)"

# 3. Setup WiFi Hotspot (Disconnects Home WiFi!)
echo "[3/4] Configuring WiFi Hotspot 'UltraDrone_Brain'..."
# Delete if exists to avoid dupes
sudo nmcli connection delete UltraDrone_Brain 2>/dev/null
sudo nmcli connection delete Hotspot 2>/dev/null

# Create Hotspot
sudo nmcli dev wifi hotspot ifname wlan0 ssid UltraDrone_Brain password password123
# Set Static IP (Gateway)
sudo nmcli connection modify Hotspot ipv4.addresses 192.168.4.1/24 ipv4.method shared
sudo nmcli connection up Hotspot
echo "   -> Hotspot Created at 192.168.4.1"

# 3. Setup 4G Dongle (Auto-Detect)
echo "[3/4] Configuring 4G Dongle..."

# Wait up to 10 seconds for modem to appear
for i in {1..10}; do
    MODEM_DEV=$(nmcli device | grep "gsm" | awk '{print $1}')
    if [ ! -z "$MODEM_DEV" ]; then
        break
    fi
    echo "   ...waiting for modem ($i/10)"
    sleep 1
done

if [ -z "$MODEM_DEV" ]; then
    echo "   [WARNING] No 4G Modem found! Is it plugged in?"
    echo "   Skipping 4G setup. Internet may not work."
else
    echo "   -> Found Modem at $MODEM_DEV"
    sudo nmcli connection delete 4G_Mobile 2>/dev/null
    # Generic APN 'internet' (Works for most, change 'internet' to your provider if needed)
    sudo nmcli connection add type gsm ifname "$MODEM_DEV" con-name 4G_Mobile apn internet
    sudo nmcli connection up 4G_Mobile
    echo "   -> 4G Connection Enabled."
fi

# Internet Verification
echo "   [TEST] Checking Internet..."
if ping -c 1 8.8.8.8 &> /dev/null; then
    echo "   âœ… INTERNET CONNECTED!"
else
    echo "   âŒ NO INTERNET. Check your 4G SIM/APN."
fi

# 4. FILESYSTEM SAFETY (Power-Cut Protection)
echo "[4/4] Installing Overlayroot (Corruption Protection)..."
# Prevents SD card corruption by writing changes to RAM instead of disk
sudo apt-get install -y overlayroot
# Configure to be enabled (default is disabled)
# We set it to 'tmpfs' mode. To edit files later, run 'sudo overlayroot-chroot'
echo 'overlayroot="tmpfs"' | sudo tee /etc/overlayroot.conf
echo "   âœ… Overlayroot Configured. Filesystem will be Read-Only after reboot."
echo "   ðŸ‘‰ To make permanent changes later, run: sudo overlayroot-chroot"

# 5. AUTO-START SERVICE (Embedded)
echo "[5/5] Creating & Installing Auto-Start Service..."

# Force Create the file
sudo cat <<EOF > /etc/systemd/system/drone_bridge.service
[Unit]
Description=Ultra Drone Bridge Service
After=network.target

[Service]
Type=simple
User=shreyash
WorkingDirectory=/mnt/sd_storage/drone_project
ExecStart=/usr/bin/python3 raxda_bridge/real_bridge_service.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable drone_bridge.service
sudo systemctl start drone_bridge.service
echo "   ✅ Service Created, Installed & Started!"

echo ">>> SETUP COMPLETE! <<<"
echo "Rebooting in 5 seconds..."
sleep 5
sudo reboot
