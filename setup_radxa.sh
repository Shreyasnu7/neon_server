#!/bin/bash
# Radxa Zero 3W Setup Script for Ultra Drone
# Run this on the Radxa: sudo ./setup_radxa.sh

echo ">>> STARTING ULTRA DRONE SETUP <<<"

# 1. Update & Install Deps
echo "[1/4] Installing dependencies..."
sudo apt update
sudo apt install -y python3-pip python3-venv network-manager

# 2. Setup WiFi Hotspot (For ESP32)
echo "[2/4] Configuring WiFi Hotspot 'UltraDrone_Brain'..."
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
MODEM_DEV=$(nmcli device | grep "gsm" | awk '{print $1}')

if [ -z "$MODEM_DEV" ]; then
    echo "   [WARNING] No 4G Modem found. Please plug in your USB Dongle."
    echo "   Skipping 4G setup."
else
    echo "   -> Found Modem at $MODEM_DEV"
    sudo nmcli connection delete 4G_Mobile 2>/dev/null
    # Create Connection (Generic APN 'internet' - change if needed)
    sudo nmcli connection add type gsm ifname "$MODEM_DEV" con-name 4G_Mobile apn internet
    sudo nmcli connection up 4G_Mobile
    echo "   -> 4G Connected"
fi

# 4. Python Environment
echo "[4/4] Setting up Python Environment..."
cd /home/rock/drone_project/ai/camera_brain/laptop_ai
pip3 install -r requirements.txt

echo ">>> SETUP COMPLETE! <<<"
echo "Rebooting in 5 seconds..."
sleep 5
sudo reboot
