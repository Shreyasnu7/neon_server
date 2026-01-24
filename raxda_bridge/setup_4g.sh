#!/bin/bash
# setup_4g.sh - Auto Configure 4G Dongle on Radxa Zero 3W
# Usage: sudo ./setup_4g.sh

echo "📡 Ultra Drone 4G Setup (Airtel/Universal)"
echo "=========================================="

# 1. Install NetworkManager (usually pre-installed)
if ! command -v nmcli &> /dev/null; then
    sudo apt update && sudo apt install -y network-manager
fi

# 2. Check for RNDIS/HiLink Device (usb0 or eth1)
# The user's image (4G UFI) is almost certainly a HiLink RNDIS device.
# It appears as a wired network card, not a serial modem.

echo "🔍 Scanning for USB Network Interfaces..."
# Find interface that is NOT wlan0, lo, or the builtin eth0 (if any)
# Typically usb0 or eth1 on Radxa
IFACE=$(ip link show | grep -E 'usb0|eth1|enx' | awk -F': ' '{print $2}' | head -n1)

if [ -n "$IFACE" ]; then
    echo "✅ Found 4G Hardware Interface: $IFACE"
    
    # 3. Configure NetworkManager for DHCP
    CON_NAME="4G-Dongle-UFI"
    echo "🛠️ Configuring '$CON_NAME'..."
    
    # Delete old configs to ensure fresh start
    sudo nmcli connection delete "$CON_NAME" 2>/dev/null
    
    # Add Ethernet connection (DHCP is default)
    # Priority: 50 (Higher than WiFi's 600)
    sudo nmcli connection add type ethernet ifname "$IFACE" con-name "$CON_NAME" connection.autoconnect yes ipv4.route-metric 50
    
    echo "⏳ activating..."
    sudo nmcli connection up "$CON_NAME"
    
    # 4. Verification
    echo "📊 Connection Status:"
    nmcli connection show "$CON_NAME" | grep -E 'connection.id|IP4.ADDRESS'
    
    echo "🌍 Testing Internet (Airtel)..."
    if ping -c 1 -W 2 8.8.8.8 &> /dev/null; then
        echo "✅ INTERNET CONNECTED! (Ping Success)"
    else
        echo "⚠️  Interface is UP, but No Internet."
        echo "    DIAGNOSIS FOR UFI DONGLES:"
        echo "    1. The Dongle might need APN setup."
        echo "    2. Connect your phone/laptop to the Dongle's WiFi (SSID: 4G-UFI-XX)"
        echo "    3. Go to http://192.168.100.1 (Pass: admin)"
        echo "    4. Set APN to: 'airtelgprs.com'"
        echo "    5. Reboot Dongle."
    fi

else
    echo "⚠️  No USB Network Interface (usb0/eth1) found."
    echo "    1. Is the Y-Cable providing power?"
    echo "    2. Try unplugging/replugging."
    echo "    3. Run 'lsusb' to check if device is detected at all."
    
    # Fallback to ModemManager check just in case it's in Stick Mode
    if command -v mmcli &> /dev/null; then
        MM_IDX=$(mmcli -L | grep -o 'Modem/[0-9]*')
        if [ -n "$MM_IDX" ]; then
             echo "ℹ️  Found Raw Modem mode ($MM_IDX). Setting up as GSM..."
             sudo nmcli connection add type gsm ifname "*" con-name "4G-GSM" apn "airtelgprs.com" connection.autoconnect yes ipv4.route-metric 50
             sudo nmcli connection up "4G-GSM"
        fi
    fi
fi
