#!/bin/bash
# smart_uart_setup.sh
# Forcefully sets the UART2 overlay, handling comments and duplicates.

echo "🔍 DIAGNOSTIC INFO:"
echo "----------------------"
echo "Existing /dev/ttyS*:"
ls -l /dev/ttyS*
echo "----------------------"
echo "Current /boot/armbianEnv.txt:"
cat /boot/armbianEnv.txt
echo "----------------------"

# Backup
sudo cp /boot/armbianEnv.txt /boot/armbianEnv.txt.bak

echo "🔧 Forcing UART2 Configuration..."

# 1. Remove ANY lines starting with 'overlays=' (prevents duplicates/conflicts)
sudo sed -i '/^overlays=/d' /boot/armbianEnv.txt

# 2. Append the clean, correct line
# We add a newline first to be safe
echo "" | sudo tee -a /boot/armbianEnv.txt
echo "overlays=uart2-m0" | sudo tee -a /boot/armbianEnv.txt

echo "✅ Configuration updated."
echo "New Content:"
grep "overlays" /boot/armbianEnv.txt

echo "⚠️  PLEASE REBOOT NOW. (sudo reboot)"
